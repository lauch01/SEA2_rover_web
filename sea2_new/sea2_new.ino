#include <Arduino.h>

// ========== micro-ROS ==========
#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>

// ================== ENCODERS ==================
#define ENC1_A PA11
#define ENC1_B PA8

#define ENC2_A PA9
#define ENC2_B PA10

volatile long ticks1 = 0;
volatile long ticks2 = 0;

volatile uint32_t last1_us = 0;
volatile uint32_t last2_us = 0;
const uint32_t MIN_PULSE_US = 80;

// ================== MOTORES (L298N) ==================
#define IN1 PB3   // (ANTES ERA PA3)
#define IN2 PA4
#define IN3 PA5
#define IN4 PA6

#define ENA PB0
#define ENB PB1

// ================== Serial para micro-ROS ==================
#define MROS Serial2   // USART2 físico (PA2 TX2 / PA3 RX2)

// ================== Control motores ==================
int speedPWM_max = 200;          
const int PWM_MIN = 0;
const int PWM_MAX = 255;

volatile uint32_t last_cmd_ms = 0;
const uint32_t CMD_TIMEOUT_MS = 500;

// ================== micro-ROS: transporte serial custom ==================
bool transport_open(struct uxrCustomTransport * transport) {
  (void) transport;
  MROS.begin(115200);
  delay(50);
  return true;
}

bool transport_close(struct uxrCustomTransport * transport) {
  (void) transport;
  // MROS.end(); // si el core soporta end()
  return true;
}

size_t transport_write(struct uxrCustomTransport* transport, const uint8_t* buf, size_t len, uint8_t* err) {
  (void) transport;
  (void) err;
  return MROS.write(buf, len);
}

size_t transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout_ms, uint8_t* err) {
  (void) transport;
  (void) err;

  uint32_t start = millis();
  size_t read_len = 0;

  while ((millis() - start) < (uint32_t)timeout_ms && read_len < len) {
    int c = MROS.read();
    if (c >= 0) {
      buf[read_len++] = (uint8_t)c;
    } else {
      // delay para no quemar CPU
      delayMicroseconds(200);
    }
  }
  return read_len;
}

// ================== ISR ENCODERS ==================
void isrEnc1A() {
  uint32_t now = micros();
  if ((uint32_t)(now - last1_us) < MIN_PULSE_US) return;
  last1_us = now;

  int b = digitalRead(ENC1_B);
  if (b == HIGH) ticks1--;
  else ticks1++;
}

void isrEnc2A() {
  uint32_t now = micros();
  if ((uint32_t)(now - last2_us) < MIN_PULSE_US) return;
  last2_us = now;

  int b = digitalRead(ENC2_B);
  if (b == HIGH) ticks2--;
  else ticks2++;
}

// ================== Motores ==================
void motorLeft(int pwm) {
  int p = constrain(abs(pwm), PWM_MIN, PWM_MAX);

  if (pwm > 0) { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); }
  else if (pwm < 0) { digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); }
  else { digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); }

  analogWrite(ENA, p);
}

void motorRight(int pwm) {
  int p = constrain(abs(pwm), PWM_MIN, PWM_MAX);

  if (pwm > 0) { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); }
  else if (pwm < 0) { digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); }
  else { digitalWrite(IN3, LOW); digitalWrite(IN4, LOW); }

  analogWrite(ENB, p);
}

void stopMotors() {
  motorLeft(0);
  motorRight(0);
}

// ================== micro-ROS Node ==================
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

rcl_publisher_t pub_left;
rcl_publisher_t pub_right;

rcl_subscription_t sub_cmdvel;
geometry_msgs__msg__Twist msg_cmdvel;

rcl_timer_t timer_pub;
rclc_executor_t executor;

std_msgs__msg__Int32 msg_left;
std_msgs__msg__Int32 msg_right;

// Convierte cmd_vel a PWM diferencial
void cmdvel_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * m = (const geometry_msgs__msg__Twist *)msgin;

  // Normalización simple: asume linear.x y angular.z en rango aproximado [-1..1]
  float v = (float)m->linear.x;   // avance
  float w = (float)m->angular.z;  // giro

  // Mezcla diferencial (tank)
  float left  = v - w;
  float right = v + w;

  // Saturar
  if (left  >  1.0f) left  =  1.0f;
  if (left  < -1.0f) left  = -1.0f;
  if (right >  1.0f) right =  1.0f;
  if (right < -1.0f) right = -1.0f;

  int pwmL = (int)(left  * speedPWM_max);
  int pwmR = (int)(right * speedPWM_max);

  motorLeft(pwmL);
  motorRight(pwmR);

  last_cmd_ms = millis();
}

// Publica ticks periódicamente
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  (void) last_call_time;
  if (timer == NULL) return;

  long t1, t2;
  noInterrupts();
  t1 = ticks1;
  t2 = ticks2;
  interrupts();

  msg_left.data = (int32_t)t1;
  msg_right.data = (int32_t)t2;

  rcl_publish(&pub_left, &msg_left, NULL);
  rcl_publish(&pub_right, &msg_right, NULL);

  // watchdog: si no llega cmd_vel, parar
  if ((millis() - last_cmd_ms) > CMD_TIMEOUT_MS) {
    stopMotors();
  }
}

void setup() {
  // ---- IO encoders ----
  pinMode(ENC1_A, INPUT_PULLUP);
  pinMode(ENC1_B, INPUT_PULLUP);
  pinMode(ENC2_A, INPUT_PULLUP);
  pinMode(ENC2_B, INPUT_PULLUP);

  int int1 = digitalPinToInterrupt(ENC1_A);
  int int2 = digitalPinToInterrupt(ENC2_A);

  attachInterrupt(int1, isrEnc1A, RISING);
  attachInterrupt(int2, isrEnc2A, RISING);

  // ---- IO motores ----
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  stopMotors();

  // -------- micro-ROS transport por Serial2 --------
  rmw_uros_set_custom_transport(
    true,
    (void *) &MROS,
    transport_open,
    transport_close,
    transport_write,
    transport_read
  );

  // -------- micro-ROS init --------
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);

  rclc_node_init_default(&node, "stm32_l432kc_rover", "", &support);

  // Publishers
  rclc_publisher_init_default(
    &pub_left,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "left_ticks"
  );

  rclc_publisher_init_default(
    &pub_right,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "right_ticks"
  );

  // Subscriber cmd_vel
  rclc_subscription_init_default(
    &sub_cmdvel,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"
  );

  // Timer 20 Hz (50ms)
  const unsigned int period_ms = 50;
  rclc_timer_init_default(
    &timer_pub,
    &support,
    RCL_MS_TO_NS(period_ms),
    timer_callback
  );

  // Executor
  rclc_executor_init(&executor, &support.context, 2, &allocator);
  rclc_executor_add_subscription(&executor, &sub_cmdvel, &msg_cmdvel, &cmdvel_callback, ON_NEW_DATA);
  rclc_executor_add_timer(&executor, &timer_pub);

  last_cmd_ms = millis();
}

void loop() {
  // Spin del executor
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
}
