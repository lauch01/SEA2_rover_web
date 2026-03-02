import math
import time
import serial

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry


def xor_checksum(s: str) -> int:
    c = 0
    for ch in s:
        c ^= ord(ch)
    return c & 0xFF


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw * 0.5)
    q.z = math.sin(yaw * 0.5)
    q.x = 0.0
    q.y = 0.0
    return q


class STM32Bridge(Node):
    """
    Bridge ROS2 <-> STM32 por serial.

    RX STM32: <M,spL,spR,CC>\n  con spL/spR en [-255..255]
    TX STM32:
      <T,tL,tR,CC>\n  (ticks acumulados) [debug opcional]
      <O,x,y,th,v,w,CC>\n  odom
    """

    def __init__(self):
        super().__init__('stm32_bridge')

        # -------- Parámetros --------
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('wheel_base', 0.15)       # m (para conversión cmd_vel->ruedas)
        self.declare_parameter('vmax', 0.6)              # m/s aproximado cuando PWM=255 (calibrable)
        self.declare_parameter('cmd_timeout_ms', 200)    # watchdog desde ROS
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')

        port = self.get_parameter('port').value
        baud = int(self.get_parameter('baud').value)

        # -------- Serial --------
        self.ser = serial.Serial(port, baudrate=baud, timeout=0.01)
        self.get_logger().info(f"Abierto serial {port} @ {baud}")

        # -------- ROS pubs/subs --------
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # -------- Estado --------
        self.last_cmd_time = time.time()
        self.cmd_timeout = float(self.get_parameter('cmd_timeout_ms').value) / 1000.0

        self.rx_buf = ""

        # último tick debug (si quieres usarlo después)
        self.last_ticks = None  # (tL, tR)

        # Loop lectura / watchdog (100 Hz)
        self.timer = self.create_timer(0.01, self.tick)

    # ------------------ TX: cmd_vel -> PWM ------------------
    def on_cmd(self, msg: Twist):
        """
        Convierte cmd_vel a setpoints de rueda y luego a PWM [-255..255].
        """
        B = float(self.get_parameter('wheel_base').value)
        VMAX = float(self.get_parameter('vmax').value)

        v = float(msg.linear.x)
        w = float(msg.angular.z)

        # cinemática diferencial
        vL = v - w * (B / 2.0)
        vR = v + w * (B / 2.0)

        # Normalizar con VMAX -> PWM
        # clamp [-1,1]
        nL = max(min(vL / VMAX, 1.0), -1.0) if VMAX > 1e-6 else 0.0
        nR = max(min(vR / VMAX, 1.0), -1.0) if VMAX > 1e-6 else 0.0

        spL = int(round(nL * 255.0))
        spR = int(round(nR * 255.0))

        self.send_motor_command(spL, spR)
        self.last_cmd_time = time.time()

    def send_motor_command(self, spL: int, spR: int):
        spL = max(min(spL, 255), -255)
        spR = max(min(spR, 255), -255)

        body = f"M,{spL},{spR}"
        cs = xor_checksum(body)
        frame = f"<{body},{cs:02X}>\n"
        self.ser.write(frame.encode('ascii', errors='ignore'))

    # ------------------ Loop: watchdog + RX ------------------
    def tick(self):
        # Watchdog ROS: si no llegan cmd_vel -> manda stop
        if (time.time() - self.last_cmd_time) > self.cmd_timeout:
            self.send_motor_command(0, 0)
            self.last_cmd_time = time.time()

        # Leer serial
        data = self.ser.read(256)
        if not data:
            return

        try:
            self.rx_buf += data.decode('ascii', errors='ignore')
        except Exception:
            return

        # Extraer frames < ... >
        while True:
            start = self.rx_buf.find('<')
            if start == -1:
                # recorta buffer si se llenó de basura
                if len(self.rx_buf) > 512:
                    self.rx_buf = self.rx_buf[-256:]
                return

            end = self.rx_buf.find('>', start + 1)
            if end == -1:
                # frame incompleto, esperar más datos
                if start > 0:
                    self.rx_buf = self.rx_buf[start:]
                return

            payload = self.rx_buf[start + 1:end].strip()
            self.rx_buf = self.rx_buf[end + 1:]

            self.handle_frame(payload)

    # ------------------ RX: parse frames ------------------
    def handle_frame(self, payload: str):
        # payload esperado: "X,...,CC"
        parts = payload.split(',')
        if len(parts) < 2:
            return

        rx_cs = parts[-1]
        body = ",".join(parts[:-1])

        # validar checksum
        if len(rx_cs) != 2:
            return
        try:
            rx = int(rx_cs, 16)
        except ValueError:
            return

        calc = xor_checksum(body)
        if rx != calc:
            return

        ftype = parts[0]

        # <T,tL,tR,CC>
        if ftype == 'T' and len(parts) == 4:
            try:
                tL = int(parts[1])
                tR = int(parts[2])
            except ValueError:
                return
            self.last_ticks = (tL, tR)
            return

        # <O,x,y,th,v,w,CC>
        if ftype == 'O' and len(parts) == 7:
            try:
                x_mm = int(parts[1])
                y_mm = int(parts[2])
                th_mrad = int(parts[3])
                v_mmps = int(parts[4])
                w_mradps = int(parts[5])
            except ValueError:
                return

            x = x_mm / 1000.0
            y = y_mm / 1000.0
            th = th_mrad / 1000.0
            v = v_mmps / 1000.0
            w = w_mradps / 1000.0

            self.publish_odom(x, y, th, v, w)
            return


    def publish_odom(self, x: float, y: float, th: float, v: float, w: float):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = str(self.get_parameter('frame_id').value)
        odom.child_frame_id = str(self.get_parameter('child_frame_id').value)

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = yaw_to_quat(th)

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w

        self.odom_pub.publish(odom)


def main():
    rclpy.init()
    node = STM32Bridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
