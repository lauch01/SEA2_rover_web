# SEA2_rover_web
# Rover Web Teleoperation + micro-ROS (STM32L432KC + RPi4 + ROS2 Humble)

Este repositorio/documentación describe **paso a paso** cómo levantar un rover (TI-RSLK/Pololu) con:
- **STM32L432KC** corriendo **micro-ROS** (motores + encoders).
- **Raspberry Pi 4** con **Ubuntu Desktop + ROS2 Humble** actuando como:
  - micro-ROS Agent (XRCE-DDS ↔ ROS2)
  - Odometry node (ticks → distance/odom + reset)
  - rosbridge (WebSocket para web UI)
  - Web server (interfaz HTML)
  - Streaming de cámara USB (mjpg-streamer)

Incluye **comandos de instalación**, **diagnóstico**, **cómo encender todo**, y **qué hacer si falla** (incluye caso típico donde “solo se arregla con reboot”).

---

## 0) Arquitectura (qué hace cada pieza)

### STM32L432KC (micro-ROS)
- **SUB**: `/cmd_vel` (`geometry_msgs/Twist`) → controla motores diferencial (PWM).
- **PUB**: `/left_ticks` y `/right_ticks` (`std_msgs/Int32`) → telemetría encoders.

> Nota: micro-ROS envía datos **binarios** por UART (XRCE). Si abres ese UART en Serial Monitor verás “caracteres raros”. Es normal.

### Raspberry Pi 4 (ROS2 Humble)
- **micro-ROS Agent**: conecta al STM32 por serial y crea los tópicos ROS2.
- **rover_odometry**: convierte ticks → `/distance` y `/odom`, y atiende `/reset_odometry`.
- **rosbridge_websocket**: sirve WebSocket `ws://<IP>:9090` para la web.
- **Web UI**: página en `http://<IP>:8000`.
- **Cámara USB**: stream MJPEG en `http://<IP>:8080/?action=stream`.

---

## 1) Requisitos

### Hardware
- STM32L432KC (Nucleo/board equivalente)
- Driver motores (ej. L298N) + motores Pololu
- Encoders TI-RSLK / Pololu
- Raspberry Pi 4 con Ubuntu Desktop
- Webcam USB
- Cable USB para STM32 hacia Raspberry Pi
- (Opcional) Cable USB extra si usas conversor USB-Serial aparte

### Software
- ROS2 Humble instalado en la Raspberry Pi
- Docker (si ejecutas el Agent con container)
- Arduino IDE + core STM32 + librería `micro_ros_arduino`

---

## 2) Parámetros del rover (usados en odometría)

- `ticks_per_rev = 360` (lectura actual: **solo canal A en RISING**)
- `wheel_diameter = 0.070 m` (70 mm)
- `wheel_base = 0.141–0.142 m` (distancia centro-centro ruedas)

---

## 3) STM32: micro-ROS (Arduino IDE)

### 3.1 Instalación (Arduino IDE)
1. Instalar **STM32 MCU based boards** en Boards Manager.
2. Instalar librería **micro_ros_arduino** (Library Manager).

### 3.2 UART usado para micro-ROS (IMPORTANTE)
En STM32L432KC se usó:

- `Serial2` = USART2 físico  
  - TX2 = **PA2**
  - RX2 = **PA3**

### 3.3 Cargar sketch micro-ROS
- Selecciona tu board STM32.
- Compila y sube.

> Si el Serial Monitor muestra bytes raros a 115200: es XRCE binario, es correcto.

---

## 4) Raspberry Pi: conectar STM32 y detectar puerto serial

Conecta el STM32 por USB a la Raspberry.

### 4.1 Identificar el dispositivo
```bash
ls -l /dev/ttyACM* /dev/ttyUSB*




###########3
EXPLICACION A DETALLE :

# 🚀 ROS2 + micro-ROS Rover System  
**STM32L432KC + Raspberry Pi 4 + Web Teleoperation + USB Camera**

---

# 📌 Project Overview

This project integrates:

- 🧠 **STM32L432KC** running micro-ROS (motor control + encoder acquisition)
- 🐧 **Raspberry Pi 4 (Ubuntu + ROS 2 Humble)**
- 🌐 Web-based control interface (virtual joystick + telemetry)
- 📷 USB webcam live streaming
- 📍 Differential-drive odometry

The rover can be controlled entirely from a browser while streaming real-time telemetry and video.

---

# 🧠 System Architecture

```
┌────────────────────────────┐
│        Browser UI          │
│  (Joystick + Telemetry)    │
└──────────────┬─────────────┘
               │ WebSocket
               ▼
┌────────────────────────────┐
│     rosbridge_server       │
│  (ROS2 ↔ WebSocket JSON)   │
└──────────────┬─────────────┘
               │ ROS2 DDS
               ▼
┌────────────────────────────┐
│     Odometry Node          │
│ (ticks → pose estimation)  │
└──────────────┬─────────────┘
               │ ROS2 DDS
               ▼
┌────────────────────────────┐
│    micro-ROS Agent         │
│ (DDS ↔ Serial XRCE-DDS)    │
└──────────────┬─────────────┘
               │ Serial (115200)
               ▼
┌────────────────────────────┐
│        STM32L432KC         │
│  micro-ROS firmware        │
│  - Motor control           │
│  - Encoder reading         │
└────────────────────────────┘
```

---

# 🏗 Workspace Structure

```
ros2_ws/
 ├── src/
 │   ├── rover_odometry/
 │   ├── rover_bringup/
 │   └── (future packages)
 ├── build/
 ├── install/
 └── log/
```

---

# 🛠 STEP 1 — Create ROS2 Workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash
```

Optional automatic sourcing:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### What is the workspace?

The workspace is the ROS2 project directory that contains:

- Source code (`src`)
- Build artifacts (`build`)
- Installed executables (`install`)
- Logs (`log`)

---

# 📦 STEP 2 — Install Required ROS2 Packages

```bash
sudo apt update
sudo apt install -y \
  ros-humble-rosbridge-server \
  ros-humble-nav-msgs \
  ros-humble-tf2-ros
```

### Why these packages?

| Package | Purpose |
|----------|----------|
| rosbridge_server | Allows browser to communicate with ROS2 via WebSocket |
| nav_msgs | Required for publishing `/odom` |
| tf2_ros | Required for broadcasting TF transforms |

---

# 📦 STEP 3 — Create `rover_odometry` Package

```bash
cd ~/ros2_ws/src
ros2 pkg create rover_odometry \
  --build-type ament_python \
  --dependencies rclpy std_msgs nav_msgs geometry_msgs tf2_ros
```

---

## 📄 What `rover_odometry` Contains

### Main Node: `odometry_node.py`

This node is responsible for:

- Subscribing to:
  - `/left_ticks`
  - `/right_ticks`
  - `/reset_odometry`
- Computing:
  - Wheel displacement from encoder ticks
  - Differential-drive kinematics
- Publishing:
  - `/distance` → accumulated linear distance
  - `/odom` → robot pose and velocity
- Broadcasting:
  - TF transform (`odom → base_link`)

### Robot Parameters Used

- `ticks_per_rev = 360`
- `wheel_diameter = 0.07 m`
- `wheel_base = 0.142 m`

This node transforms encoder measurements into meaningful position estimation.

---

# 📦 STEP 4 — Create `rover_bringup` Package

```bash
cd ~/ros2_ws/src
ros2 pkg create rover_bringup \
  --build-type ament_python \
  --dependencies launch launch_ros
mkdir -p rover_bringup/launch
```

---

## 📄 What `rover_bringup` Contains

### Launch File: `rover.launch.py`

This launch file automatically starts:

1. `rover_odometry/odometry_node`
2. `rosbridge_server/rosbridge_websocket`

It allows starting the entire ROS side of the system with one command.

---

# 🔨 STEP 5 — Build the Workspace

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

# 🔌 STEP 6 — Start micro-ROS Agent

```bash
docker run -it --rm --net=host --device=/dev/ttyACM0 \
  microros/micro-ros-agent:humble \
  serial --dev /dev/ttyACM0 -b 115200 -v6
```

### What it does

- Reads XRCE-DDS messages from STM32 via serial
- Converts them into ROS2 DDS
- Makes STM32 topics visible inside ROS2

Without this bridge, ROS2 cannot detect the microcontroller.

---

# 🚀 STEP 7 — Launch ROS System

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch rover_bringup rover.launch.py
```

This starts:

- Odometry node
- rosbridge server on port 9090

WebSocket address:

```
ws://<raspberry_ip>:9090
```

---

# 🌐 STEP 8 — Start Web Interface

```bash
cd ~/Documents/rover_web
python3 -m http.server 8000
```

Open browser:

```
http://<raspberry_ip>:8000
```

---

# 📷 STEP 9 — USB Camera Streaming

## Install dependencies

```bash
sudo apt update
sudo apt install -y \
  build-essential \
  cmake \
  git \
  libjpeg-dev \
  libv4l-dev
```

## Clone and build

```bash
cd ~
git clone https://github.com/jacksonliam/mjpg-streamer.git
cd mjpg-streamer/mjpg-streamer-experimental
make
```

## Run camera stream

```bash
./mjpg_streamer \
  -i "./input_uvc.so -d /dev/video0 -r 640x480 -f 15 -y" \
  -o "./output_http.so -p 8080 -w ./www"
```

Camera URL:

```
http://<raspberry_ip>:8080/?action=stream
```

---

# 📡 Topic Overview

| Topic | Type | Publisher | Purpose |
|--------|------|-----------|----------|
| /cmd_vel | geometry_msgs/Twist | Web UI | Motor control |
| /left_ticks | std_msgs/Int32 | STM32 | Left encoder |
| /right_ticks | std_msgs/Int32 | STM32 | Right encoder |
| /distance | std_msgs/Float32 | Odometry | Accumulated distance |
| /odom | nav_msgs/Odometry | Odometry | Robot pose |
| /reset_odometry | std_msgs/Bool | Web UI | Reset position |

---

# 🖥 Web Interface Responsibilities

The browser:

- Sends `/cmd_vel`
- Displays:
  - Encoder ticks
  - Distance
  - X, Y position
  - Yaw angle
  - Linear and angular velocity
- Shows live camera feed
- Sends reset command

Communication path:

```
roslibjs → rosbridge → ROS2
```

---

# 🔄 Component Responsibilities

## STM32 (micro-ROS firmware)

Handles:

- PWM motor control
- Encoder interrupts
- `/cmd_vel` subscription
- Tick publishing

---

## micro-ROS Agent

Handles:

```
Serial XRCE-DDS ↔ ROS2 DDS
```

---

## Odometry Node

Handles:

```
Encoder ticks → Pose estimation
```

---

## rosbridge_server

Handles:

```
ROS2 DDS ↔ WebSocket JSON
```

---

## mjpg-streamer

Handles:

```
USB camera → HTTP MJPEG stream
```

---

# 🏁 System Startup Order

1️⃣ Start micro-ROS Agent  
2️⃣ Launch ROS bringup  
3️⃣ Start web server  
4️⃣ Start camera streaming  
5️⃣ Open browser  

---

# ✅ Final Result

You now have:

- Web-based teleoperation
- Real-time encoder feedback
- Differential-drive odometry
- Reset functionality
- Live USB camera streaming
- Fully ROS2-native modular architecture

---

# 🔮 Future Extensions

- RViz visualization
- SLAM integration
- Navigation2 stack
- Autonomous waypoint control
- Web-based map display
- rosbag2 logging
- Multi-robot support

---
