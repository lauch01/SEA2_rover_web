#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32, Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    return q


class RoverOdometry(Node):
    def __init__(self):
        super().__init__('rover_odometry')

        # ===== Parámetros (tus valores) =====
        self.declare_parameter('ticks_per_rev', 360.0)
        self.declare_parameter('wheel_diameter_m', 0.070)
        self.declare_parameter('wheel_base_m', 0.142)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('update_hz', 20.0)

        self.ticks_per_rev = float(self.get_parameter('ticks_per_rev').value)
        self.wheel_diam = float(self.get_parameter('wheel_diameter_m').value)
        self.wheel_base = float(self.get_parameter('wheel_base_m').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.update_hz = float(self.get_parameter('update_hz').value)

        self.wheel_radius = self.wheel_diam / 2.0
        self.m_per_tick = (2.0 * math.pi * self.wheel_radius) / self.ticks_per_rev

        self.get_logger().info(
            f"ticks_per_rev={self.ticks_per_rev}, wheel_diam={self.wheel_diam} m, "
            f"wheel_base={self.wheel_base} m, m_per_tick={self.m_per_tick:.6f}"
        )

        # ===== Estado de ticks =====
        self.left_ticks = None
        self.right_ticks = None
        self.prev_left = None
        self.prev_right = None

        # ===== Estado odom =====
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.total_distance = 0.0

        self.last_time = self.get_clock().now()

        # Subs a ticks
        self.sub_l = self.create_subscription(Int32, 'left_ticks', self.cb_left, 50)
        self.sub_r = self.create_subscription(Int32, 'right_ticks', self.cb_right, 50)

        # Pubs
        self.pub_dist = self.create_publisher(Float32, 'distance', 10)
        self.pub_odom = self.create_publisher(Odometry, 'odom', 10)

        # TF
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        # Timer
        period = 1.0 / self.update_hz
        self.timer = self.create_timer(period, self.update)

    def cb_left(self, msg: Int32):
        self.left_ticks = int(msg.data)

    def cb_right(self, msg: Int32):
        self.right_ticks = int(msg.data)

    def update(self):
        if self.left_ticks is None or self.right_ticks is None:
            return

        # Inicialización de referencia
        if self.prev_left is None:
            self.prev_left = self.left_ticks
            self.prev_right = self.right_ticks
            self.last_time = self.get_clock().now()
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0.0:
            return

        dL_ticks = self.left_ticks - self.prev_left
        dR_ticks = self.right_ticks - self.prev_right

        self.prev_left = self.left_ticks
        self.prev_right = self.right_ticks
        self.last_time = now

        # Distancias por rueda
        dL = dL_ticks * self.m_per_tick
        dR = dR_ticks * self.m_per_tick

        # Cinemática diferencial
        dS = (dR + dL) / 2.0
        dYaw = (dR - dL) / self.wheel_base

        # Integración
        self.x += dS * math.cos(self.yaw + dYaw / 2.0)
        self.y += dS * math.sin(self.yaw + dYaw / 2.0)
        self.yaw += dYaw

        # Normaliza yaw
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        # Distancia total (módulo)
        self.total_distance += abs(dS)

        # Velocidades
        v = dS / dt
        w = dYaw / dt

        # Publicar distance
        dist_msg = Float32()
        dist_msg.data = float(self.total_distance)
        self.pub_dist.publish(dist_msg)

        # Publicar odom
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = float(self.x)
        odom.pose.pose.position.y = float(self.y)
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = yaw_to_quat(self.yaw)

        odom.twist.twist.linear.x = float(v)
        odom.twist.twist.angular.z = float(w)

        self.pub_odom.publish(odom)

        # TF opcional
        if self.tf_broadcaster is not None:
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = float(self.x)
            t.transform.translation.y = float(self.y)
            t.transform.translation.z = 0.0
            t.transform.rotation = yaw_to_quat(self.yaw)
            self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = RoverOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
