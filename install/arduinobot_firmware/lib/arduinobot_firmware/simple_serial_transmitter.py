#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import serial
import math
from geometry_msgs.msg import Quaternion

# Robot fiziksel parametreleri
wheel_radius = 0.03  # Tekerlek yarıçapı (metre)
wheel_base = 0.15    # İki tekerlek arası mesafe (metre)
encoder_resolution = 1000  # Enkoderin bir turdaki adım sayısı (örnek değer)
delta_time = 0.5     # Encoder verilerinin alınma süresi (örnek olarak 500 ms)
# İlk pozisyon ve yönelme değerleri (global değişkenler)
x = 0.0
y = 0.0
theta = 0.0

class SimpleSerialTransmitter(Node):
    def __init__(self):
        super().__init__("simple_serial_transmitter")

        self.declare_parameter("port", "/dev/virtualcom0")
        self.declare_parameter("baud_rate", 115200)

        self.port_ = self.get_parameter("port").value
        self.baud_rate_ = self.get_parameter("baud_rate").value
        self.sub_ = self.create_subscription(String, "serial_transmitter", self.msgCallback, 10)
        self.arduino_ = serial.Serial(port=self.port_, baudrate=self.baud_rate_, timeout=0.1)
        self.publisher_ = self.create_publisher(Odometry, 'odom', 1000)
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 1000)
        self.timer = self.create_timer(0.5, self.read_encoder_data)

    def quaternion_from_euler(self, roll, pitch, yaw):
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return [qx, qy, qz, qw]

    def cmd_vel_callback(self, msg):
        command = f"{msg.linear.x},{msg.angular.z}\n"
        self.arduino_.write(command.encode())
        

    def read_encoder_data(self):
        if self.arduino_.in_waiting > 0:
            data = self.arduino_.readline().decode().strip()
            try:
                #left_ticks, right_ticks = map(int, data.split(','))
                #odom = Odometry()
                #Sonradan eklendi
                #odom.header.stamp = self.get_clock().now().to_msg()
                #odom.header.frame_id = "odom"
                #odom.child_frame_id = "base_link"
                #Sonradan eklendi
                # Hesaplamalar için gerekli veriler burada işlenebilir
                #odom.twist.twist.linear.x = left_ticks * 0.01
                #odom.twist.twist.angular.z = right_ticks * 0.01
                #self.publisher_.publish(odom)
                
                left_ticks, right_ticks = map(int, data.split(','))
                delta_left = left_ticks * (2 * 3.1415 * wheel_radius) / encoder_resolution
                delta_right = right_ticks * (2 * 3.1415 * wheel_radius) / encoder_resolution

                delta_center = (delta_left + delta_right) / 2.0
                delta_theta = (delta_right - delta_left) / wheel_base

                global x, y, theta
                theta += delta_theta
                x += delta_center * math.cos(theta)
                y += delta_center * math.sin(theta)

                # Odometry mesajı oluşturma
                odom = Odometry()
                odom.header.stamp = self.get_clock().now().to_msg()
                odom.header.frame_id = "odom"
                odom.child_frame_id = "base_link"

                # Pozisyon ve yönelme
                odom.pose.pose.position.x = x
                odom.pose.pose.position.y = y
                odom.pose.pose.position.z = 0.0
                quaternion = self.quaternion_from_euler(0, 0, theta)
                odom.pose.pose.orientation.x = quaternion[0]
                odom.pose.pose.orientation.y = quaternion[1]
                odom.pose.pose.orientation.z = quaternion[2]
                odom.pose.pose.orientation.w = quaternion[3]


                # Twist (hız bilgisi)
                odom.twist.twist.linear.x = delta_center / delta_time
                odom.twist.twist.angular.z = delta_theta / delta_time

                self.publisher_.publish(odom)
            except ValueError:
                self.get_logger().error("Invalid encoder data format")

    def msgCallback(self, msg):
        self.get_logger().info("New message received, publishing on serial: %s" % self.arduino_.name)
        self.arduino_.write(msg.data.encode("utf-8"))


def main():
    rclpy.init()
    simple_serial_transmitter = SimpleSerialTransmitter()
    rclpy.spin(simple_serial_transmitter)
    simple_serial_transmitter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

