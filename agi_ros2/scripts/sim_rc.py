#!/usr/bin/env python3
"""Explicit simulated receiver. Change parameters with ros2 param set /sim_rc ..."""
import rclpy
from rclpy.node import Node
from agi_ros2.msg import Authority


class SimRc(Node):
    def __init__(self):
        super().__init__('sim_rc')
        for name, value in dict(armed=False, auto_switch=False, kill=True, rc_link=True,
                                roll=1500, pitch=1500, throttle=1000, yaw=1500).items():
            self.declare_parameter(name, value)
        self.pub = self.create_publisher(Authority, 'authority', 1)
        self.create_timer(0.02, self.tick)

    def tick(self):
        msg = Authority()
        msg.header.stamp = self.get_clock().now().to_msg()
        for name in ('armed', 'auto_switch', 'kill', 'rc_link'):
            setattr(msg, name, self.get_parameter(name).value)
        msg.manual_aetr = [max(1000, min(2000, self.get_parameter(k).value))
                           for k in ('roll', 'pitch', 'throttle', 'yaw')]
        self.pub.publish(msg)


if __name__ == '__main__':
    rclpy.init()
    node = SimRc()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
