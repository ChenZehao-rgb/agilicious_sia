#!/usr/bin/env python3
"""Explicit simulated receiver. Change parameters with ros2 param set /sim_rc ..."""
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from agi_ros2.msg import Authority


class SimRc(Node):
    def __init__(self):
        super().__init__('sim_rc')
        for name, value in dict(armed=False, auto_switch=False, kill=True, rc_link=True,
                                roll=1500, pitch=1500, throttle=1000, yaw=1500).items():
            self.declare_parameter(name, value)
        self.pub = self.create_publisher(Authority, 'authority', 1)
        # Parameter services use the default callback group. Keep the receiver
        # heartbeat runnable while a parameter request is being handled.
        self.heartbeat_group = MutuallyExclusiveCallbackGroup()
        self.create_timer(0.02, self.tick, callback_group=self.heartbeat_group)

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
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
