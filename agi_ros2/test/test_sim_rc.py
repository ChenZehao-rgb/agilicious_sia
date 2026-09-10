#!/usr/bin/env python3
"""Receiver heartbeat must continue while a parameter service is occupied."""
import importlib.util
from pathlib import Path
import threading
import time
import unittest

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.msg import Parameter, ParameterValue, SetParametersResult
from rcl_interfaces.srv import SetParameters
from agi_ros2.msg import Authority

spec = importlib.util.spec_from_file_location(
    'sim_rc', Path(__file__).resolve().parents[1] / 'scripts/sim_rc.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class SimRcTest(unittest.TestCase):
    def test_heartbeat_during_parameter_service(self):
        rclpy.init()
        receiver = module.SimRc()
        observer = rclpy.create_node('receiver_test')
        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(receiver)
        executor.add_node(observer)
        stamps = []
        occupied = threading.Event()

        def slow_parameter_service(parameters):
            occupied.set()
            time.sleep(0.35)  # Longer than the flight RC deadline.
            return SetParametersResult(successful=True)

        receiver.add_on_set_parameters_callback(slow_parameter_service)
        subscription = observer.create_subscription(
            Authority, 'authority', lambda m: stamps.append((time.monotonic(), m)), 100)
        client = observer.create_client(SetParameters, '/sim_rc/set_parameters')
        worker = threading.Thread(target=executor.spin)
        worker.start()
        try:
            self.assertTrue(client.wait_for_service(timeout_sec=5))
            deadline = time.monotonic() + 5
            while len(stamps) < 10 and time.monotonic() < deadline:
                time.sleep(0.01)
            self.assertGreaterEqual(len(stamps), 10)
            request = SetParameters.Request(parameters=[
                Parameter(name='throttle', value=ParameterValue(type=2, integer_value=1450))])
            future = client.call_async(request)
            self.assertTrue(occupied.wait(2))
            begin = len(stamps)
            time.sleep(0.30)
            self.assertFalse(future.done())
            self.assertGreater(len(stamps) - begin, 8)
            during = [t for t, _ in stamps[begin:]]
            self.assertLess(max(b-a for a, b in zip(during, during[1:])), 0.1)
            deadline = time.monotonic() + 2
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.01)
            self.assertTrue(future.done())
            self.assertTrue(future.result().results[0].successful)
            deadline = time.monotonic() + 2
            while stamps[-1][1].manual_aetr[2] != 1450 and time.monotonic() < deadline:
                time.sleep(0.01)
            self.assertEqual(stamps[-1][1].manual_aetr[2], 1450)
        finally:
            executor.shutdown()
            worker.join()
            observer.destroy_node()
            receiver.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main(verbosity=2)
