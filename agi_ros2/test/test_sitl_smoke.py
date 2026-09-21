#!/usr/bin/env python3
"""Explicit Gazebo + Betaflight SITL hover smoke test (never starts on import).

Requires the ROS workspace and simulator to be built and sourced, and simulator
ports to be unused. Runs the source flight launch with the default simulation
profile, normal timing limits and no CSV. Logs go to AGI_SITL_TEST_LOG_DIR or
/tmp/agi_sitl_hardware_adaptation. It checks authorization and KILL behavior;
it does not establish hover accuracy or CM5 timing performance.

Example after sourcing ROS and install/agi_ros2/local_setup.bash:
  ROS_DOMAIN_ID=93 ROS_LOCALHOST_ONLY=1 GZ_PARTITION=agi_hover_smoke \
    python3 agi_ros2/test/test_sitl_smoke.py
"""
import json
import argparse
import os
import signal
import subprocess
import time
from pathlib import Path
import rclpy
from agi_ros2.msg import Authority, ComputationStatus, ControlCommand, FusedState, OutputStatus
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--controller', choices=('MPC', 'GEO'), default='MPC')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    logs = Path(os.environ.get('AGI_SITL_TEST_LOG_DIR', '/tmp/agi_sitl_hardware_adaptation'))
    logs.mkdir(exist_ok=True)
    rclpy.init()
    node = rclpy.create_node('hardware_adaptation_sim_check', parameter_overrides=[Parameter('use_sim_time', value=True)])
    received = {}
    subscriptions = []
    for topic, cls in [('fused_state', FusedState), ('control_command', ControlCommand), ('output_status', OutputStatus),
                       ('computation_status', ComputationStatus), ('reference', Odometry)]:
        received[topic] = []
        subscriptions.append(node.create_subscription(cls, topic, lambda msg, key=topic: received[key].append(msg), 100))
    pub = node.create_publisher(Authority, 'authority', 1)
    processes = []
    streams = []
    try:
        for name, command in [
            ('sim', ['python3', 'betaflight_sitl/run.py', '--no-gazebo', '--no-build', '--ros2', '--duration', '30',
                     '--controller', args.controller.lower()]),
            ('flight', ['ros2', 'launch', str(root/'agi_ros2/launch/flight.launch.py'), 'record_bag:=false',
                        'controller:=' + args.controller])]:
            stream = (logs/(name+'.log')).open('w')
            streams.append(stream)
            processes.append(subprocess.Popen(command, cwd=root, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True))
        start = time.monotonic()
        warm_at = None
        last_rc = 0
        stage = 'disarmed'
        while time.monotonic() - start < 70:
            now = time.monotonic()
            if any(p.poll() is not None for p in processes):
                break
            computations = received['computation_status']
            if warm_at is None and computations and computations[-1].warm_cycles >= 50:
                warm_at = now
            if warm_at is not None:
                # A brief manual throttle interval exercises the same capture-at-AUTO workflow.
                elapsed = now - warm_at
                stage = 'manual' if elapsed < .8 else 'auto' if elapsed < 4.8 else 'kill'
                if elapsed > 5.1:
                    break
            if now - last_rc >= .02:
                msg = Authority()
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.armed = stage in ('manual','auto')
                msg.auto_switch = stage == 'auto'
                msg.kill = stage == 'kill'
                msg.rc_link = True
                msg.manual_aetr = [1500, 1500, 1430 if stage == 'manual' else 1000, 1500]
                pub.publish(msg)
                last_rc = now
            rclpy.spin_once(node, timeout_sec=.001)
        commands = received['control_command']
        output = received['output_status']
        active = [c for c in commands if c.permit_override]
        summary = {'controller': args.controller, 'stage': stage, 'counts': {k:len(v) for k,v in received.items()},
                   'active_commands': len(active), 'output_was_active': any(s.override_active for s in output),
                   'controller_matches': bool(received['computation_status']) and all(
                       s.controller_type == args.controller for s in received['computation_status']),
                   'final_output_active': output[-1].override_active if output else None,
                   'last_compute_reason': received['computation_status'][-1].reason if received['computation_status'] else None,
                   'process_codes': [p.poll() for p in processes]}
        (logs/'result.json').write_text(json.dumps(summary, indent=2))
        print(json.dumps(summary, indent=2))
        if (stage != 'kill' or not active or not summary['output_was_active'] or summary['final_output_active'] or
                not summary['controller_matches']):
            raise SystemExit(1)
    finally:
        for p in reversed(processes):
            if p.poll() is None:
                os.killpg(p.pid, signal.SIGINT)
        for p in reversed(processes):
            try: p.wait(timeout=12)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGTERM)
                p.wait(timeout=10)
        for stream in streams: stream.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
