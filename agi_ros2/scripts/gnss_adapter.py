#!/usr/bin/env python3
"""Ordinary GNSS to a fixed local frame, without inventing RTK quality."""
import json
import math
import uuid

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import String
from agi_ros2.msg import Navigation, LocalNavigation, NavigationOrigin
from shadow_support import OriginBuilder, fresh, navigation_accuracy_ok


class GnssAdapter(Node):
    def __init__(self):
        super().__init__('gnss_adapter')
        if self.get_parameter('use_sim_time').value:
            raise ValueError('GNSS adapter requires system ROS time')
        defaults = dict(origin_duration=3., origin_samples=30, origin_max_speed=.3,
                        heading_confirmed=False, heading_correction_rad=0., fc_declination_applied=False,
                        horizontal_stddev=1., vertical_stddev=2., velocity_stddev=.3,
                        heading_stddev=math.radians(10),
                        max_horizontal_accuracy=0., max_vertical_accuracy=0., max_velocity_accuracy=0.)
        for k, v in defaults.items():
            self.declare_parameter(k, v, ParameterDescriptor(read_only=True))
        self.values = {k: self.get_parameter(k).value for k in defaults}
        for k in ('horizontal_stddev', 'vertical_stddev', 'velocity_stddev', 'heading_stddev'):
            if not math.isfinite(self.values[k]) or self.values[k] <= 0:
                raise ValueError(k)
        self.accuracy_limits = tuple(self.values[k] for k in (
            'max_horizontal_accuracy', 'max_vertical_accuracy', 'max_velocity_accuracy'))
        if any(not math.isfinite(x) or x < 0 for x in self.accuracy_limits):
            raise ValueError('Accuracy limits must be finite nonnegative values; zero means unconfigured')
        if not math.isfinite(self.values['heading_correction_rad']):
            raise ValueError('heading_correction_rad')
        self.builder = OriginBuilder(self.values['origin_duration'], self.values['origin_samples'],
                                     self.values['origin_max_speed'])
        self.session = str(uuid.uuid4())
        self.pub = self.create_publisher(LocalNavigation, 'sensors/local_navigation', qos_profile_sensor_data)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.origin_pub = self.create_publisher(NavigationOrigin, 'navigation/origin', qos)
        self.status_pub = self.create_publisher(String, 'navigation/status', 10)
        self.origin_message = None
        self.reason = 'Waiting for valid stationary GPS and confirmed heading'
        self.last_receive = float('nan')
        self.create_subscription(Navigation, 'sensors/navigation', self.on_navigation, qos_profile_sensor_data)
        self.create_timer(1., self.report)

    def report(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        reason = self.reason if fresh(now, self.last_receive, .5) else 'Navigation stream stale'
        self.status_pub.publish(String(data=json.dumps(dict(reason=reason, session=self.session,
                                                           origin_ready=self.builder.origin is not None))))
        if self.origin_message is not None:
            self.origin_pub.publish(self.origin_message)

    def reject(self, msg, reason):
        self.builder.samples.clear()
        self.reason = reason
        if self.origin_message is None:
            return
        out = LocalNavigation()
        # A rejection is an event at local detection time, never a new position fix.
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'odom'
        out.session_id = self.session
        out.source_session = msg.source_session
        out.altitude_reference = msg.altitude_reference
        out.fix_type = msg.fix_type
        out.clock_aligned = msg.clock_aligned
        out.heading = msg.heading
        out.heading_valid = msg.heading_valid
        out.observation_valid = False
        out.reason = reason
        self.pub.publish(out)

    def on_navigation(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        self.last_receive = now
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if not (msg.clock_aligned and 3 <= msg.fix_type <= 6 and (fresh(now, t, .3) or 0 < t-now <= .01)):
            self.reject(msg, 'Invalid fix, clock alignment or navigation age')
            return
        if not self.values['heading_confirmed'] or not msg.heading_valid or not math.isfinite(msg.heading):
            self.reject(msg, 'True-north heading source/correction unconfirmed or invalid')
            return
        velocity = msg.velocity.x, msg.velocity.y, msg.velocity.z
        if (not all(math.isfinite(x) for x in (msg.latitude, msg.longitude, msg.altitude, *velocity)) or
                abs(msg.latitude) > 90 or abs(msg.longitude) > 180 or
                msg.altitude_reference not in ('msl', 'ellipsoid') or
                (self.builder.origin is not None and msg.altitude_reference != self.builder.basis)):
            self.reject(msg, 'Navigation position, velocity or altitude reference invalid')
            return
        position = self.builder.add(t, msg.source_session, msg.latitude, msg.longitude, msg.altitude,
                                    msg.altitude_reference, velocity)
        if position is None:
            self.reason = 'Acquiring stationary origin, or invalid/duplicate navigation'
            return
        if self.origin_message is None:
            origin = NavigationOrigin()
            origin.header = msg.header
            origin.header.frame_id = 'odom'
            origin.session_id = self.session
            origin.latitude, origin.longitude, origin.altitude = self.builder.origin
            origin.altitude_reference = self.builder.basis
            origin.heading_source = 'fc_magnetometer_assisted_yaw'
            origin.heading_correction_rad = self.values['heading_correction_rad']
            origin.fc_declination_applied = self.values['fc_declination_applied']
            self.origin_message = origin
            self.origin_pub.publish(origin)
        out = LocalNavigation()
        out.observation_valid = True
        out.header = msg.header
        out.header.frame_id = 'odom'
        out.session_id = self.session
        out.source_session = msg.source_session
        out.altitude_reference = msg.altitude_reference
        out.fix_type = msg.fix_type
        out.position.x, out.position.y, out.position.z = position
        out.velocity = msg.velocity
        out.heading = math.remainder(msg.heading + self.values['heading_correction_rad'], 2*math.pi)
        out.heading_valid = True
        accuracies = msg.horizontal_accuracy, msg.vertical_accuracy, msg.velocity_accuracy
        out.accuracy_known = all(math.isfinite(x) and x > 0 for x in accuracies)
        out.horizontal_accuracy, out.vertical_accuracy, out.velocity_accuracy = accuracies
        out.accuracy_ok = navigation_accuracy_ok(accuracies, self.accuracy_limits)
        def variance(value, floor):
            return max(value, floor)**2 if math.isfinite(value) and value > 0 else floor**2
        out.position_variance = [variance(accuracies[0], self.values['horizontal_stddev'])]*2 + [
            variance(accuracies[1], self.values['vertical_stddev'])]
        out.velocity_variance = [variance(accuracies[2], self.values['velocity_stddev'])]*3
        out.heading_variance = self.values['heading_stddev']**2
        out.clock_aligned = msg.clock_aligned
        self.reason = ('Local navigation within configured accuracy limits' if out.accuracy_ok else
                       'Navigation for diagnostics only: accuracy unknown, above limit, or limits unconfigured')
        out.reason = self.reason
        self.pub.publish(out)


def main():
    rclpy.init()
    node = GnssAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
