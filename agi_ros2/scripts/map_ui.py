#!/usr/bin/env python3
"""Local ground-computer HTTP UI. No device access, retries, or command queue."""
import argparse
from contextlib import closing
import json
import math
import mimetypes
import re
import sqlite3
import struct
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit, unquote
from map_store import MapStore, estimate

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rosidl_runtime_py.convert import message_to_ordereddict
from agi_ros2.msg import Authority, ComputationStatus, FusedState, GeographicGoal, GoalStatus, Health, NavigationOrigin, OutputStatus


def f32(value):
    return struct.unpack('f', struct.pack('f', value))[0]


# Each operation before double promotion in fc_geographic.h is a float operation.
METERS_PER_DEGREE = f32(f32(f32(f32(111.3195) * 1000) * 100) / 10000000) * 10000000 * .01


def finite(*values):
    return all(isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x) for x in values)


def clean_json(value):
    if isinstance(value, dict):
        return {key: clean_json(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean_json(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def mapping_error(state, origin):
    if not state:
        return '未收到地面站融合状态；检查控制链与 ground_station_telemetry 是否启动、DDS 是否互通'
    if state.get('state_source') != 'fc_fused':
        return '当前状态源不是 fc_fused，无法使用 FC 地理原点反算位置'
    if not state.get('fc_state_valid'):
        detail = state.get('readiness_reason') or '未提供原因'
        flags = state.get('fc_flags', 0)
        # Decode only an actual FC snapshot. An absent snapshot must not be
        # mistaken for positive evidence that the aircraft is disarmed.
        if flags & 0x80:
            conditions = []
            if not flags & 0x10:
                conditions.append('FC 未解锁')
            if not flags & 0x40:
                conditions.append('XY 融合未启用')
            if not flags & 1:
                conditions.append('XY 位置无效')
            if not flags & 2:
                conditions.append('Z 位置无效')
            if conditions:
                detail = '、'.join(conditions) + '；' + detail
        return 'FC 融合位置无效：' + detail
    if not origin or not origin['session_id'] or origin['session_id'] != state['navigation_session'] or \
            origin['fc_reset_counter'] != state['fc_reset_counter'] or not origin['fc_time_usec'] or \
            origin['fc_time_usec'] > state['fc_time_usec']:
        return 'FC origin session/reset/time mismatch'
    if origin['header']['frame_id'] != 'odom' or state['header']['frame_id'] != 'odom' or origin['altitude_reference'] != 'msl':
        return 'FC origin must use odom/MSL'
    if not origin['xy_valid'] or not origin['z_valid']:
        return 'FC GPS XY/MSL Z baseline unavailable'
    if not finite(origin['latitude'], origin['longitude'], origin['altitude'], origin['longitude_scale']) or \
            abs(origin['latitude']) > 90 or abs(origin['longitude']) > 180 or not 0 <= origin['longitude_scale'] <= 1:
        return 'invalid FC origin mapping'
    if origin['longitude_scale'] == 0:
        return 'longitude_scale is zero; longitude unknown'
    return None


def geographic_position(position, origin):
    """Inverse FC projection; caller checks mapping_error first. Zero is valid."""
    if not finite(position['x'], position['y'], position['z']):
        return None
    latitude = origin['latitude'] + position['y'] / METERS_PER_DEGREE
    longitude = (origin['longitude'] + position['x'] / (METERS_PER_DEGREE * origin['longitude_scale']) + 180) % 360 - 180
    altitude = origin['altitude'] + position['z']
    if not finite(latitude, longitude, altitude) or abs(latitude) > 90:
        return None
    return {'latitude': latitude, 'longitude': longitude, 'altitude_msl': altitude, 'relative_height': position['z']}


def heading_degrees(q):
    if not finite(*q.values()):
        return None
    norm = math.hypot(*q.values())
    if not math.isfinite(norm) or norm == 0:
        return None
    x, y, z, w = (q[key] / norm for key in ('x', 'y', 'z', 'w'))
    heading = (90 - math.degrees(math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z)))) % 360
    return 0.0 if heading == 360.0 else heading


class MBTiles:
    def __init__(self, path):
        self.path = Path(path).expanduser().resolve() if path else None
        self.info = {'available': False, 'error': '未指定地图包；使用 --mbtiles 指定本机栅格 MBTiles'}
        if self.path:
            try:
                with closing(self.connect()) as db:
                    metadata = dict(db.execute('SELECT name, value FROM metadata'))
                    db.execute('SELECT zoom_level, tile_column, tile_row, tile_data FROM tiles LIMIT 1')
                if metadata.get('format', '').lower() not in ('png', 'jpg', 'jpeg', 'webp'):
                    raise ValueError('只支持 PNG/JPEG/WebP 栅格包；不支持 PBF 向量包或未知 format')
                self.info = {'available': True, 'metadata': metadata, 'error': None}
            except (sqlite3.Error, ValueError, OSError) as error:
                self.info = {'available': False, 'error': str(error)}

    def connect(self):
        return sqlite3.connect(self.path.as_uri() + '?mode=ro', uri=True)

    def tile(self, z, x, y):
        if not self.info['available']:
            return None
        if not 0 <= z <= 30 or not 0 <= x < 2**z or not 0 <= y < 2**z:
            raise ValueError('invalid XYZ tile coordinate')
        with closing(self.connect()) as db:
            row = db.execute('SELECT tile_data FROM tiles WHERE zoom_level=? AND tile_column=? AND tile_row=?',
                             (z, x, 2**z - 1 - y)).fetchone()
        if not row:
            return None
        data = bytes(row[0])
        if data.startswith(b'\x89PNG\r\n\x1a\n'):
            mime = 'image/png'
        elif data.startswith(b'\xff\xd8\xff'):
            mime = 'image/jpeg'
        elif data.startswith(b'RIFF') and data[8:12] == b'WEBP':
            mime = 'image/webp'
        else:
            raise ValueError('tile is not a supported raster image')
        return data, mime


class MapUI(Node):
    def __init__(self, **kwargs):
        super().__init__('map_ui', **kwargs)
        self.lock = threading.Lock()
        self.latest = {}
        self.version = 0
        self.command_publisher = self.create_publisher(GeographicGoal, 'go_to_geographic_position',
                                                     QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                                                durability=DurabilityPolicy.VOLATILE))
        self.pause_publisher = self.create_publisher(Empty, 'pause_hover',
                                                     QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                                                durability=DurabilityPolicy.VOLATILE))
        topics = {'fused_state': FusedState, 'authority': Authority, 'health': Health,
                  'output_status': OutputStatus, 'computation_status': ComputationStatus,
                  'navigation/origin': NavigationOrigin, 'goal_status': GoalStatus}
        for topic, msg_type in topics.items():
            retained = topic in ('navigation/origin', 'goal_status')
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE if retained else ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL if retained else DurabilityPolicy.VOLATILE)
            self.create_subscription(msg_type, 'ground_station/' + topic,
                                     lambda msg, key=topic: self.receive(key, msg), qos)

    def receive(self, topic, message):
        data = message_to_ordereddict(message)
        with self.lock:
            self.version += 1
            self.latest[topic] = {'data': data, 'received': time.monotonic(), 'version': self.version}

    def snapshot(self):
        now = time.monotonic()
        with self.lock:
            topics = {key: {'data': item['data'], 'age_s': now - item['received'], 'version': item['version']}
                      for key, item in self.latest.items()}
        state = topics.get('fused_state', {}).get('data')
        origin = topics.get('navigation/origin', {}).get('data')
        error = mapping_error(state, origin)
        position = geographic_position(state['position'], origin) if not error else None
        goal = topics.get('goal_status', {}).get('data')
        computation = topics.get('computation_status', {}).get('data')
        active = None
        # Remote boot ID is an identity, never compared with this computer's boot ID or clock.
        goal_matches = bool(state and goal and computation and goal['clock_id'] == state['clock_id'] and
                            goal['control_session_start'] == computation['control_session_start'] and
                            goal['active_fc_session'] == state['navigation_session'] and
                            goal['active_fc_reset_counter'] == state['fc_reset_counter'] and
                            goal['active_generation'] == state['reset_counter'])
        if not error and goal_matches and goal['active_valid']:
            active = geographic_position(goal['active_enu'], origin)
        identity = ([state['clock_id'], state['navigation_session'], state['fc_reset_counter'], state['reset_counter'],
                     origin['session_id'], origin['fc_reset_counter'],
                     origin['latitude'], origin['longitude'], origin['altitude'], origin['longitude_scale']]
                    if state and origin else None)
        return clean_json({'topics': topics, 'position': position, 'mapping_error': error,
                           'heading_deg': heading_degrees(state['orientation']) if state and state['fc_state_valid'] else None,
                           'active_position': active, 'goal_matches_state': goal_matches, 'identity': identity})

    def pause(self, payload):
        if not isinstance(payload, dict) or payload:
            raise ValueError('暂停请求只接收空 JSON 对象 {}')
        with self.lock:
            self.pause_publisher.publish(Empty())
        return {'status': 'sent', 'message': '暂停请求已发送，等待控制端处理'}

    def submit(self, payload):
        if not isinstance(payload, dict) or set(payload) != {'latitude', 'longitude', 'altitude_msl'}:
            raise ValueError('只接收 latitude、longitude、altitude_msl')
        if not finite(*payload.values()) or abs(payload['latitude']) > 90 or abs(payload['longitude']) > 180:
            raise ValueError('目标必须是有限 WGS84 经纬度和 MSL 海拔')
        message = GeographicGoal(**{key: float(value) for key, value in payload.items()})
        # rclpy Publisher.publish is thread-safe. Serialize explicit HTTP submissions;
        # no Pilot calls, queues, timers, replay or application retries exist here.
        with self.lock:
            self.command_publisher.publish(message)
        return {'status': 'sent', 'message': '已发送 / 等待控制端处理；结果未确认', 'target': payload}


def web_directory():
    source = Path(__file__).resolve().parents[1] / 'web'
    if source.is_dir():
        return source
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('agi_ros2')) / 'web'


def make_server(node, tiles, host, port, maps=None):
    web = web_directory().resolve()

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def respond(self, status, data, mime='application/json; charset=utf-8'):
            if not isinstance(data, bytes):
                data = json.dumps(clean_json(data), ensure_ascii=False, allow_nan=False).encode()
            self.send_response(status)
            self.send_header('Content-Type', mime)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store' if mime.startswith('application/json') else 'no-cache')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('Content-Security-Policy', "default-src 'self'; img-src 'self' data: blob:; style-src 'self' 'unsafe-inline'; worker-src 'self' blob:; connect-src 'self'; frame-ancestors 'none'")
            self.end_headers()
            try:
                self.wfile.write(data)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_GET(self):
            path = unquote(urlsplit(self.path).path)
            if path == '/api/state':
                return self.respond(200, node.snapshot())
            if path == '/api/map':
                return self.respond(200, tiles.info)
            if path == '/api/maps':
                return self.respond(200, maps.status() if maps else {'legacy': True})
            if path.startswith('/maps/') and maps:
                try:
                    _, _, key, name = path.split('/', 3)
                    data, mime = maps.resource(key, name)
                    return self.respond(200, data, mime) if data is not None else self.respond(404, {'error': '离线地图未覆盖此处'})
                except (ValueError, OSError, sqlite3.Error) as error:
                    return self.respond(503, {'error': str(error)})
            tile_match = re.fullmatch(r'/tiles/(\d{1,2})/(\d{1,10})/(\d{1,10})', path)
            if tile_match:
                try:
                    tile = tiles.tile(*(int(x) for x in tile_match.groups()))
                    return self.respond(200, *tile) if tile else self.respond(404, {'error': '地图包或瓦片缺失'})
                except (ValueError, sqlite3.Error) as error:
                    return self.respond(400, {'error': str(error)})
            file = (web / ('index.html' if path == '/' else path.lstrip('/'))).resolve()
            if not file.is_relative_to(web) or not file.is_file():
                return self.respond(404, {'error': 'not found'})
            return self.respond(200, file.read_bytes(), mimetypes.guess_type(file.name)[0] or 'application/octet-stream')

        def do_POST(self):
            map_actions = ('estimate', 'download', 'resume', 'cancel', 'select')
            action = self.path.removeprefix('/api/maps/') if self.path.startswith('/api/maps/') else None
            if self.path not in ('/api/goal', '/api/pause') and (not maps or action not in map_actions):
                return self.respond(404, {'error': 'not found'})
            # JSON and same-origin only: an unrelated website cannot issue flight goals.
            origin = self.headers.get('Origin')
            if origin and origin != 'http://' + self.headers.get('Host', ''):
                return self.respond(403, {'error': 'cross-origin request denied'})
            if self.headers.get('Content-Type', '').split(';')[0].strip() != 'application/json':
                return self.respond(415, {'error': 'application/json required'})
            try:
                size = int(self.headers.get('Content-Length', '0'))
                if not 0 < size <= 4096:
                    raise ValueError('invalid request length')
                self.connection.settimeout(3)
                data = json.loads(self.rfile.read(size))
                if not isinstance(data, dict):
                    raise ValueError('JSON object required')
                if self.path == '/api/goal':
                    result = node.submit(data)
                elif self.path == '/api/pause':
                    result = node.pause(data)
                elif action == 'estimate':
                    result = estimate(data.get('bounds'), data.get('maxzoom', 14))
                elif action in ('download', 'resume'):
                    result = maps.start(data)
                elif action == 'select':
                    result = maps.select(data.get('id', ''))
                else:
                    maps.cancel.set()
                    result = maps.status()
            except TimeoutError as error:
                if self.path == '/api/pause':
                    return self.respond(503, {'error': '暂停结果未确认；不自动重试'})
                return self.respond(400, {'error': str(error)})
            except (ValueError, UnicodeError) as error:
                return self.respond(400, {'error': str(error)})
            except Exception:
                return self.respond(503, {'error': '发布结果未确认；不自动重试' if self.path in ('/api/goal', '/api/pause') else '地图操作失败'})
            return self.respond(202, result)

    return ThreadingHTTPServer((host, port), Handler)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8080)
    parser.add_argument('--mbtiles')
    parser.add_argument('--map-dir', default=str(Path.home() / '.local/share/agi_ros2/maps'), help='在线缓存及离线地图保存目录')
    args = parser.parse_args(rclpy.utilities.remove_ros_args()[1:])
    rclpy.init()
    node = MapUI()
    maps = None if args.mbtiles else MapStore(args.map_dir)
    server = make_server(node, MBTiles(args.mbtiles), args.host, args.port, maps)
    worker = threading.Thread(target=server.serve_forever, daemon=True)
    worker.start()
    print(f'Map UI: http://{args.host}:{server.server_port}', flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        server.server_close()
        worker.join()
        if maps:
            maps.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
