"""OpenFreeMap basemap cache and explicit, restartable regional downloads.

Only map data goes through this module. Offline packages never fetch missing data.
Provider policy: https://openfreemap.org/tos/ (2026-11-01).
"""
from contextlib import closing
import gzip
import json
import math
from pathlib import Path
import re
import sqlite3
import threading
from urllib.parse import urlsplit
from urllib.request import Request, urlopen
import uuid

MAX_TILES = 3000
MAX_BYTES = 50_000_000
ATTRIBUTION = ('<a href="https://openfreemap.org">OpenFreeMap</a> '
               '<a href="https://www.openmaptiles.org/">© OpenMapTiles</a> '
               'Data from <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a>')
SPRITES = ('sprite.json', 'sprite.png', 'sprite@2x.json', 'sprite@2x.png')


def validate_tile(data):
    """Check MVT layer envelopes; never save an HTML error page as a tile."""
    offset = 0
    while offset < len(data):
        if data[offset] != 26:  # Tile.layers: field 3, length-delimited.
            raise ValueError('地图源未返回有效的矢量瓦片')
        offset += 1
        length = 0
        for shift in range(0, 35, 7):
            if offset >= len(data):
                raise ValueError('地图瓦片被截断')
            byte = data[offset]
            offset += 1
            length |= (byte & 127) << shift
            if not byte & 128:
                break
        else:
            raise ValueError('地图瓦片长度无效')
        offset += length
        if offset > len(data):
            raise ValueError('地图瓦片被截断')


def tile_ranges(bounds, maxzoom):
    if not isinstance(bounds, list) or len(bounds) != 4 or not all(
            type(v) in (int, float) and math.isfinite(v) for v in bounds):
        raise ValueError('需要有限的西、南、东、北边界')
    west, south, east, north = bounds
    if not -180 <= west < east <= 180 or not -85.05112878 <= south < north <= 85.05112878:
        raise ValueError('请在墨卡托地图内选择区域，不跨越日期变更线')
    if type(maxzoom) is not int or not 0 <= maxzoom <= 14:
        raise ValueError('下载细节等级应为 0–14；地图可继续放大显示')
    result = []
    for z in range(maxzoom + 1):
        n = 2 ** z
        def column(lon):
            return min(n - 1, max(0, math.floor((lon + 180) / 360 * n)))
        def row(lat):
            return min(n - 1, max(0, math.floor((1 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2 * n)))
        result.append((z, column(west), column(east), row(north), row(south)))
    return result


def estimate(bounds, maxzoom):
    ranges = tile_ranges(bounds, maxzoom)
    count = sum((x1-x0+1)*(y1-y0+1) for _, x0, x1, y0, y1 in ranges)
    return {'tiles': count, 'allowed': count <= MAX_TILES, 'max_tiles': MAX_TILES, 'max_bytes': MAX_BYTES}


class MapStore:
    def __init__(self, root):
        self.root = Path(root).expanduser().resolve()
        self.root.mkdir(parents=True, exist_ok=True)
        self.lock = threading.RLock()
        self.prepare_lock = threading.Lock()
        # Combined online browsing + downloads never exceed the provider's four requests.
        self.network_slots = threading.BoundedSemaphore(3)
        self.cancel = threading.Event()
        self.worker = None
        self.active = None
        self.selected = 'online'
        settings = self.root / 'selection.json'
        if settings.exists():
            self.selected = json.loads(settings.read_text())['selected']
            self.path(self.selected)
        for file in self.root.glob('*.mbtiles'):
            if file.stem == 'online':
                continue
            info = self.metadata(file.stem)
            if info.get('status') == 'downloading':
                info.update(status='interrupted', error='上次下载中断，可继续下载')
                self.set_metadata(file.stem, info)

    def path(self, key):
        if key != 'online' and not re.fullmatch(r'[0-9a-f]{32}', key):
            raise ValueError('地图包标识无效')
        return self.root / (key + '.mbtiles')

    def connect(self, key):
        return sqlite3.connect(self.path(key), timeout=10)

    def create(self, key, info):
        with closing(self.connect(key)) as db, db:
            db.executescript('CREATE TABLE metadata(name TEXT PRIMARY KEY,value TEXT); '
                             'CREATE TABLE tiles(zoom_level INTEGER,tile_column INTEGER,tile_row INTEGER,tile_data BLOB,'
                             'PRIMARY KEY(zoom_level,tile_column,tile_row)); '
                             'CREATE TABLE assets(name TEXT PRIMARY KEY,data BLOB);')
            db.executemany('INSERT INTO metadata VALUES (?,?)', [('format', 'pbf'), ('name', info['name']),
                           ('minzoom', '0'), ('maxzoom', str(info.get('maxzoom', 14))),
                           ('attribution', ATTRIBUTION), ('agi_package', json.dumps(info, ensure_ascii=False))])
            if 'bounds' in info:
                db.execute('INSERT INTO metadata VALUES (?,?)', ('bounds', ','.join(map(str, info['bounds']))))

    def metadata(self, key):
        if not self.path(key).is_file():
            raise ValueError('地图包不存在')
        with closing(self.connect(key)) as db:
            return json.loads(db.execute("SELECT value FROM metadata WHERE name='agi_package'").fetchone()[0])

    def set_metadata(self, key, info):
        with closing(self.connect(key)) as db, db:
            db.execute("UPDATE metadata SET value=? WHERE name='agi_package'", (json.dumps(info, ensure_ascii=False),))

    def status(self):
        with self.lock:
            packages = [dict(self.metadata(p.stem), id=p.stem) for p in sorted(self.root.glob('*.mbtiles')) if p.stem != 'online']
            return {'selected': self.selected, 'directory': str(self.root), 'packages': packages,
                    'active': self.active, 'max_tiles': MAX_TILES, 'max_bytes': MAX_BYTES}

    def fetch(self, url):
        parsed = urlsplit(url)
        if parsed.scheme != 'https' or parsed.netloc != 'tiles.openfreemap.org':
            raise ValueError('地图源返回了不支持的资源地址')
        with self.network_slots:
            with self.lock:
                if self.selected != 'online':
                    raise ValueError('离线模式不请求网络')
            request = Request(url, headers={'User-Agent': 'AgiliciousGroundStation-MapUI/1.0'})
            with urlopen(request, timeout=20) as response:
                if urlsplit(response.url).netloc != 'tiles.openfreemap.org':
                    raise ValueError('地图资源重定向到未知主机')
                data = response.read(MAX_BYTES + 1)
                if len(data) > MAX_BYTES:
                    raise ValueError('单个地图资源超过 50 MB')
                if response.headers.get('Content-Encoding') == 'gzip' or data.startswith(b'\x1f\x8b'):
                    data = gzip.decompress(data)
                return data

    def put_asset(self, key, name, data):
        with closing(self.connect(key)) as db, db:
            db.execute('INSERT OR REPLACE INTO assets VALUES (?,?)', (name, data))

    def asset(self, key, name):
        with closing(self.connect(key)) as db:
            row = db.execute('SELECT data FROM assets WHERE name=?', (name,)).fetchone()
        return bytes(row[0]) if row else None

    def prepare_online(self):
        with self.prepare_lock:
            if not self.path('online').exists():
                self.create('online', {'name': 'OpenFreeMap', 'status': 'preparing'})
            info = self.metadata('online')
            if info['status'] == 'ready':
                return info
            style = json.loads(self.fetch('https://tiles.openfreemap.org/styles/liberty'))
            source = json.loads(self.fetch('https://tiles.openfreemap.org/planet'))
            if source.get('maxzoom') != 14 or source.get('minzoom') != 0:
                raise ValueError('地图源缩放范围发生变化，请更新地图配置')
            template = source['tiles'][0]
            if not re.fullmatch(r'https://tiles\.openfreemap\.org/planet/[A-Za-z0-9_]+/\{z\}/\{x\}/\{y\}\.pbf', template):
                raise ValueError('地图源瓦片路径发生变化')
            sprite = style['sprite']
            for name in SPRITES:
                data = self.fetch(sprite + name[len('sprite'):])
                if name.endswith('.json'):
                    json.loads(data)
                elif not data.startswith(b'\x89PNG\r\n\x1a\n'):
                    raise ValueError('地图图标下载格式错误')
                self.put_asset('online', name, data)
            # MapLibre >= 5.11 uses local fonts when glyphs is omitted. This makes
            # all labels (including Chinese) work offline without a glyph server.
            style.pop('glyphs', None)
            style['sources'] = {'openmaptiles': {'type': 'vector', 'tiles': [], 'minzoom': 0, 'maxzoom': 14,
                                               'attribution': ATTRIBUTION}}
            style['layers'] = [layer for layer in style['layers'] if layer.get('source', 'openmaptiles') == 'openmaptiles']
            for layer in style['layers']:
                if 'text-font' in layer.get('layout', {}):
                    layer['layout']['text-font'] = ['sans-serif']
            self.put_asset('online', 'style.json', json.dumps(style).encode())
            info.update(status='ready', template=template)
            self.set_metadata('online', info)
            return info

    def tile(self, key, z, x, y):
        if not 0 <= z <= 14 or not 0 <= x < 2**z or not 0 <= y < 2**z:
            raise ValueError('无效地图瓦片坐标')
        with closing(self.connect(key)) as db:
            row = db.execute('SELECT tile_data FROM tiles WHERE zoom_level=? AND tile_column=? AND tile_row=?',
                             (z, x, 2**z - 1 - y)).fetchone()
        return bytes(row[0]) if row else None

    def put_tile(self, key, z, x, y, data):
        with closing(self.connect(key)) as db, db:
            db.execute('INSERT OR REPLACE INTO tiles VALUES (?,?,?,?)', (z, x, 2**z - 1 - y, data))

    def resource(self, key, name):
        self.path(key)
        if key == 'online':
            with self.lock:
                if self.selected != 'online':
                    raise ValueError('已切换为离线地图')
            info = self.prepare_online()
        else:
            info = self.metadata(key)
            if info['status'] != 'ready':
                raise ValueError('地图包尚未下载完成')
        if name == 'style.json':
            style = json.loads(self.asset(key, name))
            style['sprite'] = f'/maps/{key}/sprite'
            source = style['sources']['openmaptiles']
            source['tiles'] = [f'/maps/{key}/tile/{{z}}/{{x}}/{{y}}.pbf']
            source['maxzoom'] = info.get('maxzoom', 14)
            if 'bounds' in info:
                source['bounds'] = info['bounds']
            return json.dumps(style).encode(), 'application/json'
        if name in SPRITES:
            return self.asset(key, name), 'image/png' if name.endswith('.png') else 'application/json'
        match = re.fullmatch(r'tile/(\d{1,2})/(\d{1,5})/(\d{1,5})\.pbf', name)
        if not match:
            raise ValueError('未知地图资源')
        z, x, y = map(int, match.groups())
        data = self.tile(key, z, x, y)
        if data is None and key == 'online':
            data = self.fetch(info['template'].format(z=z, x=x, y=y))
            validate_tile(data)
            self.put_tile(key, z, x, y, data)
        return data, 'application/x-protobuf'

    def select(self, key):
        with self.lock:
            self.path(key)
            if key != 'online' and self.metadata(key)['status'] != 'ready':
                raise ValueError('只能使用已完成的离线地图')
            temp = self.root / 'selection.tmp'
            temp.write_text(json.dumps({'selected': key}))
            temp.replace(self.root / 'selection.json')
            self.selected = key
            if key != 'online':
                self.cancel.set()
        return self.status()

    def start(self, payload):
        with self.lock:
            if self.worker and self.worker.is_alive():
                raise ValueError('已有下载任务，请等待或取消')
            if self.selected != 'online':
                raise ValueError('请先切换到在线地图再下载')
            key = payload.get('id')
            if key:
                info = self.metadata(key)
                if key == 'online' or info['status'] == 'ready':
                    raise ValueError('该地图无需继续下载')
            else:
                bounds, maxzoom = payload.get('bounds'), payload.get('maxzoom', 14)
                size = estimate(bounds, maxzoom)
                if not size['allowed']:
                    raise ValueError('区域超过 3000 瓦片，请缩小区域或降低细节等级')
                name = payload.get('name', '').strip()
                if not name or len(name) > 80:
                    raise ValueError('请填写 1–80 字的地图名称')
                key = uuid.uuid4().hex
                info = dict(name=name, bounds=bounds, maxzoom=maxzoom, total=size['tiles'], completed=0, bytes=0)
                self.create(key, info)
            info.update(status='downloading', error='', phase='准备地图样式')
            self.set_metadata(key, info)
            self.active = key
            self.cancel.clear()
            self.worker = threading.Thread(target=self.download, args=(key, info), daemon=True)
            self.worker.start()
        return self.status()

    def download(self, key, info):
        try:
            if 'template' not in info:
                online = self.prepare_online()
                for name in ('style.json', *SPRITES):
                    self.put_asset(key, name, self.asset('online', name))
                info['template'] = online['template']
                self.set_metadata(key, info)
            with closing(self.connect(key)) as db:
                info['completed'], tile_bytes = db.execute('SELECT count(*),coalesce(sum(length(tile_data)),0) FROM tiles').fetchone()
                info['bytes'] = tile_bytes + db.execute('SELECT coalesce(sum(length(data)),0) FROM assets').fetchone()[0]
            info['phase'] = '下载地图'
            for z, x0, x1, y0, y1 in tile_ranges(info['bounds'], info['maxzoom']):
                for x in range(x0, x1 + 1):
                    for y in range(y0, y1 + 1):
                        if self.cancel.is_set():
                            raise InterruptedError('下载已暂停，可继续')
                        if self.tile(key, z, x, y) is not None:
                            continue
                        # Reuse viewed tiles only from the same pinned dataset.
                        data = self.tile('online', z, x, y) if self.metadata('online').get('template') == info['template'] else None
                        if data is None:
                            data = self.fetch(info['template'].format(z=z, x=x, y=y))
                        validate_tile(data)
                        if info['bytes'] + len(data) > MAX_BYTES:
                            raise ValueError('超过地图源单次 50 MB 额度，请缩小区域或降低细节等级')
                        self.put_tile(key, z, x, y, data)
                        info['completed'] += 1
                        info['bytes'] += len(data)
                        self.set_metadata(key, info)
            if info['completed'] != info['total'] or info['bytes'] > MAX_BYTES:
                raise ValueError('地图包完整性检查失败')
            info.update(status='ready', phase='下载完成，可离线使用', error='')
        except InterruptedError as error:
            info.update(status='paused', error=str(error))
        except Exception as error:
            info.update(status='error', error=f'下载未完成：{error}')
        finally:
            self.set_metadata(key, info)
            with self.lock:
                self.active = None

    def close(self):
        self.cancel.set()
        if self.worker:
            self.worker.join(timeout=25)
