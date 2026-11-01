#!/usr/bin/env python3
"""Deterministic map download lifecycle tests; no ROS, network or hardware."""
import json
from pathlib import Path
import sqlite3
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from map_store import MapStore, estimate, validate_tile


class StoreTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.store = MapStore(self.tmp.name)
        self.calls = []
        self.fail_at = None
        self.store.fetch = self.fetch

    def tearDown(self):
        self.store.close()
        self.tmp.cleanup()

    def fetch(self, url):
        self.calls.append(url)
        if self.fail_at and self.fail_at in url:
            raise OSError('simulated connection loss')
        if url.endswith('/styles/liberty'):
            return json.dumps({'version': 8, 'glyphs': 'remote', 'sprite': 'https://tiles.openfreemap.org/sprites/test',
                               'sources': {}, 'layers': [{'id': 'base', 'type': 'background'},
                               {'id': 'removed', 'type': 'raster', 'source': 'ne2_shaded'},
                               {'id': 'label', 'type': 'symbol', 'source': 'openmaptiles',
                                'layout': {'text-font': ['Remote font']}}]}).encode()
        if url.endswith('/planet'):
            return b'{"minzoom":0,"maxzoom":14,"tiles":["https://tiles.openfreemap.org/planet/test/{z}/{x}/{y}.pbf"]}'
        if url.endswith('.json'):
            return b'{}'
        if url.endswith('.png'):
            return b'\x89PNG\r\n\x1a\nTEST'
        return b'\x1a\x00'

    def start(self, **changes):
        payload = dict(name='沈阳测试', bounds=[123.42, 41.79, 123.43, 41.80], maxzoom=3)
        payload.update(changes)
        self.store.start(payload)
        self.store.worker.join(5)
        self.assertFalse(self.store.worker.is_alive())
        return self.store.status()['packages'][-1]

    def test_geometry_and_limits(self):
        self.assertEqual(estimate([-1, -1, 1, 1], 1)['tiles'], 5)
        self.assertFalse(estimate([-180, -80, 180, 80], 14)['allowed'])
        for bounds, zoom in [([1, 0, -1, 1], 2), ([0, 0, 1, 90], 2), ([0, 0, float('nan'), 1], 2),
                             ([False, 0, 1, 1], 2), ([0, 0, 1, 1], True), ([0, 0, 1, 1], 15)]:
            with self.assertRaises(ValueError):
                estimate(bounds, zoom)

    def test_offline_restart_has_all_assets_and_no_network_fallback(self):
        pack = self.start()
        self.assertEqual(pack['status'], 'ready')
        self.assertEqual(pack['completed'], pack['total'])
        self.store.select(pack['id'])
        restored = MapStore(self.tmp.name)
        def forbidden(_):
            self.fail('offline read attempted a network request')
        restored.fetch = forbidden
        try:
            self.assertEqual(restored.status()['selected'], pack['id'])
            style = json.loads(restored.resource(pack['id'], 'style.json')[0])
            self.assertNotIn('glyphs', style)
            self.assertNotIn('ne2_shaded', style['sources'])
            self.assertEqual(style['layers'][-1]['layout']['text-font'], ['sans-serif'])
            for name in ('sprite.json', 'sprite.png', 'sprite@2x.json', 'sprite@2x.png', 'tile/0/0/0.pbf'):
                self.assertIsNotNone(restored.resource(pack['id'], name)[0])
            self.assertIsNone(restored.resource(pack['id'], 'tile/3/0/0.pbf')[0])
            with self.assertRaises(ValueError):
                restored.resource('online', 'style.json')
            with self.assertRaises(ValueError):
                restored.resource('../escape', 'style.json')
        finally:
            restored.close()

    def test_partial_failure_resume_does_not_repeat_downloaded_tiles(self):
        self.fail_at = '/2/'
        pack = self.start()
        self.assertEqual(pack['status'], 'error')
        self.assertGreater(pack['completed'], 0)
        self.assertLess(pack['completed'], pack['total'])
        with self.assertRaises(ValueError):
            self.store.select(pack['id'])
        with self.assertRaises(ValueError):
            self.store.resource(pack['id'], 'style.json')
        self.fail_at = None
        self.calls.clear()
        pack = self.start(id=pack['id'])
        self.assertEqual(pack['status'], 'ready')
        self.assertFalse(any('/0/0/0.pbf' in call for call in self.calls))

    def test_pause_and_interrupted_restart(self):
        fetch = self.store.fetch
        def pause(url):
            result = fetch(url)
            if url.endswith('/0/0/0.pbf'):
                self.store.cancel.set()
            return result
        self.store.fetch = pause
        pack = self.start()
        self.assertEqual(pack['status'], 'paused')
        pack['status'] = 'downloading'
        self.store.set_metadata(pack['id'], pack)
        restored = MapStore(self.tmp.name)
        self.assertEqual(restored.metadata(pack['id'])['status'], 'interrupted')
        self.assertIsNone(restored.active)
        restored.close()

    def test_size_limit_and_invalid_tile_are_not_ready(self):
        with patch('map_store.MAX_BYTES', 1):
            pack = self.start()
        self.assertEqual(pack['status'], 'error')
        for bad in (b'<html>Error</html>', b'\x1a\x04abc', b'\x1a\x80'):
            with self.assertRaises(ValueError):
                validate_tile(bad)
        validate_tile(b'')  # An empty protobuf tile is valid.
        validate_tile(b'\x1a\x00')

    def test_cached_tile_tms_rows(self):
        self.store.prepare_online()
        self.store.resource('online', 'tile/2/1/0.pbf')
        count = len(self.calls)
        self.assertEqual(self.store.resource('online', 'tile/2/1/0.pbf')[0], b'\x1a\x00')
        self.assertEqual(count, len(self.calls))
        with sqlite3.connect(self.store.path('online')) as db:
            self.assertEqual(db.execute('SELECT tile_row FROM tiles').fetchone()[0], 3)


if __name__ == '__main__':
    unittest.main(verbosity=2)
