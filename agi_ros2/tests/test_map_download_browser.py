#!/usr/bin/env python3
"""Exercise download UI with a copied real-map fixture, then restart offline.

Source the ROS installation for map_ui imports. No ROS node is created. The
fixture must be a completed package made by MapStore; original files are read
only. Test-only dependency: Playwright and Chrome. No internet is used.
"""
import argparse
import json
from pathlib import Path
import shutil
import sys
import tempfile
import threading

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import map_ui
from map_store import MapStore
from playwright.sync_api import sync_playwright, expect


class NoFlightNode:
    def __init__(self):
        self.commands = []

    def snapshot(self):
        return dict(topics={}, position=None, mapping_error='等待真实遥测', heading_deg=None,
                    active_position=None, goal_matches_state=False, identity=None)

    def submit(self, payload):
        self.commands.append(payload)
        raise AssertionError('Map operation issued a flight command')


def serve(store, node):
    server = map_ui.make_server(node, map_ui.MBTiles(None), '127.0.0.1', 0, store)
    thread = threading.Thread(target=server.serve_forever)
    thread.start()
    return server, thread, f'http://127.0.0.1:{server.server_port}'


def stop(server, thread, store):
    server.shutdown()
    server.server_close()
    thread.join()
    store.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fixture', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--browser', default=shutil.which('google-chrome'))
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    errors, bad_responses, requests, network = [], [], [], []
    node = NoFlightNode()
    with tempfile.TemporaryDirectory() as tmp, sync_playwright() as p:
        # Seed online cache with real downloaded map data. Outside the fixture,
        # return valid empty tiles: this test never fetches another provider area.
        shutil.copy2(args.fixture, Path(tmp) / 'online.mbtiles')
        store = MapStore(tmp)
        meta = store.metadata('online')
        assert meta['status'] == 'ready'
        fixture_bounds = meta.pop('bounds')
        meta.update(name='OpenFreeMap fixture', maxzoom=14)
        store.set_metadata('online', meta)
        store.fetch = lambda _: b''
        server, thread, url = serve(store, node)
        browser = p.chromium.launch(executable_path=args.browser, headless=True,
                                    args=['--no-sandbox', '--enable-unsafe-swiftshader'])
        try:
            context = browser.new_context(viewport={'width': 1440, 'height': 1000})
            context.route('**/*', lambda route: route.continue_() if route.request.url.startswith(url) else route.abort())
            page = context.new_page()
            page.on('pageerror', lambda error: errors.append(str(error)))
            page.on('response', lambda response: bad_responses.append([response.status, response.url]) if response.status >= 400 else None)
            page.on('request', lambda request: requests.append(request.url))
            page.goto(url)
            expect(page.locator('#map-source')).to_have_value('online')
            expect(page.locator('#map-manager')).not_to_be_visible()
            page.locator('#map-summary').click()
            expect(page.locator('#map-manager')).to_be_visible()
            page.screenshot(path=str(args.output / 'map-menu-open.png'), full_page=True)
            page.set_viewport_size({'width': 390, 'height': 844})
            panel = page.locator('#map-manager').bounding_box()
            assert panel['x'] >= 0 and panel['x'] + panel['width'] <= 390, panel
            assert panel['y'] + panel['height'] <= 844, panel
            page.screenshot(path=str(args.output / 'map-menu-narrow.png'), full_page=True)
            page.set_viewport_size({'width': 1440, 'height': 1000})
            page.keyboard.press('Escape')
            expect(page.locator('#map-manager')).not_to_be_visible()
            page.locator('#map-summary').click()
            west, south, east, north = fixture_bounds
            page.evaluate('point => map.setView(point, 17, {animate:false})', [(south+north)/2, (west+east)/2])
            expect(page.locator('.maplibregl-canvas')).to_be_visible()
            page.locator('#select-area').click()
            page.locator('#map').click(position={'x': 420, 'y': 350})
            page.locator('#map').click(position={'x': 600, 'y': 500})
            assert page.locator('#latitude').input_value() == ''
            assert page.locator('#longitude').input_value() == ''
            expect(page.locator('#download-map')).to_be_enabled()
            page.locator('#map-name').fill('浏览器下载测试')
            page.locator('#download-map').click()
            expect(page.locator('#map-source option').filter(has_text='离线 · 浏览器下载测试')).to_have_count(1, timeout=15000)
            key = page.locator('#map-source option').filter(has_text='离线 · 浏览器下载测试').get_attribute('value')
            page.locator('#map-source').select_option(key)
            expect(page.locator('#map-status')).to_have_text('地图 · 离线就绪', timeout=15000)
            page.locator('#close-map-manager').click()
            expect(page.locator('#map-manager')).not_to_be_visible()
            assert not node.commands
            page.close()
            stop(server, thread, store)

            store = MapStore(tmp)
            assert store.selected == key
            def forbidden(url):
                network.append(url)
                raise AssertionError('Offline server attempted an external request')
            store.fetch = forbidden
            server, thread, url = serve(store, node)
            page = context.new_page()
            page.on('pageerror', lambda error: errors.append(str(error)))
            page.on('response', lambda response: bad_responses.append([response.status, response.url]) if response.status >= 400 else None)
            page.on('request', lambda request: requests.append(request.url))
            page.goto(url)
            expect(page.locator('#map-source')).to_have_value(key)
            expect(page.locator('#map-warning')).to_contain_text('离线地图 · 仅显示', timeout=15000)
            page.wait_for_timeout(1000)
            features = page.evaluate('baseLayer.getMaplibreMap().queryRenderedFeatures().length')
            assert features > 0, 'Real-map fixture did not render any features'
            expect(page.locator('.leaflet-control-attribution')).to_contain_text('OpenMapTiles')
            page.screenshot(path=str(args.output / 'offline-restart.png'), full_page=True)
            page.reload()
            expect(page.locator('#map-warning')).to_contain_text('离线地图 · 仅显示', timeout=15000)
            page.locator('.leaflet-control-zoom-in').click()
            page.wait_for_timeout(1000)
            page.screenshot(path=str(args.output / 'offline-zoom.png'), full_page=True)
            assert not network and not node.commands and not errors and not bad_responses, (network, errors, bad_responses)
            assert all(request.startswith('http://127.0.0.1:') or request.startswith('blob:') for request in requests)
            result = dict(passed=True, fixture=str(args.fixture), commands=len(node.commands), page_errors=errors,
                          bad_responses=bad_responses, network_attempts=network, rendered_features=features,
                          browser_requests=len(requests), package=store.metadata(key))
            (args.output / 'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2))
            print(json.dumps(result, ensure_ascii=False), flush=True)
        finally:
            browser.close()
            stop(server, thread, store)


if __name__ == '__main__':
    main()
