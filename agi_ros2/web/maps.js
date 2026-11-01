'use strict';
let mapSelection = false, firstCorner = null, selectedBounds = null, areaLayer = null;
let mapCatalog = null, baseLayer = null, loadedMap = null, mapLoadVersion = 0;
let estimateVersion = 0, estimateAllowed = false, mapActionBusy = false;
let mapStatusText = '地图 · 加载中', mapStatusReady = false, mapServiceError = false;
const statusNames = {downloading: '下载中', ready: '可离线使用', paused: '已暂停', interrupted: '上次中断', error: '下载未完成'};

function setMapStatus(text, ready) {
  mapStatusText = text; mapStatusReady = ready; renderMapSummary();
}
function renderMapSummary() {
  const active = mapCatalog?.packages.find(p => p.id === mapCatalog.active);
  $('map-status').textContent = mapServiceError ? '地图 · 服务未连接' : active ?
    `地图 · 下载 ${Math.round(100 * active.completed / active.total)}%` : mapStatusText;
  $('map-summary').className = 'badge ' + (mapStatusReady && !mapServiceError ? 'good' : 'stale');
}
$('close-map-manager').onclick = () => { $('map-menu').open = false; $('map-summary').focus(); };
document.addEventListener('keydown', event => {
  if (event.key === 'Escape' && $('map-menu').open) { $('map-menu').open = false; $('map-summary').focus(); }
});

async function mapAPI(action, body) {
  const response = await fetch('/api/maps' + (action ? '/' + action : ''), body === undefined ? {} : {
    method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(body)
  });
  const result = await response.json();
  if (!response.ok) throw new Error(result.error || '地图服务不可用');
  return result;
}
function endSelection() {
  mapSelection = false; firstCorner = null;
  $('select-area').textContent = '框选区域（点击两角）';
  map.doubleClickZoom.enable(); $('map').classList.remove('selecting-area');
}
function setArea(bounds) {
  const west = bounds.getWest(), east = bounds.getEast(), south = bounds.getSouth(), north = bounds.getNorth();
  selectedBounds = [west, south, east, north];
  if (areaLayer) map.removeLayer(areaLayer);
  areaLayer = L.rectangle(bounds, {color: '#d98b24', weight: 2, fillOpacity: .08, interactive: false}).addTo(map);
  endSelection(); updateEstimate();
}
function selectMapCorner(point) {
  if (!firstCorner) { firstCorner = point; $('map-action').textContent = '再点击区域对角，完成框选'; }
  else { setArea(L.latLngBounds(firstCorner, point)); $('map-action').textContent = '下载区域已选定；未改变飞行目标'; }
}
async function updateEstimate() {
  const version = ++estimateVersion;
  estimateAllowed = false; updateMapButtons();
  if (!selectedBounds) return;
  try {
    const info = await mapAPI('estimate', {bounds: selectedBounds, maxzoom: Number($('map-detail').value)});
    if (version !== estimateVersion) return;
    estimateAllowed = info.allowed;
    $('map-estimate').textContent = `${info.tiles} 块地图 · 包含概览至所选细节\n` +
      (info.allowed ? '单包最多 3000 块 / 50 MB，下载时检查实际大小。' : '区域过大，请缩小区域或降低细节。');
  } catch (error) { if (version === estimateVersion) $('map-estimate').textContent = error.message; }
  updateMapButtons();
}
function updateMapButtons() {
  const online = mapCatalog?.selected === 'online';
  $('download-map').disabled = !online || !estimateAllowed || !!mapCatalog?.active || mapActionBusy;
  $('cancel-download').hidden = !mapCatalog?.active;
  $('download-panel').hidden = !online;
  $('view-package').hidden = online;
}
function renderCatalog(info) {
  mapCatalog = info;
  const options = [{id: 'online', name: '在线 · OpenFreeMap'}, ...info.packages.filter(p => p.status === 'ready')];
  const signature = JSON.stringify(options.map(p => [p.id, p.name]));
  if ($('map-source').dataset.signature !== signature) {
    $('map-source').replaceChildren(...options.map(p => new Option(p.id === 'online' ? p.name : `离线 · ${p.name}`, p.id)));
    $('map-source').dataset.signature = signature;
  }
  $('map-source').value = info.selected;
  $('map-storage').textContent = '保存在地面电脑：' + info.directory;
  $('map-jobs').replaceChildren();
  for (const pack of info.packages) {
    if (pack.status === 'ready') continue;
    const row = document.createElement('div');
    row.textContent = `${pack.name} · ${statusNames[pack.status] || pack.status}\n${pack.completed}/${pack.total} 块 · ${(pack.bytes / 1e6).toFixed(1)} MB\n${pack.error || pack.phase || ''}`;
    if (!info.active && info.selected === 'online') {
      const resume = document.createElement('button'); resume.textContent = '继续下载';
      resume.onclick = () => runMapAction('resume', {id: pack.id}); row.append(document.createElement('br'), resume);
    }
    $('map-jobs').append(row);
  }
  const active = info.packages.find(p => p.id === info.active);
  $('map-progress').hidden = !active;
  if (active) { $('map-progress').max = active.total; $('map-progress').value = active.completed; }
  renderMapSummary();
  updateMapButtons();
}
async function runMapAction(action, body) {
  mapActionBusy = true; updateMapButtons();
  try {
    const info = await mapAPI(action, body); renderCatalog(info);
    $('map-action').textContent = action === 'cancel' ? '正在暂停，已下载部分会保留' :
      action === 'select' ? (info.selected === 'online' ? '已切换在线地图' : '已切换离线地图，重启后保留选择') : '下载已开始，可在此查看进度';
    if (action === 'select') { endSelection(); await showBasemap(info.selected); }
  } catch (error) { $('map-action').textContent = error.message; }
  finally { mapActionBusy = false; updateMapButtons(); }
}
function fitPackage() {
  const pack = mapCatalog?.packages.find(p => p.id === mapCatalog.selected);
  if (pack) {
    const [w, s, e, n] = pack.bounds;
    $('follow').checked = false; centered = true;
    map.fitBounds([[s, w], [n, e]], {padding: [35, 35], maxZoom: 17, animate: false});
  }
}
async function showBasemap(key) {
  const version = ++mapLoadVersion;
  loadedMap = key;
  if (baseLayer) { map.removeLayer(baseLayer); baseLayer = null; }
  mapWarning = key === 'online' ? '正在加载在线地图…' : '正在读取离线地图…';
  $('map-warning').textContent = mapWarning;
  const pack = mapCatalog.packages.find(p => p.id === key);
  $('map-summary').title = key === 'online' ? '在线 · OpenFreeMap；点击管理地图' : `离线 · ${pack.name}；点击管理地图`;
  setMapStatus(key === 'online' ? '地图 · 在线加载中' : '地图 · 离线加载中', false);
  try {
    const response = await fetch(`/maps/${key}/style.json`);
    if (!response.ok) { const result = await response.json(); throw new Error(result.error || '底图加载失败'); }
    const style = await response.json();
    if (version !== mapLoadVersion) return;
    style.sprite = new URL(style.sprite, location.origin).href;
    // URL.href percent-encodes {z}/{x}/{y}; keep these template tokens intact.
    for (const source of Object.values(style.sources)) source.tiles = source.tiles.map(url => location.origin + url);
    baseLayer = L.maplibreGL({style, interactive: false, pane: 'tilePane'}).addTo(map);
    const gl = baseLayer.getMaplibreMap();
    let resourceError = false;
    gl.on('error', () => {
      if (version !== mapLoadVersion) return;
      resourceError = true;
      setMapStatus(key === 'online' ? '地图 · 在线加载失败' : '地图 · 离线资源缺失', false);
      mapWarning = key === 'online' ? '在线地图加载失败，请检查网络；下载未完成的区域不能保证离线可用' : '离线地图资源缺失；未联网补图';
      $('map-warning').textContent = mapWarning;
    });
    gl.on('load', () => {
      if (version !== mapLoadVersion || resourceError) return;
      setMapStatus(key === 'online' ? '地图 · 在线' : '地图 · 离线就绪', true);
      mapWarning = key === 'online' ? '' : '离线地图 · 仅显示已下载区域，区域外可能为空白';
      $('map-warning').textContent = mapWarning;
    });
    if (key !== 'online') fitPackage();
  } catch (error) {
    if (version === mapLoadVersion) {
      $('map-warning').textContent = '地图加载失败：' + error.message; loadedMap = null;
      setMapStatus('地图 · 加载失败', false);
    }
  }
}
async function pollMaps() {
  try {
    const info = await mapAPI('');
    mapServiceError = false;
    if (info.legacy) {
      $('map-manager').hidden = true;
      $('map-summary').onclick = event => event.preventDefault();
      await loadMap(); return;
    }
    const previous = mapCatalog;
    renderCatalog(info);
    if (previous?.active && !info.active) {
      const finished = info.packages.find(p => p.id === previous.active);
      if (finished?.status === 'ready') $('map-action').textContent = `${finished.name} 已下载完成，请在上方选择“离线 · ${finished.name}”使用`;
    }
    if (loadedMap !== info.selected && !mapActionBusy) {
      // Explicit user-requested initial map view; this is never a navigation origin.
      if (!previous && info.selected === 'online' && !centered) map.setView([41.80, 123.43], 12, {animate: false});
      await showBasemap(info.selected);
    }
  } catch (error) {
    mapServiceError = true; renderMapSummary();
    $('map-action').textContent = '地图服务连接失败：' + error.message;
  }
  setTimeout(pollMaps, 1000);
}
$('select-area').onclick = () => {
  if (mapSelection) { endSelection(); $('map-action').textContent = '已取消框选'; return; }
  mapSelection = true; firstCorner = null; $('follow').checked = false; centered = true;
  map.doubleClickZoom.disable(); $('map').classList.add('selecting-area');
  $('select-area').textContent = '取消框选'; $('map-action').textContent = '在地图上分别点击区域的两个对角';
};
$('use-view').onclick = () => { $('follow').checked = false; centered = true; setArea(map.getBounds()); };
$('view-shenyang').onclick = () => { $('follow').checked = false; centered = true; map.setView([41.80, 123.43], 12); };
$('view-package').onclick = fitPackage;
$('map-detail').onchange = updateEstimate;
$('map-source').onchange = event => runMapAction('select', {id: event.target.value});
$('download-map').onclick = () => runMapAction('download', {name: $('map-name').value, bounds: selectedBounds, maxzoom: Number($('map-detail').value)});
$('cancel-download').onclick = () => runMapAction('cancel', {});
pollMaps();
