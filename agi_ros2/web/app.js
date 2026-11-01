'use strict';
const $ = id => document.getElementById(id);
const num = value => typeof value === 'number' && Number.isFinite(value);
const fmt = (value, digits = 2) => num(value) ? value.toFixed(digits) : '未知';
const map = L.map('map').setView([0, 0], 2); // Neutral global view, never a navigation origin.
map.attributionControl.setPrefix('Leaflet 1.9.4');
let snapshot = null, identity = null, centered = false, lastHTTP = 0, lastStateVersion = -1;
let httpFailure = null;
let airplane = null, originMarker = null, draftMarker = null, activeMarker = null, track = [];
let altitudeInitialized = false, mapWarning = '';
let positionRendered = false, lastReasons = '';
new ResizeObserver(() => map.invalidateSize({pan: false})).observe($('map'));
const trail = L.polyline([], {color: '#72b6ee', weight: 3}).addTo(map);
const direction = L.polyline([], {color: '#86e1ad', weight: 2, dashArray: '6 8'}).addTo(map);
const latlng = position => [position.latitude, position.longitude];
const read = id => $(id).value.trim() === '' ? null : Number($(id).value);
function badge(id, text, fresh) { $(id).textContent = text; $(id).className = 'badge ' + (fresh ? 'good' : 'stale'); }
function data(topic) { return snapshot?.topics[topic]?.data; }
function age(topic) { return (snapshot?.topics[topic]?.age_s ?? Infinity) + (performance.now() - lastHTTP) / 1000; }
// Display stale annotations only; these are not flight acceptance gates.
function fresh(topic) { return age(topic) < (topic === 'fused_state' ? 1 : 3); }
function marker(previous, point, color, label) {
  if (!point) { if (previous) map.removeLayer(previous); return null; }
  if (previous) { previous.setLatLng(latlng(point)); return previous; }
  return L.circleMarker(latlng(point), {radius: 7, color, fillColor: color, fillOpacity: .75, weight: 2})
    .bindTooltip(label, {direction: 'top'}).addTo(map);
}
function draft() {
  const latitude = read('latitude'), longitude = read('longitude'), altitude_msl = read('altitude');
  return num(latitude) && Math.abs(latitude) <= 90 && num(longitude) && Math.abs(longitude) <= 180
    ? {latitude, longitude, altitude_msl} : null;
}
function updateDraft() {
  const target = draft(), position = snapshot?.position, origin = data('navigation/origin');
  draftMarker = marker(draftMarker, target, '#ffbc66', '待选目标');
  let relative = null, distance = null, delta = null;
  if (target && num(target.altitude_msl) && !snapshot?.mapping_error && origin) relative = target.altitude_msl - origin.altitude;
  if (target && position) {
    distance = map.distance(latlng(target), latlng(position));
    if (num(target.altitude_msl)) delta = target.altitude_msl - position.altitude_msl;
  }
  $('target-metrics').textContent = `相对原点高度 ${fmt(relative)} m\n水平距离 ${fmt(distance)} m · 高度差 ${fmt(delta)} m` +
    (position && !fresh('fused_state') ? '\n依据最后位置（已过期）' : '');
}
for (const id of ['latitude', 'longitude', 'altitude']) $(id).addEventListener('input', () => {
  if (id === 'altitude') altitudeInitialized = true;
  updateDraft();
});
map.on('click', event => {
  if (mapSelection) { selectMapCorner(event.latlng); return; }
  $('latitude').value = event.latlng.lat.toFixed(8);
  $('longitude').value = event.latlng.wrap().lng.toFixed(8);
  updateDraft();
});
map.on('dragstart', () => { $('follow').checked = false; });
$('clear').onclick = () => {
  for (const id of ['latitude', 'longitude', 'altitude']) $(id).value = '';
  altitudeInitialized = true;
  updateDraft();
};
$('current-altitude').onclick = () => {
  if (snapshot?.position && fresh('fused_state')) {
    $('altitude').value = snapshot.position.altitude_msl.toFixed(3);
    altitudeInitialized = true;
    updateDraft();
  } else $('submission').textContent = '当前有效海拔未知或已过期';
};
$('locate').onclick = () => { if (snapshot?.position) map.setView(latlng(snapshot.position), Math.max(map.getZoom(), 17)); };
$('fit').onclick = () => {
  const points = [snapshot?.position, snapshot?.active_position, draft()].filter(Boolean).map(latlng);
  if (points.length) map.fitBounds(L.latLngBounds(points), {padding: [65, 65], maxZoom: 18});
};
$('goal-form').addEventListener('submit', async event => {
  event.preventDefault();
  if ($('execute').disabled) return;
  const target = draft();
  if (!target || !num(target.altitude_msl)) return;
  $('execute').disabled = true;
  $('submission').textContent = '正在发送；结果未确认…';
  try {
    const response = await fetch('/api/goal', {method: 'POST', headers: {'Content-Type': 'application/json'},
      body: JSON.stringify(target), signal: AbortSignal.timeout(5000)});
    const result = await response.json();
    $('submission').textContent = response.ok ? `${result.message}\n${fmt(target.latitude, 8)}, ${fmt(target.longitude, 8)}, ${fmt(target.altitude_msl)} m MSL`
      : (result.error || '发送结果未确认');
  } catch (_) { $('submission').textContent = '连接中断：发送结果未确认。请查看控制端全局反馈；不会自动重试。'; }
  finally { $('execute').disabled = false; }
});
function renderTelemetry() {
  const state = data('fused_state'), authority = data('authority'), health = data('health'), output = data('output_status');
  const goal = data('goal_status'), computation = data('computation_status');
  const httpOK = lastHTTP > 0 && !httpFailure && performance.now() - lastHTTP < 1500;
  // A late browser timer is not evidence of a broken HTTP connection. Keep
  // transport failures separate from delayed rendering and stale ROS topics.
  badge('connection', httpFailure || (!lastHTTP ? 'UI 连接中' : !httpOK ? 'UI 在线 · 更新延迟' : !fresh('fused_state') ? 'UI 在线 · 位置未连接 / 过期' :
    snapshot?.position ? 'UI 在线 · 位置更新中' : 'UI 在线 · 状态已收到，位置无效'), httpOK && fresh('fused_state') && !!snapshot?.position);
  const prefix = topic => fresh(topic) ? '' : '过期 · ';
  badge('authority', authority ? `${prefix('authority')}ARM ${authority.armed ? 'ON' : 'OFF'} / AUTO ${authority.auto_switch ? 'ON' : 'OFF'} / KILL ${authority.kill ? 'ON' : 'OFF'} / RC ${authority.rc_link ? 'OK' : '丢失'}` : 'ARM / AUTO 未知', !!authority && fresh('authority'));
  badge('output', output ? `${prefix('output_status')}输出 ${output.override_active ? '接管中' : '未接管'}` : '输出 未知', !!output && fresh('output_status') && output.transport_healthy);
  badge('navigation', !snapshot?.mapping_error && snapshot?.position ? `${prefix('fused_state')}FC 导航 / 原点有效` : '导航未知 · ' + (snapshot?.mapping_error || '位置无效'), !!snapshot?.position && fresh('fused_state'));
  $('navigation').title = snapshot?.mapping_error || '';
  badge('battery', health ? `${prefix('health')}电压 ${fmt(health.battery_voltage)} V` : '电压 未知', !!health && num(health.battery_voltage) && fresh('health'));
  const connected = fresh('fused_state') && fresh('computation_status') && httpOK;
  const phase = ['无在线目标', '跟踪目标 / 转向', '终点收敛', '稳定悬停'];
  const sessionMatches = !!goal && !!computation && goal.control_session_start === computation.control_session_start && goal.clock_id === state?.clock_id;
  const phaseConfirmed = connected && sessionMatches && (!goal.active_valid || snapshot.goal_matches_state);
  const initialAuto = !goal?.active_valid && computation?.trajectory_active;
  const phaseText = initialAuto ? (computation.hover_stable ? '稳定悬停' : 'AUTO 控制中') : phase[goal?.active_phase] || '阶段未知';
  const phaseColor = initialAuto ? (computation.hover_stable ? 'hovering' : 'settling') : ['idle', 'tracking', 'settling', 'hovering'][goal?.active_phase];
  $('phase').textContent = `${phaseConfirmed ? '' : '未确认 · 最后报告：'}${phaseText}`;
  $('phase').className = 'phase ' + (phaseConfirmed ? phaseColor || 'unconfirmed' : 'unconfirmed');
  $('active-detail').textContent = goal?.active_valid ? `活动请求 #${goal.active_request_sequence}\nENU ${fmt(goal.active_enu.x)}, ${fmt(goal.active_enu.y)}, ${fmt(goal.active_enu.z)} m` +
    (!snapshot.goal_matches_state ? '\n会话未匹配；不显示旧目标坐标' : '') : initialAuto ? 'AUTO 参考已激活 · 尚无在线目标' : goal ? '当前快照无活动目标' : '活动目标未知';
  if (goal?.request_sequence) {
    const requested = goal.request_source === 'go_to_geographic_position' ? goal.requested_geographic : goal.requested_enu;
    const values = goal.request_source === 'go_to_geographic_position' ? `${fmt(requested.latitude, 8)}, ${fmt(requested.longitude, 8)}, ${fmt(requested.altitude_msl)} m MSL`
      : `${fmt(requested.x)}, ${fmt(requested.y)}, ${fmt(requested.z)} m (${goal.request_frame})`;
    $('request').textContent = `#${goal.request_sequence} · ${['无请求', '待处理', '已接受', '已拒绝'][goal.request_state] || '未知'}\n${values}\n${goal.request_reason}`;
  } else $('request').textContent = '尚无目标请求';
  const reasons = [['目标', goal?.active_reason], ['控制', computation?.reason], ['输出', output?.reason],
    ['最近故障', output?.last_fault], ['健康', health?.reason]]
    .flatMap(([label, value]) => (value || '').split(/[\r\n;；]+/).map(text => [label, text.trim()]).filter(([, text]) => text));
  const signature = JSON.stringify(reasons);
  if (signature !== lastReasons) {
    lastReasons = signature;
    $('reason').replaceChildren(...reasons.map(([label, text]) => {
      const item = document.createElement('li'); item.textContent = `${label}：${text}`; return item;
    }));
  }
  const labels = {'fused_state': '位置 / 姿态', authority: '授权', health: '电池 / 健康', output_status: '输出', computation_status: '控制', 'navigation/origin': '原点快照', goal_status: '目标快照'};
  $('freshness').textContent = Object.entries(labels).map(([key, label]) => `${label}：${fmt(Number.isFinite(age(key)) ? age(key) : null, 1)} s` +
    (key === 'navigation/origin' || key === 'goal_status' ? ' · 保留型' : fresh(key) ? '' : ' · 未更新')).join('\n');
  if (airplane) airplane.getElement()?.classList.toggle('old', !fresh('fused_state') || !httpOK || !snapshot?.position);
}
function renderState() {
  const key = JSON.stringify(snapshot.identity);
  if (key !== identity) {
    identity = key; track = []; trail.setLatLngs([]); lastStateVersion = -1;
    if (airplane) { map.removeLayer(airplane); airplane = null; }
  }
  const point = snapshot.position, origin = data('navigation/origin');
  originMarker = marker(originMarker, !snapshot.mapping_error ? origin : null, '#5ecce8', 'FC 导航原点');
  activeMarker = marker(activeMarker, snapshot.active_position, '#86e1ad', '已接受目标');
  direction.setLatLngs(point && snapshot.active_position ? [latlng(point), latlng(snapshot.active_position)] : []);
  if (point) {
    if (!airplane) airplane = L.marker(latlng(point), {icon: L.divIcon({className: 'aircraft', html: '<span>▲</span>', iconSize: [36, 36], iconAnchor: [18, 18]})}).addTo(map);
    airplane.setLatLng(latlng(point));
    const arrow = airplane.getElement()?.querySelector('span');
    if (arrow) { arrow.style.transform = `rotate(${snapshot.heading_deg ?? 0}deg)`; arrow.textContent = num(snapshot.heading_deg) ? '▲' : '●'; }
    const version = snapshot.topics.fused_state.version;
    if (version !== lastStateVersion) {
      lastStateVersion = version;
      track.push(latlng(point)); if (track.length > 2400) track.shift(); trail.setLatLngs(track);
      if (!centered) { map.setView(latlng(point), 17, {animate: false}); centered = true; }
      else if ($('follow').checked && fresh('fused_state')) map.panTo(latlng(point), {animate: false});
    }
    if (!altitudeInitialized && fresh('fused_state')) { $('altitude').value = point.altitude_msl.toFixed(3); altitudeInitialized = true; }
  }
  if (!positionRendered) { renderPosition(); positionRendered = true; }
  updateDraft(); renderTelemetry();
}
function renderPosition() {
  if (!snapshot) return;
  const point = snapshot.position;
  let text = '位置未知 · ' + (snapshot.mapping_error || '非有限位置');
  if (point) {
    const velocity = data('fused_state')?.velocity;
    const speed = velocity && [velocity.x, velocity.y, velocity.z].every(num) ? Math.hypot(velocity.x, velocity.y, velocity.z) : null;
    text = `${fmt(point.latitude, 7)}, ${fmt(point.longitude, 7)} · MSL ${fmt(point.altitude_msl)} m · 相对原点 ${fmt(point.relative_height)} m · ${fmt(speed)} m/s · 航向 ${fmt(snapshot.heading_deg, 1)}°` +
      (fresh('fused_state') && !httpFailure && performance.now() - lastHTTP < 1500 ? '' : ' · 位置过期');
  }
  // Only the bottom-left text is throttled; markers and the trail stay live.
  if ($('position').textContent !== text) $('position').textContent = text;
}
async function poll() {
  const start = performance.now();
  let updated = false;
  try {
    const response = await fetch('/api/state', {cache: 'no-store', signal: AbortSignal.timeout(1500)});
    if (!response.ok) throw new Error('HTTP');
    snapshot = await response.json(); lastHTTP = performance.now(); httpFailure = null; updated = true;
  } catch (error) {
    httpFailure = error.name === 'TimeoutError' ? 'UI 请求超时' : error.name === 'TypeError' ? 'UI HTTP 中断' : 'UI 数据读取失败';
  }
  // Rendering exceptions must not be reported as network failures or stop polling.
  try { if (updated) renderState(); else renderTelemetry(); }
  finally {
    // One in-flight request. Nominal 20 Hz local rendering, independent of DDS relay rates.
    setTimeout(poll, Math.max(0, 50 - (performance.now() - start)));
  }
}
async function loadMap() {
  try {
    const info = await (await fetch('/api/map')).json();
    if (!info.available) { setMapStatus('地图 · 未加载', false); mapWarning = info.error; return; }
    const meta = info.metadata;
    setMapStatus('地图 · 本地栅格', true);
    $('map-summary').title = meta.name || '本地栅格地图';
    const options = {};
    for (const key of ['minzoom', 'maxzoom']) if (meta[key] !== undefined && num(Number(meta[key])) && Number(meta[key]) >= 0 && Number(meta[key]) <= 30) options[key === 'minzoom' ? 'minNativeZoom' : 'maxNativeZoom'] = Number(meta[key]);
    // Attribution is untrusted package metadata; render as text, never execute HTML.
    const attribution = document.createElement('span'); attribution.textContent = meta.attribution || '地图来源未提供';
    options.attribution = attribution.innerHTML;
    L.tileLayer('/tiles/{z}/{x}/{y}', {...options, maxZoom: 30}).on('tileerror', () => {
      setMapStatus('地图 · 瓦片缺失', false);
      mapWarning = '当前视野有缺失瓦片；底图覆盖不限制目标输入'; $('map-warning').textContent = mapWarning;
    }).addTo(map);
    if (meta.bounds) {
      const b = meta.bounds.split(',').map(Number);
      if (b.length === 4 && b.every(num) && b[0] >= -180 && b[2] <= 180 && b[1] >= -90 && b[3] <= 90 && b[0] < b[2] && b[1] < b[3] && !centered) map.fitBounds([[b[1], b[0]], [b[3], b[2]]]);
    }
  } catch (_) { mapWarning = '本机地图元数据读取失败'; setMapStatus('地图 · 加载失败', false); }
  finally { $('map-warning').textContent = mapWarning; }
}
setInterval(renderTelemetry, 250);
setInterval(renderPosition, 200);
poll();
