'use strict';
// One static copy: labels and native input tooltips share the same text.
const mpcHelp = {
  Q_pos_x: 'ENU x（东向）位置误差权重。增大后更重视该方向的位置跟踪，可能增加控制修正。',
  Q_pos_y: 'ENU y（北向）位置误差权重。增大后更重视该方向的位置跟踪，可能增加控制修正。',
  Q_pos_z: 'ENU z（向上）位置误差权重。增大后更重视高度跟踪，可能增加升降方向的控制修正。',
  Q_att_x: '滚转相关的倾斜误差权重。增大后更重视该姿态分量的参考跟踪。',
  Q_att_y: '俯仰相关的倾斜误差权重。增大后更重视该姿态分量的参考跟踪。',
  Q_att_z: '偏航相关的姿态误差权重。增大后更重视朝向跟踪。',
  Q_vel: '三轴速度误差的共用权重。增大后更重视参考速度跟踪，不直接设置飞行速度。',
  R_collective_thrust: '总推力加速度（T/m）偏离参考值的惩罚。增大后更抑制额外推力修正，响应可能更保守。',
  R_body_rates_x: '机体 x 轴（滚转）角速度偏离参考值的惩罚。增大后更抑制该轴的额外角速度修正。',
  R_body_rates_y: '机体 y 轴（俯仰）角速度偏离参考值的惩罚。增大后更抑制该轴的额外角速度修正。',
  R_body_rates_z: '机体 z 轴（偏航）角速度偏离参考值的惩罚。增大后更抑制该轴的额外角速度修正。',
  exp_decay: '预测时域的权重缩放系数。1 不额外缩放；0＜值＜1 更偏向近端，值＞1 加重远端。'
};
const tuningFields = {
  trajectory: {go_to_pose_max_vel: '最大/巡航速度（m/s）', go_to_pose_max_acc: '峰值加减速度（m/s²）'},
  mpc: Object.fromEntries(Object.keys(mpcHelp).map(name => [name, name]))
};
let tuningCurrent = null, tuningReady = false, tuningReading = false;
const tuningBusy = {trajectory: false, mpc: false}, tuningDirty = new Set();
const tuningInput = name => $('tuning-' + name);
function tuningActual(group, name) {
  if (name.startsWith('R_body_rates_')) return tuningCurrent?.mpc.R_body_rates?.['xyz'.indexOf(name.slice(-1))];
  return tuningCurrent?.[group]?.[name];
}
for (const [group, fields] of Object.entries(tuningFields)) {
  for (const [name, caption] of Object.entries(fields)) {
    const item = document.createElement('div'), label = document.createElement('label');
    label.textContent = caption; label.htmlFor = 'tuning-' + name;
    const input = document.createElement('input');
    input.id = label.htmlFor; input.type = 'number'; input.step = 'any'; input.min = '0'; input.placeholder = '未知';
    if (mpcHelp[name]) label.title = input.title = mpcHelp[name];
    const actual = document.createElement('div'); actual.className = 'hint'; actual.id = 'actual-' + name;
    input.oninput = () => { tuningDirty.add(name); renderTuning(); };
    item.append(label, input, actual); $('tuning-fields-' + group).append(item);
  }
  $('tuning-apply-' + group).onclick = () => applyTuning(group);
  $('tuning-revert-' + group).onclick = () => {
    for (const name of Object.keys(fields)) tuningDirty.delete(name);
    renderTuning();
  };
}
function renderTuning() {
  for (const [group, fields] of Object.entries(tuningFields)) {
    const available = tuningReady && (group !== 'mpc' || tuningCurrent.controller_type === 'MPC');
    for (const name of Object.keys(fields)) {
      const actual = tuningActual(group, name), input = tuningInput(name);
      if (!tuningDirty.has(name)) input.value = num(actual) ? actual : '';
      input.disabled = !available || tuningBusy[group];
      $('actual-' + name).textContent = (tuningReady ? '实际：' : '过期/未知：') +
        (actual == null ? '不可用' : Array.isArray(actual) ? JSON.stringify(actual) : actual) +
        (tuningDirty.has(name) ? ' · 草稿' : '');
    }
    $('tuning-apply-' + group).disabled = !available || tuningBusy[group] || tuningReading;
    $('tuning-revert-' + group).disabled = tuningBusy[group];
  }
  $('tuning-read').disabled = tuningReading || Object.values(tuningBusy).some(Boolean);
}
function acceptTuning(current) {
  tuningCurrent = current; tuningReady = true;
  $('tuning-identity').textContent = `${current.mode === 'hardware' ? '实机' : current.mode === 'sitl' ? '仿真' : '未知模式'} · ${current.controller_type}`;
  $('tuning-read-status').textContent = '已读取控制端当前值' + (current.controller_type === 'MPC' ? '' : '；MPC 参数不适用');
  renderTuning();
}
// Called from the existing state render; it never fetches parameters.
function checkTuningSession(state) {
  const computation = state?.topics?.computation_status;
  const goal = state?.topics?.goal_status;
  if (tuningReady && ((computation && computation.age_s < 3 &&
      computation.data.control_session_start !== tuningCurrent.control_session_start) ||
      (goal && goal.age_s < 3 && (goal.data.clock_id !== tuningCurrent.clock_id ||
       goal.data.control_session_start !== tuningCurrent.control_session_start)))) {
    tuningReady = false;
    $('tuning-read-status').textContent = '控制会话已改变，旧实际值已过期；请点击“读取当前值”';
    renderTuning();
  }
}
async function readTuning() {
  if (tuningReading || Object.values(tuningBusy).some(Boolean)) return;
  tuningReading = true; renderTuning(); $('tuning-read-status').textContent = '读取中…';
  try {
    const response = await fetch('/api/tuning', {cache: 'no-store', signal: AbortSignal.timeout(3500)});
    const result = await response.json();
    if (!response.ok) throw new Error(result.error || '参数不可读取');
    acceptTuning(result);
  } catch (error) {
    tuningReady = false; $('tuning-read-status').textContent = '参数不可读取：' + error.message;
  } finally { tuningReading = false; renderTuning(); }
}
async function applyTuning(group) {
  if ($('tuning-apply-' + group).disabled) return;
  const values = {}, submitted = [], resultBox = $('tuning-result-' + group);
  for (const name of Object.keys(tuningFields[group])) {
    if (!tuningDirty.has(name)) continue;
    const value = read('tuning-' + name);
    if (!num(value) || value < 0 || (value === 0 && (group === 'trajectory' || name === 'exp_decay'))) {
      resultBox.textContent = '未应用：请输入有效数值'; return;
    }
    submitted.push(name);
    if (value === tuningActual(group, name)) continue;
    if (name.startsWith('R_body_rates_')) {
      values.R_body_rates ??= [...tuningCurrent.mpc.R_body_rates];
      values.R_body_rates['xyz'.indexOf(name.slice(-1))] = value;
    } else values[name] = value;
  }
  if (!Object.keys(values).length) { resultBox.textContent = '没有改变的参数'; return; }
  tuningBusy[group] = true; renderTuning(); resultBox.textContent = '正在应用…';
  try {
    const response = await fetch('/api/tuning', {method: 'POST', headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({group, values}), signal: AbortSignal.timeout(6000)});
    const result = await response.json();
    resultBox.textContent = result.message || result.error || '结果未确认；请读取当前值核实';
    if (result.status === 'applied') {
      if (result.current) {
        for (const name of submitted) tuningDirty.delete(name);
        acceptTuning(result.current);
      } else {
        tuningReady = false;
        $('tuning-read-status').textContent = '已确认应用，回读未完成；请读取当前值核实';
      }
    }
  } catch (_) { resultBox.textContent = '结果未确认；可能已执行，不自动重试，请读取当前值核实'; }
  finally { tuningBusy[group] = false; renderTuning(); }
}
$('open-tuning').onclick = () => { $('tuning-dialog').showModal(); readTuning(); };
$('close-tuning').onclick = () => $('tuning-dialog').close();
$('tuning-read').onclick = readTuning;
renderTuning();
