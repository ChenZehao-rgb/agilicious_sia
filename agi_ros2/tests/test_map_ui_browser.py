#!/usr/bin/env python3
"""Optional Playwright QA with synthetic local DDS/map data; never a real map.

Source ROS/install first; use an unused ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY=1.
Playwright is a test-only dependency, not required by the ground station.
"""
import argparse
import base64
import json
import math
import shutil
import sqlite3
import sys
import threading
import time
from unittest.mock import patch
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_map_ui import mapping
import map_ui
import ground_station_telemetry as relay
import runtime_launcher
import rclpy
from std_msgs.msg import Empty
from rclpy.executors import SingleThreadedExecutor
from agi_ros2.msg import FusedState, NavigationOrigin, GoalStatus, ComputationStatus, Health, GeographicGoal, Authority
from playwright.sync_api import sync_playwright, expect

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output', type=Path, required=True)
parser.add_argument('--browser', default=shutil.which('google-chrome'))
args = parser.parse_args()
out = args.output.resolve()
out.mkdir(parents=True, exist_ok=False)
path = out/'synthetic.mbtiles'
if path.exists(): path.unlink()
with sqlite3.connect(path) as db:
    db.executescript('CREATE TABLE metadata(name TEXT,value TEXT); CREATE TABLE tiles(zoom_level INTEGER,tile_column INTEGER,tile_row INTEGER,tile_data BLOB);')
    db.executemany('INSERT INTO metadata VALUES (?,?)', [('format','png'),('name','SYNTHETIC TEST FIXTURE - NOT A REAL MAP'),('minzoom','0'),('maxzoom','2'),('bounds','-180,-85,180,85'),('attribution','Generated single-colour test tiles; no geographic data')])
    png=base64.b64decode('iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mNk+A8AAQUBAScY42YAAAAASUVORK5CYII=')
    for z in range(3):
        for x in range(2**z):
            for y in range(2**z): db.execute('INSERT INTO tiles VALUES (?,?,?,?)',(z,x,y,png))
rclpy.init()
node = map_ui.MapUI()
# A synthetic browser fixture is its own deployment, even when other test
# domains or the user's real listener are running on this host.
with patch.object(relay, 'RuntimeLauncher', side_effect=lambda mode, notify: runtime_launcher.RuntimeLauncher(mode, notify, out)):
    transfer = relay.GroundStationTelemetry()
source = rclpy.create_node('browser_fixture')
executor=SingleThreadedExecutor()
for n in [node,transfer,source]: executor.add_node(n)
s,origin=mapping(); origin.altitude=100.
s.position.z=2.
s.velocity.x=math.nan
pub=source.create_publisher(FusedState,'fused_state',relay.LATEST)
opub=source.create_publisher(NavigationOrigin,'navigation/origin',relay.RETAINED)
gpub=source.create_publisher(GoalStatus,'goal_status',relay.RETAINED)
cpub=source.create_publisher(ComputationStatus,'computation_status',relay.LATEST)
hpub=source.create_publisher(Health,'health',relay.LATEST)
apub=source.create_publisher(Authority,'authority',relay.LATEST)
goal=GoalStatus(clock_id=s.clock_id,control_session_start=10.,active_valid=True,request_sequence=1,active_request_sequence=1,
                active_fc_session=s.navigation_session,active_fc_reset_counter=s.fc_reset_counter,active_generation=s.reset_counter,
                active_phase=GoalStatus.STABLE,request_state=GoalStatus.ACCEPTED)
goal.active_enu.x=3.;goal.active_enu.z=2.
pauses=[]
source.create_subscription(Empty,'pause_hover',pauses.append,1)
commands=[]
source.create_subscription(GeographicGoal,'go_to_geographic_position',lambda m: commands.append(m),1)
enabled=[False]; ticks=[0]; moving=[False]; auto_active=[True]; hover_stable=[True]
def emit():
    if not enabled[0]: return
    if moving[0]: s.position.x += .002
    s.fc_time_usec += 10000
    s.fc_state_sequence+=1;pub.publish(s)
    cpub.publish(ComputationStatus(control_session_start=10.,state_valid=True,controller_success=True,
                                   trajectory_active=auto_active[0],hover_stable=hover_stable[0]))
    hpub.publish(Health(battery_voltage=16.2))
    apub.publish(Authority(armed=True,auto_switch=True,receiver_valid=True,rc_link=True))
    if ticks[0]%100==0:
        # Real FC metadata refreshes its time even when its mapping is unchanged.
        origin.fc_time_usec=s.fc_time_usec
        origin.header.stamp.sec=ticks[0]//100
        opub.publish(origin); gpub.publish(goal)
    ticks[0]+=1
source.create_timer(.01,emit)
runtime_requests=[]; runtime_stops=[]; runtime_shutdowns=[]
def fake_runtime_start():
    runtime_requests.append(time.monotonic())
    transfer.runtime.update(state='starting', reason='synthetic startup', name='launch_realtime',
                            pid='12345', log='/synthetic/hardware.log')
    with transfer.runtime.lock:
        transfer.runtime.entries['launch_realtime']['state']='running'
    threading.Timer(.3, lambda: transfer.runtime.update(state='running', reason='synthetic process alive')).start()
# Browser regression must never invoke the real hardware entry.
transfer.runtime.start=fake_runtime_start
def fake_runtime_stop():
    runtime_stops.append(time.monotonic())
    transfer.runtime.update(state='stopping', reason='synthetic stopping')
    def finish():
        with transfer.runtime.lock:
            transfer.runtime.entries['launch_realtime'].update(state='exited', exit_code='0')
        transfer.runtime.update(state='stopped', reason='synthetic stopped')
    threading.Timer(.3, finish).start()
transfer.runtime.stop=fake_runtime_stop
def fake_runtime_shutdown():
    runtime_shutdowns.append(time.monotonic())
    with transfer.runtime.lock:
        transfer.runtime.shutdown_state = 'failed' if len(runtime_shutdowns) == 1 else 'accepted'
        transfer.runtime.shutdown_reason = ('关机失败：sudo: a password is required' if len(runtime_shutdowns) == 1
                                            else '关机命令已返回成功，等待主机关闭；尚未确认断电')
        transfer.runtime.shutdown_log = '/synthetic/shutdown.log'
    transfer.runtime.notify()
# Never let the browser fixture call a real shutdown implementation.
transfer.runtime.shutdown=fake_runtime_shutdown
# Dedicated synthetic runtime service; no flight controller is involved.
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
from rclpy.parameter import Parameter
runtime_values = dict(mode='sitl', controller_type='MPC', clock_id=s.clock_id, control_session_start=10.,
                      go_to_pose_max_vel=2., go_to_pose_max_acc=1., Q_pos_x=50., Q_pos_y=50., Q_pos_z=100.,
                      Q_att_x=10., Q_att_y=10., Q_att_z=50., Q_vel=5., R_collective_thrust=1.,
                      R_body_rates=[100.,100.,100.], exp_decay=1.)
tuning_requests=[]; tuning_reads=[]
def read_tuning(request, response):
    tuning_reads.append(time.monotonic())
    response.values = [Parameter(name, value=runtime_values.get(name)).get_parameter_value() for name in request.names]
    return response
def set_tuning(request, response):
    patch_values = {item.name: Parameter.from_parameter_msg(item).value for item in request.parameters}
    if 'R_body_rates' in patch_values: patch_values['R_body_rates'] = list(patch_values['R_body_rates'])
    tuning_requests.append(patch_values)
    runtime_values.update(patch_values)
    response.result.successful = True
    return response
source.create_service(GetParameters, 'flight_control/get_runtime_parameters', read_tuning)
source.create_service(SetParametersAtomically, 'flight_control/set_runtime_parameters', set_tuning)
worker=threading.Thread(target=executor.spin);worker.start()
server=map_ui.make_server(node,map_ui.MBTiles(path),'127.0.0.1',0)
http_thread=threading.Thread(target=server.serve_forever);http_thread.start()
url=f'http://127.0.0.1:{server.server_port}'
errors=[]; requests=[]
try:
    with sync_playwright() as p:
        browser=p.chromium.launch(executable_path=args.browser,headless=True,args=['--no-sandbox'])
        context=browser.new_context(viewport={'width':1440,'height':960})
        context.route('**/*', lambda route: route.continue_() if route.request.url.startswith(url) else route.abort())
        page=context.new_page();page.on('pageerror',lambda e: errors.append(str(e)));page.on('request',lambda r: requests.append(r.url))
        page.goto(url)
        page.locator('#open-tuning').click()
        expect(page.locator('#tuning-read-status')).to_contain_text('已读取')
        expect(page.locator('#tuning-identity')).to_have_text('仿真 · MPC')
        # All twelve exact tooltip strings come from the user-supplied prompt.
        help_text = page.evaluate('mpcHelp')
        assert list(help_text) == ['Q_pos_x','Q_pos_y','Q_pos_z','Q_att_x','Q_att_y','Q_att_z','Q_vel',
                                  'R_collective_thrust','R_body_rates_x','R_body_rates_y','R_body_rates_z','exp_decay']
        for name, description in help_text.items():
            label = page.locator(f'label[for="tuning-{name}"]')
            field = page.locator('#tuning-' + name)
            expect(label).to_have_attribute('title', description)
            expect(field).to_have_attribute('title', description)
            label.hover(); field.hover(); page.locator('#tuning-title').hover()
        assert not tuning_requests
        page.locator('#tuning-Q_pos_x').fill('60')
        page.locator('#tuning-go_to_pose_max_vel').fill('1.5')
        runtime_values['go_to_pose_max_acc'] = .8
        page.locator('#tuning-read').click()
        expect(page.locator('#actual-go_to_pose_max_acc')).to_contain_text('0.8')
        expect(page.locator('#tuning-Q_pos_x')).to_have_value('60')
        page.locator('#tuning-apply-trajectory').click()
        expect(page.locator('#tuning-result-trajectory')).to_contain_text('已确认应用')
        assert tuning_requests == [{'go_to_pose_max_vel':1.5}]
        expect(page.locator('#tuning-Q_pos_x')).to_have_value('60')
        page.locator('#tuning-R_body_rates_y').fill('110')
        page.locator('#tuning-apply-mpc').click()
        expect(page.locator('#tuning-result-mpc')).to_contain_text('已确认应用')
        assert tuning_requests[-1] == {'Q_pos_x':60., 'R_body_rates':[100.,110.,100.]}
        page.screenshot(path=str(out/'tuning.png'))
        page.locator('#tuning-Q_pos_y').fill('65')
        page.keyboard.press('Escape')
        expect(page.locator('#tuning-dialog')).not_to_be_visible()
        assert len(tuning_requests)==2
        page.locator('#open-tuning').click()
        expect(page.locator('#tuning-Q_pos_y')).to_have_value('65')
        page.locator('#tuning-revert-mpc').click()
        expect(page.locator('#tuning-Q_pos_y')).to_have_value('50')
        held_tuning=[]
        page.route('**/api/tuning', lambda route: held_tuning.append(route) if route.request.method == 'POST' else route.continue_())
        page.locator('#tuning-Q_vel').fill('6')
        page.locator('#tuning-apply-mpc').click()
        expect(page.locator('#tuning-apply-mpc')).to_be_disabled()
        expect(page.locator('#tuning-result-mpc')).to_contain_text('结果未确认', timeout=8000)
        assert len(held_tuning)==1 and len(tuning_requests)==2
        for route in held_tuning: route.abort()
        page.unroute('**/api/tuning')
        # Explicit rejection keeps the draft and the last actual value.
        page.route('**/api/tuning', lambda route: route.fulfill(status=200, content_type='application/json',
                   body='{"status":"rejected","message":"未应用：fixture rejection"}')
                   if route.request.method == 'POST' else route.continue_())
        page.locator('#tuning-apply-mpc').click()
        expect(page.locator('#tuning-result-mpc')).to_contain_text('未应用')
        expect(page.locator('#tuning-Q_vel')).to_have_value('6')
        expect(page.locator('#actual-Q_vel')).to_contain_text('实际：5')
        page.unroute('**/api/tuning')
        # A confirmed SET survives a failed GET; no invented rollback/failure.
        page.route('**/api/tuning', lambda route: route.fulfill(status=200, content_type='application/json',
                   body='{"status":"applied","message":"已确认应用，回读未完成"}')
                   if route.request.method == 'POST' else route.continue_())
        page.locator('#tuning-apply-mpc').click()
        expect(page.locator('#tuning-result-mpc')).to_contain_text('已确认应用，回读未完成')
        expect(page.locator('#tuning-apply-mpc')).to_be_disabled()
        page.unroute('**/api/tuning')
        page.locator('#tuning-read').click()
        expect(page.locator('#tuning-apply-mpc')).to_be_enabled()
        page.evaluate("checkTuningSession({topics:{computation_status:{age_s:0,data:{control_session_start:11}}}})")
        expect(page.locator('#tuning-read-status')).to_contain_text('旧实际值已过期')
        expect(page.locator('#tuning-apply-trajectory')).to_be_disabled()
        assert len(tuning_requests)==2
        page.locator('#tuning-read').click()
        expect(page.locator('#tuning-apply-mpc')).to_be_enabled()
        page.evaluate("checkTuningSession({topics:{goal_status:{age_s:0,data:{clock_id:'new-boot',control_session_start:10}}}})")
        expect(page.locator('#tuning-read-status')).to_contain_text('旧实际值已过期')
        runtime_values['controller_type']='GEO'
        page.locator('#tuning-read').click()
        expect(page.locator('#tuning-read-status')).to_contain_text('MPC 参数不适用')
        expect(page.locator('#tuning-apply-mpc')).to_be_disabled()
        expect(page.locator('#tuning-apply-trajectory')).to_be_enabled()
        runtime_values['controller_type']='MPC'
        count_reads = len(tuning_reads)
        page.wait_for_timeout(300)
        assert len(tuning_reads)==count_reads
        page.locator('#close-tuning').click()
        page.reload()
        assert len(tuning_requests)==2
        expect(page.locator('#runtime-service')).to_contain_text('启动服务在线')
        expect(page.locator('#position')).to_contain_text('位置未知')
        expect(page.locator('#runtime-start')).to_be_enabled()
        expect(page.locator('#runtime-shutdown')).to_be_enabled()
        page.locator('#runtime-mode').select_option('simulation')
        expect(page.locator('#runtime-shutdown')).to_be_hidden()
        expect(page.locator('#runtime-service')).to_contain_text('该模式服务未连接')
        expect(page.locator('#runtime-start')).to_be_disabled()
        page.reload()
        expect(page.locator('#runtime-mode')).to_have_value('simulation')
        expect(page.locator('#runtime-start')).to_be_disabled()
        page.locator('#runtime-mode').select_option('hardware')
        # HTTP timeout is not a remote rejection and must never auto-resend.
        held_start=[]
        page.route('**/api/runtime/start', lambda route: held_start.append(route))
        page.locator('#runtime-start').click()
        expect(page.locator('#runtime-request')).to_contain_text('启动结果未确认', timeout=7000)
        assert len(held_start)==1 and runtime_requests==[]
        for route in held_start: route.abort()
        page.unroute('**/api/runtime/start')
        page.locator('#runtime-start').click()
        expect(page.locator('#runtime-request')).to_have_text('启动请求已发送，等待执行端反馈')
        expect(page.locator('#runtime-state')).to_have_text('运行中')
        expect(page.locator('#runtime-start')).to_be_disabled()
        assert len(runtime_requests)==1
        page.reload()
        expect(page.locator('#runtime-state')).to_have_text('运行中')
        assert len(runtime_requests)==1
        page.route('**/api/state', lambda route: route.abort())
        expect(page.locator('#runtime-service')).to_contain_text('未确认')
        assert len(runtime_requests)==1 and transfer.runtime.snapshot()['state']=='running'
        page.unroute('**/api/state')
        expect(page.locator('#runtime-service')).to_contain_text('启动服务在线')
        expect(page.locator('#runtime-stop')).to_be_enabled()
        page.route('**/api/runtime/stop', lambda route: route.fulfill(status=404, content_type='application/json',
                                                                    body='{"error":"not found"}'))
        page.locator('#runtime-stop').click()
        expect(page.locator('#runtime-request')).to_contain_text('请重启 map_ui.py')
        assert runtime_stops==[]
        page.unroute('**/api/runtime/stop')
        held_stop=[]
        page.route('**/api/runtime/stop', lambda route: held_stop.append(route))
        page.locator('#runtime-stop').click()
        expect(page.locator('#runtime-request')).to_contain_text('结束任务结果未确认', timeout=7000)
        assert len(held_stop)==1 and runtime_stops==[]
        for route in held_stop: route.abort()
        page.unroute('**/api/runtime/stop')
        page.locator('#runtime-stop').click()
        expect(page.locator('#runtime-request')).to_have_text('结束任务请求已发送，等待执行端反馈')
        expect(page.locator('#runtime-state')).to_have_text('已结束')
        expect(page.locator('#runtime-stop')).to_be_disabled()
        expect(page.locator('#runtime-start')).to_be_enabled()
        page.reload()
        expect(page.locator('#runtime-state')).to_have_text('已结束')
        assert len(runtime_stops)==1 and len(runtime_requests)==1
        held_shutdown=[]
        page.route('**/api/runtime/shutdown', lambda route: held_shutdown.append(route))
        page.locator('#runtime-shutdown').click()
        expect(page.locator('#runtime-request')).to_contain_text('关机结果未确认', timeout=7000)
        assert len(held_shutdown)==1 and runtime_shutdowns==[]
        for route in held_shutdown: route.abort()
        page.unroute('**/api/runtime/shutdown')
        page.locator('#runtime-shutdown').click()
        expect(page.locator('#runtime-request')).to_have_text('关机请求已发送，等待执行端反馈')
        expect(page.locator('#runtime-shutdown-status')).to_contain_text('sudo: a password is required')
        expect(page.locator('#runtime-shutdown')).to_be_enabled()
        expect(page.locator('#runtime-state')).to_have_text('已结束')
        page.locator('#runtime-shutdown').click()
        expect(page.locator('#runtime-shutdown-status')).to_contain_text('关机命令已返回成功')
        expect(page.locator('#runtime-shutdown')).to_be_disabled()
        expect(page.locator('#runtime-start')).to_be_disabled()
        assert len(runtime_shutdowns)==2
        page.reload()
        expect(page.locator('#runtime-shutdown-status')).to_contain_text('尚未确认断电')
        expect(page.locator('#runtime-shutdown')).to_be_disabled()
        page.route('**/api/state', lambda route: route.abort())
        expect(page.locator('#runtime-shutdown-status')).to_contain_text('最后报告 / 未确认')
        assert len(runtime_shutdowns)==2
        page.unroute('**/api/state')
        enabled[0]=True
        expect(page.locator("#phase")).to_have_text("稳定悬停")
        assert page.locator('#altitude').input_value()=='102.000'
        expect(page.locator('#position')).to_contain_text('未知 m/s')
        expect(page.locator('#phase')).to_have_class('phase hovering')
        hover_color=page.locator('#phase').evaluate('(e) => getComputedStyle(e).color')
        assert page.locator('#phase').evaluate('(e) => parseFloat(getComputedStyle(e).fontSize)') >= 28
        # AUTO capture must report hover even with no online goal ever accepted.
        goal.active_valid=False;goal.active_phase=GoalStatus.NO_TARGET;goal.request_sequence=0;gpub.publish(goal)
        expect(page.locator('#phase')).to_have_text('稳定悬停')
        expect(page.locator('#active-detail')).to_contain_text('尚无在线目标')
        hover_stable[0]=False
        expect(page.locator('#phase')).to_have_text('AUTO 控制中')
        auto_active[0]=False
        expect(page.locator('#phase')).to_have_text('无在线目标')
        auto_active[0]=True;hover_stable[0]=True
        goal.active_valid=True;goal.request_sequence=1
        goal.active_phase=GoalStatus.MOVING;gpub.publish(goal)
        expect(page.locator('#phase')).to_have_text('跟踪目标 / 转向')
        expect(page.locator('#phase')).to_have_class('phase tracking')
        assert page.locator('#phase').evaluate('(e) => getComputedStyle(e).color') != hover_color
        goal.active_phase=GoalStatus.SETTLING;gpub.publish(goal)
        expect(page.locator('#phase')).to_have_class('phase settling')
        goal.active_phase=GoalStatus.STABLE
        goal.active_reason='first detail;second detail\nthird detail'
        gpub.publish(goal)
        expect(page.locator('#phase')).to_have_class('phase hovering')
        expect(page.locator('#reason li')).to_have_text(['目标：first detail','目标：second detail','目标：third detail'])
        assert page.locator('#map-info').count()==0
        assert '请求序号属于控制进程' not in page.locator('body').inner_text()
        assert '显示本机距接收更新的时间' not in page.locator('body').inner_text()
        # Stable text should not cause DOM writes even though state keeps arriving.
        page.evaluate('''() => {
          window.positionChanges=[];
          window.invalidPositionChanges=[];
          new MutationObserver(() => positionChanges.push(performance.now()))
            .observe(document.getElementById('position'), {childList:true, characterData:true, subtree:true});
          new MutationObserver(() => {
            const text=document.getElementById('connection').textContent;
            if(text.includes('位置无效')) invalidPositionChanges.push(text);
          }).observe(document.getElementById('connection'), {childList:true});
        }''')
        page.wait_for_timeout(2100)
        assert page.evaluate('positionChanges.length')==0
        before_version=page.evaluate('lastStateVersion')
        moving[0]=True;page.wait_for_timeout(2400);moving[0]=False
        changes=page.evaluate('positionChanges')
        assert 10 <= len(changes) <= 13, changes
        assert all(b-a>=150 for a,b in zip(changes,changes[1:])),changes
        assert page.evaluate('lastStateVersion')-before_version>20
        assert page.evaluate('invalidPositionChanges')==[]
        # Main-thread stalls used to flash HTTP disconnected despite successful
        # requests. Distinguish them from actual transport failures and timeouts.
        page.evaluate('''() => {
          window.connectionChanges=[];
          new MutationObserver(() => connectionChanges.push(document.getElementById('connection').textContent))
            .observe(document.getElementById('connection'), {childList:true});
          const end=performance.now()+2100; while(performance.now()<end) {}
        }''')
        page.wait_for_timeout(500)
        assert not any('HTTP 中断' in text for text in page.evaluate('connectionChanges'))
        expect(page.locator('#connection')).to_contain_text('位置更新中')
        page.route('**/api/state', lambda route: route.abort())
        expect(page.locator('#connection')).to_have_text('UI HTTP 中断')
        page.unroute('**/api/state')
        expect(page.locator('#connection')).to_contain_text('位置更新中')
        network=context.new_cdp_session(page)
        network.send('Network.enable')
        network.send('Network.emulateNetworkConditions', {'offline':False,'latency':2000,
                     'downloadThroughput':-1,'uploadThroughput':-1})
        expect(page.locator('#connection')).to_have_text('UI 请求超时')
        network.send('Network.emulateNetworkConditions', {'offline':False,'latency':0,
                     'downloadThroughput':-1,'uploadThroughput':-1})
        network.detach()
        expect(page.locator('#connection')).to_contain_text('位置更新中')
        page.locator('#altitude').fill('105.5')
        page.locator('#map').click(position={'x':450,'y':350})
        assert page.locator('#altitude').input_value()=='105.5'
        assert page.locator('#latitude').input_value()
        page.locator('#latitude').fill('0.00001');page.locator('#longitude').fill('0.00002')
        page.locator('#execute').click();expect(page.locator("#submission")).to_contain_text("已发送")
        time.sleep(.3);assert len(commands)==1
        page.reload();expect(page.locator("#phase")).to_have_text("稳定悬停")
        assert len(commands)==1
        # No selected target is needed; moving state and goal HTTP busy do not disable pause.
        page.locator('#clear').click()
        goal.active_phase=GoalStatus.MOVING;gpub.publish(goal)
        expect(page.locator('#phase')).to_have_text('跟踪目标 / 转向')
        page.evaluate("document.getElementById('execute').disabled=true")
        expect(page.locator('#pause')).to_be_enabled()
        page.locator('#pause').click()
        expect(page.locator('#submission')).to_have_text('暂停请求已发送，等待控制端处理')
        page.wait_for_timeout(200);assert len(pauses)==1 and len(commands)==1
        for field in ['latitude','longitude','altitude']: assert page.locator('#'+field).input_value()==''
        goal.pause_active=True;goal.request_source='pause_hover';goal.request_sequence=2
        goal.active_request_sequence=2;gpub.publish(goal)
        expect(page.locator('#phase')).to_have_text('减速中')
        expect(page.locator('#request')).to_contain_text('暂停并悬停')
        assert '0.00, 0.00, 0.00' not in page.locator('#request').inner_text()
        assert page.evaluate('activeMarker.getTooltip().getContent()')=='悬停点'
        goal.active_phase=GoalStatus.SETTLING;gpub.publish(goal)
        expect(page.locator('#phase')).to_have_text('悬停收敛中')
        goal.active_phase=GoalStatus.STABLE;gpub.publish(goal)
        expect(page.locator('#phase')).to_have_text('已暂停，稳定悬停')
        page.reload();expect(page.locator('#phase')).to_have_text('已暂停，稳定悬停')
        assert len(pauses)==1
        page.locator('#latitude').fill('1.23');page.locator('#longitude').fill('2.34');page.locator('#altitude').fill('110')
        # Actual AbortSignal timeout; no HTTP retry or inferred success.
        held_routes=[]
        page.route('**/api/pause', lambda route: held_routes.append(route))
        page.locator('#pause').click()
        expect(page.locator('#pause')).to_be_disabled()
        expect(page.locator('#submission')).to_contain_text('暂停结果未确认',timeout=7000)
        expect(page.locator('#pause')).to_be_enabled()
        assert len(held_routes)==1 and len(pauses)==1
        for route in held_routes: route.abort()
        page.unroute('**/api/pause')
        assert [page.locator('#'+field).input_value() for field in ['latitude','longitude','altitude']]==['1.23','2.34','110']
        page.wait_for_timeout(400)
        page.screenshot(path=str(out/'desktop.png'),full_page=True)
        enabled[0]=False;page.wait_for_timeout(3500)
        assert '过期' in page.locator('#connection').inner_text()
        assert '未确认' in page.locator('#phase').inner_text()
        expect(page.locator('#phase')).to_have_class('phase unconfirmed')
        assert '位置过期' in page.locator('#position').inner_text(), page.locator('#position').inner_text()
        # New tile fetch still works with sources stopped and all external URLs blocked.
        assert page.request.get(url+'/tiles/2/0/0').status==200
        page.locator('.leaflet-control-zoom-out').click()
        page.screenshot(path=str(out/'stale.png'),full_page=True)
        page.set_viewport_size({'width':700,'height':900});page.screenshot(path=str(out/'narrow.png'),full_page=True)
        page.close();assert len(commands)==1
        assert len(runtime_requests)==1 and len(runtime_stops)==1 and transfer.runtime.snapshot()['state']=='stopped'
        assert not errors, errors
        assert all(r.startswith(url) for r in requests),requests
        (out/'result.json').write_text(json.dumps({'passed':True,'tuning_tooltips':12,'tuning_patches':tuning_requests,'tuning_timeout_no_retry':True,'requests':len(requests),'external_requests':0,'commands':len(commands),'pauses':len(pauses),'pause_timeout_no_retry':True,'runtime_no_aircraft_online':True,'runtime_requests':len(runtime_requests),
            'runtime_timeout_refresh_disconnect_no_retry':True,'runtime_stop_requests':len(runtime_stops),'runtime_stop_timeout_no_retry':True,'runtime_old_backend_404_guidance':True,'hardware_shutdown_mock_requests':len(runtime_shutdowns),
            'shutdown_hardware_only_permission_error_timeout_and_refresh':True,'page_errors':errors,'fixture':'synthetic, not real map',
            'position_updates_in_2_4s':len(changes), 'position_min_interval_ms':min(b-a for a,b in zip(changes,changes[1:])),
            'initial_auto_hover':True,'browser_stall_without_false_disconnect':True,'http_failure_and_timeout_recovery':True,
            'origin_refresh_without_invalid_position':True},indent=2))
        browser.close()
finally:
    server.shutdown();server.server_close();http_thread.join()
    executor.shutdown();worker.join()
    for n in [node,transfer,source]:n.destroy_node()
    rclpy.shutdown()
