"""Real ROS 2 node + PTY fake FC: rates, topics, timeout isolation and rosbag."""
import collections
import os
import pty
import select
import signal
import subprocess
import tempfile
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_prefix
from agi_ros2.msg import MspEvent
from rclpy.qos import QoSProfile


def main():
    rclpy.init()
    observer = rclpy.create_node('msp_test_observer')
    events = []
    attitude = []
    observer.create_subscription(MspEvent, '/msp/events', events.append, QoSProfile(depth=1000))
    observer.create_subscription(MspEvent, '/msp/attitude', attitude.append, QoSProfile(depth=100))
    master, slave = pty.openpty()
    processes = []
    with tempfile.TemporaryDirectory(prefix='ros2_msp_test_') as work:
        config = Path(work) / 'params.yaml'
        config.write_text('''/**:
  ros__parameters:
    mode: bench
    props_removed: true
    device: "''' + os.ttyname(slave) + '''"
    baud: 115200
    msp.attitude.rate_hz: 20.0
    msp.rc.enabled: false
    msp.analog.enabled: false
    msp.battery.enabled: false
    msp.gps.rate_hz: 3.0
''')
        logfile = open(Path(work)/'process.log', 'w+')
        try:
            recorder = subprocess.Popen(['ros2','bag','record','--output',work+'/bag',
                '/msp/events','/msp/attitude','/msp/status','/msp/config'], stdout=logfile, stderr=logfile)
            processes.append(recorder)
            executable = str(Path(get_package_prefix('agi_ros2')) / 'lib/agi_ros2/betaflight_msp_node')
            proc = subprocess.Popen([executable,
                '--ros-args','--params-file',str(config)], stdout=logfile, stderr=logfile)
            processes.append(proc)
            buffer = bytearray()
            received = collections.defaultdict(list)
            end = time.monotonic()+5
            while time.monotonic()<end:
                rclpy.spin_once(observer, timeout_sec=0)
                assert proc.poll() is None, 'MSP node exited'
                if not select.select([master], [], [], .001)[0]:
                    continue
                buffer.extend(os.read(master,4096))
                while len(buffer)>=6:
                    assert buffer[:3] == b'$M<'
                    size,code = buffer[3:5]
                    if len(buffer)<size+6:
                        break
                    frame = buffer[:size+6]
                    del buffer[:size+6]
                    check = 0
                    for b in frame[3:-1]:
                        check ^= b
                    assert check == frame[-1]
                    received[code].append(time.monotonic())
                    if code == 200:
                        assert frame[5:-1] == bytes([220,5,220,5,232,3,220,5])
                    else:
                        assert size == 0
                    if code == 106:
                        continue  # no GPS: record timeout; bench RC continues
                    payload = {1:b'\x00\x01\x31',2:b'BTFL',108:b'\x01\x00\x02\x00\x03\x00'}.get(code,b'')
                    check = len(payload)^code
                    for b in payload:
                        check ^= b
                    reply = b'$M>'+bytes([len(payload),code])+payload+bytes([check])
                    os.write(master,reply[:4])
                    os.write(master,reply[4:])
            for code,hz in [(200,100),(108,20),(101,5)]:
                times = received[code]
                measured = (len(times)-1)/(times[-1]-times[0])
                assert abs(measured-hz)<hz*.08, (code,measured)
                print(f'code {code}: {measured:.2f} Hz')
            assert len(received[106])==1
            assert not received[105] and not received[110] and not received[130]
            assert attitude and all(m.code==108 and m.event=='rx' and len(m.payload)==6 for m in attitude)
            assert any(m.code==200 and m.event=='tx' and len(m.payload)==8 for m in events)
            assert any(m.code==200 and m.event=='rx' for m in events)
            # Discovery may miss the initial timeout, but cumulative error persists.
            assert any(m.errors>=1 for m in events)
            assert any(m.code==108 and m.latency_seconds>=0 for m in attitude)
            assert all(m.header.stamp.sec>0 and m.steady_time>0 for m in events)
            for process in reversed(processes):
                process.send_signal(signal.SIGINT)
                process.wait(timeout=10)
            import rosbag2_py
            reader = rosbag2_py.SequentialReader()
            reader.open(rosbag2_py.StorageOptions(uri=work+'/bag', storage_id='sqlite3'),
                        rosbag2_py.ConverterOptions('', ''))
            counts = collections.Counter()
            while reader.has_next():
                topic, _, _ = reader.read_next()
                counts[topic]+=1
            assert counts['/msp/config']>=1
            assert counts['/msp/events']>100 and counts['/msp/attitude']>10 and counts['/msp/status']>2, counts
            print('PASS: UART rates, selected topics, raw frames/ACK, timestamps/latency, timeout isolation and rosbag messages', dict(counts))
        except Exception:
            logfile.flush()
            logfile.seek(0)
            print(logfile.read())
            raise
        finally:
            for process in processes:
                if process.poll() is None:
                    process.kill()
                    process.wait()
            logfile.close()
            os.close(master)
            os.close(slave)
            observer.destroy_node()
            rclpy.shutdown()


if __name__=='__main__':
    main()
