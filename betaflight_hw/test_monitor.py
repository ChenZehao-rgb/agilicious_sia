"""PTY integration tests for the real hardware executable; no flight hardware."""
import collections
import os
import pty
import select
import subprocess
import sys
import time


def run(executable, bench=False, custom=False):
    master, slave = pty.openpty()
    args = [executable, '--bench' if bench else '--monitor',
            os.ttyname(slave), '115200', '--duration', '2']
    if bench:
        args += ['--props-removed', '--rc', '1510,1490,1000,1520']
    if custom:
        args += ['--rates', 'attitude=20,status=5,gps=3']
    # A temporary file prevents stdout backpressure from distorting the timing.
    import tempfile
    with tempfile.TemporaryFile(mode='w+') as output:
        proc = subprocess.Popen(args, stdout=output, stderr=subprocess.PIPE, text=True)
        received = collections.defaultdict(list)
        buffer = bytearray()
        try:
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline and proc.poll() is None:
                if not select.select([master], [], [], 0.02)[0]:
                    continue
                buffer.extend(os.read(master, 4096))
                while len(buffer) >= 6:
                    assert buffer[:3] == b'$M<'
                    size, code = buffer[3:5]
                    if len(buffer) < size + 6:
                        break
                    frame = buffer[:size + 6]
                    del buffer[:size + 6]
                    checksum = 0
                    for b in frame[3:-1]:
                        checksum ^= b
                    assert checksum == frame[-1]
                    received[code].append(time.monotonic())
                    if code == 200:
                        assert bench and size == 8
                        assert frame[5:-1] == bytes([230,5,210,5,232,3,240,5])
                        payload = b''
                    else:
                        assert size == 0
                        assert code in {1,2,3,5,64,111,108,105,101,110,130,106}
                        payload = {1: b'\x00\x01\x31', 2: b'BTFL'}.get(code, b'\x01\x02\x03')
                    if custom and code == 106:
                        continue  # Missing optional GPS reply must not stall RC.
                    checksum = len(payload) ^ code
                    for b in payload:
                        checksum ^= b
                    reply = b'$M>' + bytes([len(payload),code]) + payload + bytes([checksum])
                    # Fragment responses to exercise incremental decoding.
                    os.write(master, reply[:4])
                    os.write(master, reply[4:])
            _, stderr = proc.communicate(timeout=2)
            assert proc.returncode == 0, stderr
            expected = {108:20,101:5} if custom else {108:10,105:10,101:5,110:2,130:2,106:2}
            for code, hz in expected.items():
                assert abs(len(received[code]) - 2 * hz) <= 1, (code, len(received[code]))
            if bench:
                assert 190 <= len(received[200]) <= 201, len(received[200])
            else:
                assert not received[200]
            if custom:
                assert len(received[106]) == 1
                assert not received[105] and not received[110] and not received[130]
            output.seek(0)
            log = output.read()
            assert ',rx,108,' in log
            if custom:
                assert ',timeout,106,' in log
        finally:
            if proc.poll() is None:
                proc.kill()
                proc.wait()
            os.close(master)
            os.close(slave)


if __name__ == '__main__':
    run(sys.argv[1])
    run(sys.argv[1], bench=True, custom=True)
    for args in [['--bench', '/no/device'],
                 ['--monitor', '/no/device', '--rates', 'gps=nan'],
                 ['--monitor', '/no/device', '--rates', 'unknown=3']]:
        result = subprocess.run([sys.argv[1]] + args, capture_output=True)
        assert result.returncode != 0
        assert b'open /no/device' not in result.stderr
    print('PASS: default/custom rates, RC 100 Hz, AETR payload, fragmented replies, GPS timeout isolation, argument validation')
