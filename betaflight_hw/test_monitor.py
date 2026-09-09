"""End-to-end read-only diagnostic check against a PTY fake FC, no hardware."""
import os
import pty
import select
import signal
import subprocess
import sys
import time


def main():
    executable = sys.argv[1]
    master, slave = pty.openpty()
    proc = subprocess.Popen(
        [executable, "--monitor", os.ttyname(slave), "115200"],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
    )
    received = []
    buffer = bytearray()
    try:
        deadline = time.monotonic() + 3
        while len(received) < 15 and time.monotonic() < deadline:
            if not select.select([master], [], [], 0.1)[0]:
                if proc.poll() is not None:
                    break
                continue
            buffer.extend(os.read(master, 1024))
            while len(buffer) >= 6:
                if buffer[:3] != b"$M<":
                    raise AssertionError("unexpected protocol/CLI write")
                size, code = buffer[3:5]
                if len(buffer) < size + 6:
                    break
                frame = buffer[:size + 6]
                del buffer[:size + 6]
                assert size == 0, "monitor attempted a payload write"
                assert code in {1, 2, 3, 5, 64, 111, 101, 105, 110}
                assert frame[-1] == code
                received.append(code)
                payload = bytes([1, 2, 3])
                checksum = len(payload) ^ code
                for value in payload:
                    checksum ^= value
                os.write(master, b"$M>" + bytes([len(payload), code]) + payload + bytes([checksum]))
        assert len(received) >= 15, received
        assert received[:6] == [1, 2, 3, 5, 64, 111]
        proc.send_signal(signal.SIGINT)
        stdout, stderr = proc.communicate(timeout=2)
        assert proc.returncode == 0, stderr
        assert "MONITOR ONLY" in stdout
        assert len(stdout.splitlines()) >= 17
        print("PASS: monitor handshake, read-only requests, replies and SIGINT exit")
    finally:
        if proc.poll() is None:
            proc.kill()
            proc.wait()
        os.close(master)
        os.close(slave)


if __name__ == "__main__":
    main()
