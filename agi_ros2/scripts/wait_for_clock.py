#!/usr/bin/env python3
"""Wait for the host's initial time synchronization before hardware startup."""
import argparse
import math
import subprocess
import sys
import time


def positive_timeout(value):
    result = float(value)
    if not math.isfinite(result) or result <= 0:
        raise ValueError('clock_sync_timeout must be finite and positive')
    return result


def wait_for_sync(timeout):
    deadline = time.monotonic() + positive_timeout(timeout)
    reason = 'NTP synchronization pending'
    print('[AGI clock] Waiting for timedatectl NTPSynchronized=yes', flush=True)
    while time.monotonic() < deadline:
        try:
            result = subprocess.run(
                ['timedatectl', 'show', '--property=NTPSynchronized', '--value'],
                capture_output=True, text=True, timeout=max(.001, min(2., deadline - time.monotonic())))
            if result.returncode == 0 and result.stdout.strip() == 'yes':
                print('[AGI clock] Initial synchronization confirmed', flush=True)
                return True
            reason = result.stderr.strip() or 'NTPSynchronized=' + result.stdout.strip()
        except (OSError, subprocess.TimeoutExpired) as error:
            reason = str(error)
        remaining = deadline - time.monotonic()
        if remaining > 0:
            time.sleep(min(.5, remaining))
    print(f'[AGI clock] Hardware startup blocked: {reason}', file=sys.stderr, flush=True)
    return False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=positive_timeout, default=60.)
    return 0 if wait_for_sync(parser.parse_args().timeout) else 1


if __name__ == '__main__':
    sys.exit(main())
