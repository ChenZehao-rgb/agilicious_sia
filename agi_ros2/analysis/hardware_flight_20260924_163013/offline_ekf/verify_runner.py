#!/usr/bin/env python3
"""Retain auditable synthetic CSV cases and verify the native offline runner."""
import csv
import json
import math
from pathlib import Path
import subprocess

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]


def initial(time=1., segment=0):
    return ['S', time, segment, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1]


def imu(time):
    return ['I', time, 0, 0, 9.8066, 0, 0, 0]


def gps(time, position=0.):
    return ['G', time, position, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, .03]


def read_csv(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def run_case(directory, name, rows, nis):
    event_file = directory/f'{name}_events.csv'
    with event_file.open('w', newline='') as stream:
        csv.writer(stream).writerows(rows)
    prefix = directory/name
    result = subprocess.run([str(HERE/'offline_ekf'), str(event_file), str(prefix),
                             str(nis), '.025', str(HERE/'inputs/hardware_snapshot.yaml')],
                            capture_output=True, text=True)
    (directory/f'{name}.log').write_text(result.stdout+result.stderr)
    if result.returncode:
        raise AssertionError(result.stderr)
    return {kind:read_csv(directory/f'{name}_{kind}.csv') for kind in ['states','updates','segments']}


def main():
    directory = HERE/'unitcases'
    directory.mkdir(exist_ok=True)
    rows = [initial()]
    for i in range(21):
        time = 1+i*.01
        rows.append(imu(time))
        if i == 10:
            rows.append(gps(time))
        if i == 20:
            rows.append(gps(time, 100))
    rows.extend([imu(1.3), gps(1.3), initial(1.3, 1), imu(1.3), gps(1.305), imu(1.31)])
    checks = {}
    for name, nis in [('gated',24.322), ('ungated','inf')]:
        result = run_case(directory, name, rows, nis)
        statuses = [row['status'] for row in result['updates']]
        expected = ['accepted','rejected' if name == 'gated' else 'accepted','segment_inactive','accepted']
        assert statuses == expected, statuses
        assert float(result['updates'][1]['nis']) > 24.322
        assert all(len(row) == 35 and None not in row for row in result['updates'])
        assert [row['action'] for row in result['segments']] == ['initialized','stopped_imu_gap','initialized']
        drift = max(abs(float(row['pz'])) for row in result['states'])
        assert drift < 1e-10
        checks[name] = {'statuses':statuses, 'outlier_nis':float(result['updates'][1]['nis']),
                        'max_stationary_vertical_drift_m':drift}

    # Changing IMU data plus delayed measurement delivery exercises getAt cache
    # rewind and addRtk replay independently of the stationary acceptance case.
    schedules = {}
    for name, delay in [('immediate',0.), ('delayed',.020)]:
        events = []
        for i in range(501):
            time = 1+i*.002
            sample = ['I',time,.2*math.sin(i*.03),.1*math.cos(i*.017),9.8066,
                      .02*math.sin(i*.01),.01*math.cos(i*.02),.1]
            events.append((time,1,sample))
        for i in range(1,10):
            time = 1+i*.1
            fix = gps(time,.02*math.sin(i))
            fix[8] = .1*(time-1)
            events.append((time+delay,0,fix))
        events.sort(key=lambda event:(event[0],event[1]))
        schedules[name] = run_case(directory, name, [initial()]+[event[2] for event in events], 'inf')
    left, right = schedules['immediate']['updates'], schedules['delayed']['updates']
    assert len(left) == len(right) == 9
    fields = ['post_px','post_py','post_pz','post_vx','post_vy','post_vz','post_yaw',
              'pvarx','pvary','pvarz','vvarx','vvary','vvarz','yawvar','nis']
    differences = {field:max(abs(float(a[field])-float(b[field])) for a,b in zip(left,right)) for field in fields}
    assert max(differences.values()) < 1e-10, differences
    assert all(row['status'] == 'accepted' for row in left+right)
    checks['immediate_vs_delayed'] = {'gps_delay_seconds':.020, 'updates_per_case':9,
                                    'max_absolute_differences':differences}
    checks['passed'] = True
    checks['gravity_m_s2'] = 9.8066
    (directory/'verification.json').write_text(json.dumps(checks, indent=2)+'\n')
    print(json.dumps(checks, indent=2))


if __name__ == '__main__':
    main()
