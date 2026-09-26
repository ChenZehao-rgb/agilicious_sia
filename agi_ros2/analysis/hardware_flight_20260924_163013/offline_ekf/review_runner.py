#!/usr/bin/env python3
"""Independently check real-bag GPS delivery ordering and rejection clusters."""
import csv
import json
from pathlib import Path
import subprocess
import tempfile

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]


def read_csv(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def main():
    result = json.loads((HERE/'results/results.json').read_text())['cases'][0]
    assert result['name'] == 'linear_2s'
    imu = np.genfromtxt(HERE/'inputs/imu.csv', delimiter=',', skip_header=1)
    nav = np.genfromtxt(HERE/'inputs/nav_linear.csv', delimiter=',', skip_header=1)
    reference = read_csv(HERE/'results/linear_2s_updates.csv')
    profile = HERE/'inputs/hardware_snapshot.yaml'
    if not profile.exists():
        profile = ROOT/'agi_ros2/config/hardware.yaml'
    cuts = np.flatnonzero(np.diff(imu[:, 0]) > .025)+1
    parts = np.split(imu, cuts)
    assert len(parts) == len(result['seeds']) == 2
    fields = ['post_px','post_py','post_pz','post_vx','post_vy','post_vz','post_yaw',
              'pvarx','pvary','pvarz','vvarx','vvary','vvarz','yawvar','nis']
    audit = []
    with tempfile.TemporaryDirectory(prefix='agi-ekf-delayed-bag-') as temporary:
        directory = Path(temporary)
        for epoch, (samples, seed) in enumerate(zip(parts, result['seeds'])):
            gps = nav[(nav[:, 1] == epoch) & (nav[:, 0] >= seed['time_s'])]
            first = gps[0]
            time = first[0]
            assert samples[0, 0] <= time <= gps[-1, 0] <= samples[-1, 0]
            initial = ['S', time, epoch, *first[2:8], *seed['quaternion_wxyz'],
                       *seed['gyro_bias'], *seed['acceleration_bias'], *first[9:15]]
            events = [(row[0],1,['I',*row]) for row in samples if row[0] >= time]
            events += [(row[0]+.02,0,['G',row[0],*row[2:]]) for row in gps[1:]]
            events.sort(key=lambda event:(event[0],event[1]))
            with (directory/'events.csv').open('w', newline='') as stream:
                csv.writer(stream).writerows([initial]+[event[2] for event in events])
            process = subprocess.run([str(HERE/'offline_ekf'), str(directory/'events.csv'),
                                      str(directory/'output'), '24.322', '.025',
                                      str(profile)],
                                     capture_output=True, text=True)
            assert process.returncode == 0, process.stderr
            actual = read_csv(directory/'output_updates.csv')
            expected = [row for row in reference if int(row['segment']) == epoch]
            assert len(actual) == len(expected)
            assert [row['status'] for row in actual] == [row['status'] for row in expected]
            difference = {field:max(abs(float(a[field])-float(b[field])) for a,b in zip(actual,expected))
                          for field in fields}
            # State-only integration can differ at submicrometre level because
            # getAt cache split times differ. The posterior/NIS must not change.
            assert max(difference[field] for field in fields if field.startswith('post_')) < 1e-6
            assert max(difference[field] for field in fields if not field.startswith('post_')) < 1e-12
            audit.append({'epoch':epoch, 'updates':len(actual), 'delay_seconds':.020,
                          'statuses_match':True, 'max_absolute_differences':difference})
    clusters, current = [], []
    for row in reference:
        if row['status'] == 'rejected':
            current.append(row)
        elif current:
            clusters.append(current)
            current = []
    if current:
        clusters.append(current)
    innovations = ['innovation_px','innovation_py','innovation_pz','innovation_vx',
                   'innovation_vy','innovation_vz','innovation_yaw']
    review = {
        'coordinate_and_field_mapping_review':
            'S/G indexes, quaternion wxyz, FLU IMU and ENU navigation match native interfaces.',
        'source_session_and_gap_review':
            'The two navigation epochs correspond to the two IMU intervals; real gap '
            '41.795901696 to 42.205385216 s; second explicit seed is 42.360858368 s.',
        'real_bag_delayed_delivery':audit,
        'rejection_clusters':[
            {'start_s':float(group[0]['t']), 'end_s':float(group[-1]['t']), 'count':len(group),
             'maximum_nis':max(float(row['nis']) for row in group),
             'initial_innovation':{field:float(group[0][field]) for field in innovations}}
            for group in clusters],
        'interpretation':
            'Terminal rejection starts at 84.266281216 s, well after the clock/session restart. '
            'Velocity/position innovations are already inconsistent at onset; rejection removes correction '
            'and discrepancies grow. NIS uses correlated covariance: axis magnitudes alone are not an exact '
            'NIS decomposition. Actual-bag delayed delivery preserves NIS, covariance and decisions; '
            'submicrometre rejected-state integration differences cannot explain metre-scale discrepancies. '
            'No CSV/cache artifact found. Sensor latency, IMU bias/vibration, initialization, reconstructed '
            'altitude and noise modelling remain candidates, not established causes.',
        'robustness_note':
            'run_analysis.py maps two IMU chunks to source epochs by enumeration. Verified for this bag; '
            'a general tool should explicitly validate every chunk/session association.'}
    (HERE/'runner_review.json').write_text(json.dumps(review, indent=2)+'\n')
    print(json.dumps(audit, indent=2))


if __name__ == '__main__':
    main()
