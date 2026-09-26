#!/usr/bin/env python3
"""Export scientific figures from saved, conditional EKF runs (no ROS)."""
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

HERE = Path(__file__).resolve().parent
BLUE, ORANGE, GREY, RED = '#2468a2', '#be760a', '#68747d', '#ae344b'


def read_table(name):
    with (HERE/'results'/name).open() as stream:
        rows = list(csv.DictReader(stream))
    return {key:np.array([r[key] for r in rows], dtype=str if key in ('status', 'source') else float)
            for key in rows[0]}


def input_residual(table, prefix, size):
    axes = ('x', 'y', 'z')[:size]
    return np.linalg.norm(np.array([table[f'post_{prefix}{a}']-table[f'gps_{prefix}{a}'] for a in axes]), axis=0)


def finish(fig, name):
    fig.savefig(HERE/'results'/f'{name}.png', dpi=170, bbox_inches='tight')
    fig.savefig(HERE/'results'/f'{name}.pdf', bbox_inches='tight')
    plt.close(fig)


def run():
    plt.rcParams.update({'font.size':11, 'axes.spines.top':False, 'axes.spines.right':False,
                         'axes.grid':True, 'grid.color':'#e1e5e8', 'grid.linewidth':.6,
                         'legend.frameon':False, 'lines.linewidth':1.6})
    main = read_table('linear_2s_updates.csv')
    state = read_table('linear_2s_states.csv')
    control = read_table('linear_2s_ungated_updates.csv')
    nav = np.genfromtxt(HERE/'inputs/nav_linear.csv', delimiter=',', names=True)
    imu = np.genfromtxt(HERE/'inputs/imu.csv', delimiter=',', names=True)
    gap_index = np.argmax(np.diff(imu['t']))
    gap = [imu['t'][gap_index], imu['t'][gap_index+1]]
    fig, axes = plt.subplots(1, 2, figsize=(13, 5), gridspec_kw={'width_ratios':[1, 1.4]})
    for epoch in (0, 1):
        n, s, c = nav[nav['epoch'] == epoch], state['segment'] == epoch, control['segment'] == epoch
        label = lambda text: text if epoch == 0 else None
        axes[0].plot(n['px'], n['py'], '.', color=GREY, ms=3, alpha=.7, label=label('GNSS input'))
        axes[0].plot(state['px'][s], state['py'][s], color=BLUE, label=label('EKF, NIS gate = 24.322'))
        axes[0].plot(control['post_px'][c], control['post_py'][c], '--', color=ORANGE,
                     label=label('Gate disabled (diagnostic only)'))
        start = np.flatnonzero(s)[0]
        axes[0].plot(state['px'][start], state['py'][start], 's', color=BLUE, ms=6)
        axes[0].annotate(f'Seed {epoch+1}', (state['px'][start], state['py'][start]),
                         xytext=(8, 6), textcoords='offset points', fontsize=10)
        axes[1].plot(n['t'], n['pz'], color=GREY, ls=':', label=label('MSL reconstructed from 1 Hz snapshots'))
        axes[1].plot(state['t'][s], state['pz'][s], color=BLUE, label=label('EKF'))
        axes[1].plot(control['t'][c], control['post_pz'][c], '--', color=ORANGE,
                     label=label('Gate disabled'))
    axes[0].set(xlabel='East (m)', ylabel='North (m)', title='Horizontal trajectory in local ENU')
    axes[0].set_aspect('equal', adjustable='datalim')
    axes[0].legend(loc='lower right', fontsize=9)
    axes[1].axvspan(*gap, color=GREY, alpha=.22)
    axes[1].set(xlabel='Corrected time from first IMU sample (s)', ylabel='Up relative to chosen origin (m)',
                title='Vertical trajectory; original GPS altitude is missing')
    axes[1].legend(loc='lower left', fontsize=9)
    fig.suptitle('Conditional native EKF replay | 2026-09-24 flight bag', y=1.03, fontsize=15)
    fig.text(.5, -.035, '36.148773 s wall-clock step removed; real IMU gap preserved. Two explicitly initialized segments; no independent position truth.',
             ha='center', fontsize=10)
    fig.tight_layout()
    finish(fig, 'trajectory')

    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    for table, color, linestyle, label in [(main, BLUE, '-', 'NIS gate = 24.322'),
                                          (control, ORANGE, '--', 'Gate disabled (diagnostic only)')]:
        for epoch in (0, 1):
            mask = table['segment'] == epoch
            t = table['t'][mask]
            axes[0].plot(t, input_residual(table, 'p', 2)[mask], linestyle, color=color,
                         label=label if epoch == 0 else None)
            axes[1].plot(t, input_residual(table, 'v', 3)[mask], linestyle, color=color)
            axes[2].plot(t, table['nis'][mask], linestyle, color=color, alpha=.85)
    rejected = main['status'] == 'rejected'
    axes[2].plot(main['t'][rejected], main['nis'][rejected], 'x', color=RED, ms=4, label='Rejected updates')
    axes[2].axhline(24.322, color='#343a40', ls=':', label='NIS threshold 24.322')
    axes[0].set(ylabel='Horizontal residual (m)', title='Posterior minus the same GNSS input: consistency, not position accuracy')
    axes[1].set(ylabel='3D velocity residual (m/s)')
    axes[2].set(ylabel='7D innovation NIS (log scale)', yscale='log',
                xlabel='Corrected time from first IMU sample (s)')
    for ax in axes:
        ax.axvspan(*gap, color=GREY, alpha=.22)
        ax.set_xlim(0, imu['t'][-1])
    axes[0].legend(loc='upper left', fontsize=10)
    axes[2].legend(loc='upper left', fontsize=10)
    fig.suptitle('Update consistency and rejection history', fontsize=15)
    fig.text(.5, .005, 'Gate-disabled runs are a sensitivity check, not a recommended flight setting. Missing IMU is never propagated across.',
             ha='center', fontsize=10)
    fig.tight_layout(rect=(0, .025, 1, .98))
    finish(fig, 'consistency')

    sensitivity = {}
    baseline = {(int(main['segment'][i]), main['t'][i]):i for i in range(len(main['t']))}
    for name in ['hold_2s', 'linear_1s', 'linear_2s_ungated']:
        table = read_table(name+'_updates.csv')
        rows = []
        for i, t in enumerate(table['t']):
            key = (int(table['segment'][i]), t)
            if key not in baseline:
                continue
            j = baseline[key]
            delta = [table['post_p'+a][i]-main['post_p'+a][j] for a in 'xyz']
            rows.append([key[0], np.linalg.norm(delta[:2]), abs(delta[2])])
        values = np.array(rows)
        sensitivity[name] = {str(epoch):{
            'common_updates':int(sum(values[:, 0] == epoch)),
            'horizontal_difference_max_m':float(values[values[:, 0] == epoch, 1].max()),
            'vertical_difference_max_m':float(values[values[:, 0] == epoch, 2].max())}
            for epoch in (0, 1)}
    (HERE/'results/sensitivity.json').write_text(json.dumps(sensitivity, indent=2)+'\n')
    print(json.dumps(sensitivity, indent=2))


if __name__ == '__main__':
    run()
