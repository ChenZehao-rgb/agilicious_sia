"""Native-sample windows, explicit gaps/boundaries, no high-rate interpolation."""
import json
import numpy as np
from log_io import read_columns
from semantics import times, numeric, raw_rc, force_newtons, VOLTAGE
from alignment import mapped


def stats(t, y):
    return dict(mean=float(np.mean(y)), median=float(np.median(y)), std=float(np.std(y)),
                span=float(np.ptp(y)), slope=float(np.polyfit(t-t[0], y, 1)[0]) if len(t)>1 else 0., count=len(y))


def load_pair(session, config):
    bf, bd = read_columns(session['bf']['file'], {**config['bf'], **session['bf']})
    px, pd = read_columns(session['px4']['file'], {**config['px4'], **session['px4']})
    for name, cols in [('bf', bf), ('px4', px)]:
        if 'sample_slice' in session[name]:
            lo, hi = session[name]['sample_slice']
            if not isinstance(lo, int) or not isinstance(hi, int) or not 0 <= lo < hi <= len(next(iter(cols.values()))):
                raise ValueError('Invalid explicit sample_slice')
            for key in cols: cols[key] = cols[key][lo:hi]
    tb, tp = times(bf, config['bf']), times(px, config['px4'])
    if not np.all(np.isfinite(tb)) or not np.all(np.isfinite(tp)) or np.any(np.diff(tb) <= 0) or np.any(np.diff(tp) <= 0):
        raise ValueError('Duplicate/reversed/reset timestamps: explicitly split/repair source segment before pairing')
    c, bv, cu = raw_rc(bf, config['bf']['axis'])
    raw, f, pv = force_newtons(px, config['px4'])
    v = numeric(bf, config['bf']['voltage_field'])*VOLTAGE[config['bf']['voltage_unit']]
    motors = np.column_stack([numeric(bf, name) for name in config['bf']['motor_fields']])
    if len(config['bf']['motor_fields']) != 4:
        raise ValueError('Specify all four motor fields; unavailable motors require a separate diagnostic-only analysis')
    return dict(tb=tb, tp=tp, c=c, bv=bv, cu=cu, raw=raw, f=f, pv=pv, v=v, motors=motors,
                bf=bf, px=px, dropouts=pd)


def extract(session, cfg, alignment):
    d = load_pair(session, cfg)
    tb, tp, c, f, v = (d[k] for k in ('tb', 'tp', 'c', 'f', 'v'))
    w = cfg['steady']
    lower, upper = session['bf']['interval_s']
    # RC steps and large motor steps; other profile/arm transitions split candidates too.
    cut = np.abs(np.diff(c)) > w['step_rc']
    cut |= np.max(np.abs(np.diff(d['motors'], axis=0)), axis=1) > w['motor_step']
    cut |= np.diff(tb) > w['bf_max_gap_s']
    for name in cfg['bf'].get('boundary_fields', []):
        cut |= np.diff(numeric(d['bf'], name)) != 0
    bounds = np.r_[0, np.flatnonzero(cut)+1, len(tb)]
    rows = []
    margin = w['transition_s'] + alignment['uncertainty_s']/alignment['a']
    for start, end in zip(bounds[:-1], bounds[1:]):
        l, h = max(tb[start], lower)+margin, min(tb[end-1], upper)-margin
        row = {k: session[k] for k in ('run_id','battery_id','block_id','arm_cycle','split','provenance','tested_motor_index')}
        row.update(start_bf_s=float(l), end_bf_s=float(h), accepted=False, reason='',
                   direction=session.get('direction', 'unknown'), throttle_raised=session.get('throttle_raised', 'unknown'),
                   alignment_id=alignment['alignment_id'], configuration_sha256=session['configuration_sha256'],
                   rc_uncertainty=d['cu'], margin_s=margin)
        reasons = []
        if h-l < w['min_duration_s']:
            row['reason'] = 'short/transition/boundary'; rows.append(row); continue
        bi = (tb>=l) & (tb<=h)
        pl, ph = mapped(l,alignment), mapped(h,alignment)
        row.update(start_px4_s=float(pl), end_px4_s=float(ph))
        if pl < session['px4']['interval_s'][0] or ph > session['px4']['interval_s'][1]:
            reasons.append('outside paired PX4 interval')
        u = alignment['uncertainty_s']
        summaries = []
        for shift in (-u, 0., u):
            pi = (tp>=pl+shift) & (tp<=ph+shift)
            if sum(pi)<w['min_force_samples'] or not np.all(d['pv'][pi]):
                reasons.append('insufficient/invalid force samples'); continue
            pts = tp[pi]
            if (np.max(np.diff(pts)) > w['px4_max_gap_s'] or pts[0]-(pl+shift)>w['px4_max_gap_s'] or
                    ph+shift-pts[-1]>w['px4_max_gap_s']):
                reasons.append('PX4 gap')
            for dropout in d['dropouts']:
                a = dropout['timestamp_us']*1e-6
                if a <= ph+shift and a+dropout['duration_ms']*.001 >= pl+shift:
                    reasons.append('ULog dropout')
            summary = stats(pts, f[pi]); summaries.append(summary)
            if summary['std']>w['force_std_n'] or abs(summary['slope'])>w['force_slope_n_s']:
                reasons.append('force unstable under alignment perturbation')
        if sum(bi)<2 or not np.all(d['bv'][bi]) or not np.all(np.isfinite(v[bi])) or not np.all(v[bi]>0):
            reasons.append('invalid RC/voltage')
        if sum(bi)>=2:
            row.update(axis_field=cfg['bf']['axis']['field'], axis_semantics=cfg['bf']['axis']['semantics'],
                       axis_raw_mean=float(np.mean(numeric(d['bf'],cfg['bf']['axis']['field'])[bi])),
                       timestamp_bf_first_raw=int(d['bf'][cfg['bf']['timestamp_field']][bi][0]) if cfg['bf']['timestamp_unit']=='us' else float(d['bf'][cfg['bf']['timestamp_field']][bi][0]),
                       voltage_source=cfg['bf']['voltage_field'])
            for side,cols,mask,tt in [('bf',d['bf'],bi,tb),('px4',d['px'],(tp>=pl)&(tp<=ph),tp)]:
                for field in cfg[side].get('diagnostic_fields',[]):
                    if sum(mask): row[side+'_'+field+'_mean']=float(np.mean(numeric(cols,field)[mask]))
            for key, values, maxstd, maxslope in (
                ('rc',c,w['rc_std'],w['rc_slope_s']), ('voltage',v,w['voltage_std_v'],w['voltage_slope_v_s'])):
                st=stats(tb[bi],values[bi]); row.update({key+'_'+k: val for k,val in st.items()})
                if st['std']>maxstd or abs(st['slope'])>maxslope:
                    reasons.append(key+' unstable')
            if np.max(np.diff(tb[bi]))>w['bf_max_gap_s']:
                reasons.append('BF gap')
            for i in range(4):
                st=stats(tb[bi],d['motors'][bi,i]); row.update({f'motor{i}_'+k:val for k,val in st.items()})
                if not all(np.isfinite(val) for val in st.values()) or st['std']>w['motor_std'] or abs(st['slope'])>w['motor_slope_s']:
                    reasons.append(f'motor{i} unstable')
        if len(summaries)==3:
            row.update({'force_'+key: value for key,value in summaries[1].items()})
            row['force_mean_early'] = summaries[0]['mean']
            row['force_mean_late'] = summaries[2]['mean']
            row['alignment_force_sensitivity_n'] = max(s['mean'] for s in summaries)-min(s['mean'] for s in summaries)
            if row['alignment_force_sensitivity_n']>w['max_alignment_force_change_n']:
                reasons.append('alignment sensitive')
            row['force_independent_samples'] = 'unknown'
            seq = cfg['px4'].get('sample_seq_field')
            if seq:
                pi=(tp>=pl)&(tp<=ph)
                row['force_independent_samples'] = len(np.unique(d['px'][seq][pi]))
            row['raw_force_mean'] = float(np.mean(d['raw'][(tp>=pl)&(tp<=ph)]))
        row.update(accepted=not reasons, reason='; '.join(sorted(set(reasons))))
        rows.append(row)
    # Tare is never estimated from idle. Report explicit baseline drift, do not detrend.
    tare=[]
    for interval in session.get('no_thrust_intervals_px4_s', []):
        mask=(tp>=interval[0]) & (tp<=interval[1])
        if sum(mask): tare.append(dict(interval=interval, **stats(tp[mask],f[mask])))
    return rows, tare
