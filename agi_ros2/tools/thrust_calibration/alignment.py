"""Auditable affine effective alignment, with held-out common-event anchors."""
import itertools
import numpy as np


def detect_edges(t, y, threshold, separation_s):
    indices = np.flatnonzero(np.abs(np.diff(y)) >= threshold) + 1
    edges = []
    for i in indices:
        if not edges or t[i]-edges[-1]['time_s'] >= separation_s:
            edges.append({'time_s': float(t[i]), 'direction': int(np.sign(y[i]-y[i-1])),
                          'change': float(y[i]-y[i-1])})
    return edges


def fit_alignment(anchors, cfg, bf_interval):
    if len(anchors) < 8:
        raise ValueError('At least 6 fit + 2 held-out common edges required; two endpoints are not validation')
    x = np.array([r['bf_s'] for r in anchors], float)
    y = np.array([r['px4_s'] for r in anchors], float)
    train = np.array([r['role'] == 'fit' for r in anchors])
    test = np.array([r['role'] == 'check' for r in anchors])
    if not np.all(np.isfinite(x+y)) or np.sum(train) < 6 or np.sum(test) < 2 or not np.all(train | test):
        raise ValueError('Invalid anchor roles or times')
    if len(set(x)) != len(x) or np.any(np.diff(y[np.argsort(x)]) <= 0):
        raise ValueError('Repeated or crossed anchors')
    if any(r.get('direction') not in (-1, 1) or not r.get('evidence') for r in anchors):
        raise ValueError('Each anchor needs common-event direction and correspondence evidence')
    duration = bf_interval[1]-bf_interval[0]
    if duration <= 0 or min(x) < bf_interval[0] or max(x) > bf_interval[1]:
        raise ValueError('Anchor outside explicit BF segment')
    # Both fit and check must constrain the beginning and end.
    for mask in (train, test):
        if min(x[mask]) > bf_interval[0]+duration*.25 or max(x[mask]) < bf_interval[1]-duration*.25:
            raise ValueError('Fit/check anchors must cover both ends of segment')
    origin = float(np.mean(x[train]))
    xx = x-origin
    limit = float(cfg['max_residual_s'])
    drift = cfg['fit_drift']
    candidates = []
    for i, j in itertools.combinations(np.flatnonzero(train), 2):
        a = (y[j]-y[i])/(x[j]-x[i]) if drift else 1.
        if abs(a-1)*1e6 > cfg['max_drift_ppm']:
            continue
        b = y[i]-a*xx[i]
        good = train & (np.abs(y-(a*xx+b)) <= limit)
        candidates.append((int(sum(good)), a, b, good))
    if not candidates:
        raise ValueError('No bounded affine alignment')
    best = max(candidates, key=lambda c: c[0])
    good = best[3]
    if sum(good) < 6 or sum(train & ~good) > cfg['max_outlier_edges']:
        raise ValueError('Too many mismatched edges')
    if drift:
        a, b = np.linalg.lstsq(np.column_stack([xx[good], np.ones(sum(good))]), y[good], rcond=None)[0]
    else:
        a, b = 1., np.median(y[good]-xx[good])
    residual = y-(a*xx+b)
    uncertainty = max(float(np.max(np.abs(residual[good | test]))), float(cfg['minimum_uncertainty_s']))
    if abs(a-1)*1e6 > cfg['max_drift_ppm'] or np.max(np.abs(residual[test])) > limit:
        raise ValueError('Held-out edge/drift check failed')
    # Competing fits with substantial support and different offset are ambiguous.
    alternatives = [(aa, bb) for count, aa, bb, _ in candidates
                    if count >= best[0]-1 and abs(bb-b) > 2*limit]
    if alternatives:
        raise ValueError('Ambiguous correspondence: multiple supported offsets')
    return dict(formula='t_px4 = a * (t_bf - bf_origin_s) + px4_at_origin_s',
                bf_origin_s=origin, px4_at_origin_s=float(b), a=float(a),
                b_s=float(b-a*origin), drift_ppm=float((a-1)*1e6),
                uncertainty_s=uncertainty, applicable_bf_interval_s=bf_interval,
                method='robust supplied common-edge correspondences with held-out check',
                quality='steady_pairing', response_delay_included=True,
                pure_clock_offset=False, constraints=cfg, alternatives=[],
                edges=[dict(r, residual_s=float(e), inlier=bool(g)) for r, e, g in zip(anchors, residual, good | test)])


def mapped(t, alignment):
    return alignment['a']*(t-alignment['bf_origin_s'])+alignment['px4_at_origin_s']
