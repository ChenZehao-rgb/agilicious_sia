"""Grouped voltage identification, block uncertainty and immutable held-out evaluation."""
import numpy as np


def pava(y, weights):
    pools=[]
    for i,(v,w) in enumerate(zip(y,weights)):
        pools.append([i,i,float(v),float(w)])
        while len(pools)>1 and pools[-2][2]>pools[-1][2]:
            b,a=pools.pop(),pools.pop(); w=a[3]+b[3]
            pools.append([a[0],b[1],(a[2]*a[3]+b[2]*b[3])/w,w])
    out=np.empty(len(y))
    for a,b,v,w in pools: out[a:b+1]=v
    return out


def regression(rows, width, cfg):
    c=np.array([r['rc_mean'] for r in rows]); v=np.array([r['voltage_mean'] for r in rows]); f=np.array([r['force_mean'] for r in rows])
    groups=np.floor(c/width+.5).astype(int)
    good=[]
    for g in np.unique(groups):
        ix=np.flatnonzero(groups==g)
        if len(ix)>=cfg['min_points_per_bin'] and np.ptp(v[ix])>=cfg['min_voltage_span_v']:
            if len({(rows[i]['battery_id'],rows[i]['block_id']) for i in ix}) >= cfg['min_train_blocks']:
                good.append(g)
    if len(good)<cfg['min_rc_bins']:
        raise ValueError('p unidentifiable: insufficient repeated RC/voltage/block overlap')
    mask=np.isin(groups,good); c,v,f,groups=c[mask],v[mask],f[mask],groups[mask]
    # Each bin gets its own intercept and local RC slope when jitter is observable.
    design=[np.log(v)]
    bin_info=[]
    for g in good:
        ix=groups==g; center=float(np.mean(c[ix])); design.append(ix.astype(float))
        jitter=(c-center)*ix
        slope_col=None
        if np.ptp(c[ix])>cfg['minimum_jitter_rc']:
            slope_col=len(design); design.append(jitter)
        bin_info.append(dict(group=int(g), center=center, rc_min=float(min(c[ix])), rc_max=float(max(c[ix])), mask=ix, slope_col=slope_col,
                             voltage_min=float(min(v[ix])),voltage_max=float(max(v[ix])), count=int(sum(ix))))
    X=np.column_stack(design); y=np.log(f)
    beta,_,rank,_=np.linalg.lstsq(X,y,rcond=None)
    if rank<X.shape[1] or np.linalg.cond(X)>cfg['max_design_condition'] or len(y)<=X.shape[1]+2:
        raise ValueError('RC/voltage confounding or insufficient residual degrees of freedom')
    residual=y-X@beta
    stderr=float(np.sqrt(np.linalg.inv(X.T@X)[0,0]*sum(residual**2)/(len(y)-X.shape[1])))
    local=[]
    for b in bin_info:
        ix=b['mask']; xx=np.log(v[ix]); yy=y[ix]
        if b['slope_col'] is not None: yy=yy-beta[b['slope_col']]*(c[ix]-b['center'])
        p=float(np.dot(xx-np.mean(xx),yy-np.mean(yy))/sum((xx-np.mean(xx))**2))
        b['local_log_slope_per_rc']=float(beta[b['slope_col']]) if b['slope_col'] is not None else 0.
        b['rc_effect_log_span']=abs(b['local_log_slope_per_rc'])*float(np.ptp(c[ix]))
        local.append(p); b['p_local']=p
    if max(local)-min(local)>cfg['max_local_p_spread']:
        raise ValueError('Single p fails across RC bins; restrict supported domain explicitly')
    return float(beta[0]),stderr,bin_info,(c,v,f,groups),residual


def metrics(rows, curve, p, vref, domain, floor):
    errors=[]; relative=[]; low_errors=[]; details=[]; excluded=[]
    cc,tt=np.array(curve).T
    for r in rows:
        c,v,f=r['rc_mean'],r['voltage_mean'],r['force_mean']
        if not(cc[0]<=c<=cc[-1] and domain[0]<=v<=domain[1]):
            excluded.append(r['run_id']); continue
        predicted=float(np.interp(c,cc,tt)*(v/vref)**p/4.)
        e=predicted-f; errors.append(e)
        if f>=floor: relative.append(abs(e)/f)
        else: low_errors.append(abs(e))
        details.append(dict(run_id=r['run_id'],block_id=r['block_id'],rc=c,voltage=v,force_n=f,predicted_single_n=predicted,
                            error_n=e,direction=r.get('direction','unknown'),throttle_raised=r.get('throttle_raised','unknown')))
    return dict(count=len(errors),excluded_count=len(excluded),excluded_runs=excluded,
                rmse_single_n=float(np.sqrt(np.mean(np.square(errors)))) if errors else None,
                max_low_force_absolute_error_n=max(low_errors) if low_errors else 0.,
                mean_relative_error=float(np.mean(relative)) if relative else None,
                max_relative_error=float(max(relative)) if relative else None,
                relative_denominator='measured positive single-motor N above configured floor',points=details)


def fit(rows,cfg):
    train=[r for r in rows if r['accepted'] and r['split']=='train' and r['force_mean']>cfg['force_log_floor_n']]
    validation=[r for r in rows if r['accepted'] and r['split']=='validation']
    if not validation: raise ValueError('Preassigned model-selection validation blocks required')
    width=cfg['rc_bin_width']
    p,se,bins,(c,v,f,g),residual=regression(train,width,cfg)
    if not cfg['p_min']<=p<=cfg['p_max'] or se>cfg['max_p_standard_error']:
        raise ValueError('p outside configured uncertainty/plausibility bounds')
    alignment_sensitivity=[]
    for key in ('force_mean_early', 'force_mean_late'):
        if all(key in r for r in train):
            shifted=[dict(r, force_mean=r[key]) for r in train]
            shifted_p=regression(shifted,width,cfg)[0]
            alignment_sensitivity.append(dict(perturbation=key,p=shifted_p))
            if abs(shifted_p-p)>cfg['max_alignment_p_change']:
                raise ValueError('p sensitive to alignment uncertainty')
    sensitivity=[]
    for factor in (.75,1.,1.25):
        pp,*_=regression(train,width*factor,cfg)
        sensitivity.append(dict(width=width*factor,p=pp))
    if np.ptp([x['p'] for x in sensitivity])>cfg['max_bin_p_change']:
        raise ValueError('p sensitive to RC bin width')
    vmin=max(b['voltage_min'] for b in bins); vmax=min(b['voltage_max'] for b in bins)
    if vmax-vmin<cfg['min_voltage_span_v']:
        raise ValueError('No common voltage domain across RC bins')
    vref=float(np.median(v)); vref=float(np.clip(vref,vmin,vmax))
    curves={}; candidates={}; raw_curves={}
    for label,pp in [('p0',0.),('p1',1.),('p2',2.),('p_fit',p)]:
        cs=[]; fs=[]; weights=[]
        for b in bins:
            ix=g==b['group']; center=int(round(b['center']))
            # Normalize local measured C jitter to the exported integer RC knot.
            ff=f[ix]*(vref/v[ix])**pp*np.exp(b['local_log_slope_per_rc']*(center-c[ix]))
            cs.append(center); fs.append(float(np.mean(ff)*4)); weights.append(sum(ix))
        mono=pava(fs,weights); curve=list(zip(cs,mono.tolist()))
        correction=float(max(abs(mono-np.array(fs))))
        curves[label]=curve; raw_curves[label]=list(zip(cs,fs))
        candidates[label]=dict(p=pp,monotonic_max_correction_total_n=correction,
                               validation=metrics(validation,curve,pp,vref,(vmin,vmax),cfg['relative_force_floor_n']))
    viable=[k for k in candidates if candidates[k]['validation']['count']>=cfg['min_validation_points']]
    if not viable: raise ValueError('Insufficient validation points inside training support')
    selected=min(viable,key=lambda k:candidates[k]['validation']['rmse_single_n'])
    if candidates[selected]['monotonic_max_correction_total_n']>cfg['max_monotonic_correction_total_n']:
        raise ValueError('Excessive monotonic correction; inspect hysteresis/axis')
    # Block bootstrap, preserving all windows within each block.
    blocks=sorted({(r['battery_id'],r['block_id']) for r in train})
    rng=np.random.default_rng(20261029); bootstrap=[]
    for _ in range(cfg['bootstrap_repeats']):
        sampled=[r for i in rng.integers(0,len(blocks),len(blocks)) for r in train if (r['battery_id'],r['block_id'])==blocks[i]]
        try: bootstrap.append(regression(sampled,width,{**cfg,'min_train_blocks':1})[0])
        except ValueError: pass
    if len(bootstrap)<cfg['bootstrap_repeats']//2:
        raise ValueError('Insufficient identifiable block bootstrap replicates')
    ci=np.quantile(bootstrap,[.025,.975]).tolist()
    if ci[1]-ci[0]>cfg['max_p_ci_width']:
        raise ValueError('Block uncertainty too wide for common p')
    return dict(alignment_sensitivity=alignment_sensitivity,p_fit=p,p_standard_error=se,p_block_ci95=ci,bin_sensitivity=sensitivity,
                bins=[{k:v for k,v in b.items() if k!='mask'} for b in bins],
                reference_voltage_v=vref,valid_voltage_min_v=vmin,valid_voltage_max_v=vmax,
                selected=selected,voltage_exponent_p=candidates[selected]['p'],curve=curves[selected],
                raw_curve=raw_curves[selected],candidates=candidates,training_log_residuals=residual.tolist(),
                train_blocks=blocks,validation_blocks=sorted({(r['battery_id'],r['block_id']) for r in validation}),
                sample_count=len(train),assumption='one measured motor scaled by four; no increase in sample count',
                provenance='synthetic' if any(r['provenance']=='synthetic' for r in rows) else 'measured')


def validate(rows, model, cfg):
    test=[r for r in rows if r['accepted'] and r['split']=='test']
    score=metrics(test,model['curve'],model['voltage_exponent_p'],model['reference_voltage_v'],
                  (model['valid_voltage_min_v'],model['valid_voltage_max_v']),cfg['relative_force_floor_n'])
    passed=(score['count']>=cfg['min_test_points'] and score['mean_relative_error'] is not None and
            score['mean_relative_error']<=cfg['mean_relative_limit'] and score['max_relative_error']<=cfg['max_relative_limit'] and
            score['max_low_force_absolute_error_n']<=cfg['low_force_absolute_limit_n'])
    # Every exported knot needs held-out nearby RC coverage over the declared voltage range.
    missing=[]
    for c,_ in model['curve']:
        vv=[r['voltage_mean'] for r in test if abs(r['rc_mean']-c)<=cfg['rc_bin_width']]
        if not vv or min(vv)>model['valid_voltage_min_v']+cfg['coverage_voltage_tolerance_v'] or max(vv)<model['valid_voltage_max_v']-cfg['coverage_voltage_tolerance_v']:
            missing.append(c)
    return dict(passed=bool(passed and not missing),heldout=score,uncovered_rc=missing,
                test_blocks=sorted({(r['battery_id'],r['block_id']) for r in test}),
                independent_final_test=bool(test),physical_compensation_validation='not_performed')
