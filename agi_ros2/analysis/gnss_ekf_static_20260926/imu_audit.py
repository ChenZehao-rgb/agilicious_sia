#!/usr/bin/env python3
"""IMU/attitude/timing audit of the first 100 bag seconds and the complete bag.

Consumes the original message-grain CSV extraction without modifying it.
The root analysis independently verifies the CSVs against the MCAP source.
All windows use bag log time minus metadata starting_time; state joins use
exact acquisition nanoseconds. No measured external attitude/position truth.
"""
from pathlib import Path
import json
import numpy as np
import pandas as pd
import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
BAG = "hardware_20260926_161859_702449"
SOURCE = HERE.parent / "hardware_diagnostic_20260926" / BAG
G = 9.8066


def stats(values):
    a = np.asarray(values, dtype=float)
    a = a[np.isfinite(a)]
    if not a.size:
        return {"n": 0}
    return dict(n=int(a.size), mean=float(a.mean()), std=float(a.std()),
                min=float(a.min()), p01=float(np.quantile(a, .01)),
                p05=float(np.quantile(a, .05)), median=float(np.median(a)),
                p95=float(np.quantile(a, .95)), p99=float(np.quantile(a, .99)), max=float(a.max()))


def vec(d, prefix):
    return d[[prefix + "." + x for x in "xyz"]].to_numpy(float)


def counts(d, fields):
    return {k: {str(v): int(c) for v, c in d[k].value_counts(dropna=False).items()} for k in fields}


def transitions(d, field):
    return d.loc[d[field].ne(d[field].shift()), ["bag_s", "stamp_s", field]].to_dict("records")


def summarize_imu(d):
    a, w = vec(d, "linear_acceleration"), vec(d, "angular_velocity")
    return dict(n=len(d), acc_mean_xyz=a.mean(0).tolist(), acc_std_xyz=a.std(0).tolist(),
                acc_mean_vector_norm=float(np.linalg.norm(a.mean(0))),
                acc_norm=stats(np.linalg.norm(a, axis=1)),
                gravity_mean_vector_residual=float(np.linalg.norm(a.mean(0))-G),
                gyro_mean_xyz_rad_s=w.mean(0).tolist(), gyro_std_xyz_rad_s=w.std(0).tolist(),
                gyro_mean_norm_rad_s=float(np.linalg.norm(w.mean(0))),
                gyro_norm_rad_s=stats(np.linalg.norm(w, axis=1)),
                finite_acc_gyro_rows=int(np.isfinite(np.column_stack((a, w))).all(axis=1).sum()))


def timing(d):
    t = d["header.stamp_ns"].to_numpy(np.int64)
    dt = np.diff(t) * 1e-9
    log_dt = np.diff(d.log_ns.to_numpy(np.int64)) * 1e-9
    return dict(n=len(d), first_bag_s=float(d.bag_s.iloc[0]), last_bag_s=float(d.bag_s.iloc[-1]),
                mean_hz=float((len(d)-1)/(dt.sum())), acquisition_gap_s=stats(dt),
                recorder_gap_s=stats(log_dt), acquisition_backwards=int((dt<0).sum()),
                acquisition_duplicates=int((dt==0).sum()), gaps_gt_5ms=int((dt>.005).sum()),
                gaps_gt_10ms=int((dt>.01).sum()), gaps_gt_25ms=int((dt>.025).sum()),
                recorder_delay_s=stats((d.log_ns-d["header.stamp_ns"])*1e-9),
                largest_gaps=[dict(after_bag_s=float(d.bag_s.iloc[j+1]), acquisition_gap_s=float(dt[j]),
                                   recorder_gap_s=float(log_dt[j])) for j in np.argsort(dt)[-10:][::-1]])


def summarize_state(d):
    q = d[["orientation." + x for x in "xyzw"]].to_numpy(float)
    norm = np.linalg.norm(q, axis=1)
    q = q/norm[:, None]
    x, y, z, w = q.T
    roll = np.arctan2(2*(w*x+y*z), 1-2*(x*x+y*y))
    pitch = np.arcsin(np.clip(2*(w*y-z*x), -1, 1))
    yaw = np.unwrap(np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z)))
    return dict(n=len(d), first_bag_s=float(d.bag_s.iloc[0]), last_bag_s=float(d.bag_s.iloc[-1]),
                quaternion_norm=stats(norm),
                euler_deg={k:stats(v*180/np.pi) for k,v in zip(["roll", "pitch", "yaw_unwrapped"], [roll,pitch,yaw])},
                acceleration={x:stats(d["acceleration."+x]) for x in "xyz"},
                acceleration_norm=stats(np.linalg.norm(vec(d,"acceleration"),axis=1)),
                body_rate={x:stats(d["body_rates."+x]) for x in "xyz"},
                velocity={x:stats(d["velocity."+x]) for x in "xyz"},
                velocity_norm=stats(np.linalg.norm(vec(d,"velocity"),axis=1)))


def main():
    data = {k:pd.read_csv(SOURCE / (k+".csv.gz")) for k in ["sensors_imu", "fused_state", "authority", "health", "parameter_events"]}
    i, f, a, h, p = (data[k] for k in data)
    meta = yaml.safe_load((ROOT / "bags" / BAG / "metadata.yaml").read_text())["rosbag2_bagfile_information"]
    start = meta["starting_time"]["nanoseconds_since_epoch"]
    for d in [i,f,a,h]:
        assert np.max(np.abs(d.bag_s - (d.log_ns-start)*1e-9)) < 1e-10
    out = dict(bag=BAG, source=str(SOURCE.relative_to(ROOT)), start_ns=start,
               duration_s=meta["duration"]["nanoseconds"]*1e-9,
               assumptions=["first100 window: 0 <= message recorder time - metadata start < 100 s",
                            "stationary is operator assertion; sensor statistics test consistency but do not prove truth",
                            "std uses ddof=0 population definition, same as startup checks",
                            "recorder delay includes device-to-ROS and DDS/recording, not pure device transport latency",
                            "no fusion parameter events or source commit in selected bag parameter stream; current code formulas are interpretive assumptions"],
               parameters=p[["node","new_parameters.0.name","new_parameters.0.value.double_value","new_parameters.0.value.string_value"]].fillna("").to_dict("records"),
               imu_timing=timing(i), fused_timing=timing(f),
               fused_transitions={k:transitions(f,k) for k in ["initialized","reset_counter"]},
               authority_transitions={k:transitions(a,k) for k in ["armed","auto_switch","kill","rc_link"]},
               windows={})
    for label,lo,hi in [("first100",0,100),("initialization_2p798_5p798",2.798,5.798),
                        ("10_30",10,30),("30_60",30,60),("60_100",60,100),
                        ("100_210",100,210),("210_227p87",210,227.87),("227p87_end",227.87,1e9),("all",0,1e9)]:
        si,sf,sa,sh=[d[(d.bag_s>=lo)&(d.bag_s<hi)] for d in [i,f,a,h]]
        fs=sf[sf.initialized]
        win=dict(imu=summarize_imu(si),imu_timing=timing(si),
                 fused_counts=counts(sf,["initialized","imu_ready","estimator_ready","navigation_ready","navigation_valid","accuracy_known","navigation_accuracy_ok","readiness_reason"]),
                 authority_counts=counts(sa,["armed","auto_switch","kill","rc_link"]),
                 health_counts=counts(sh,["imu_ready","estimator_ready","navigation_ready","config_verified","transport_healthy","thrust_calibrated","geofence_ok"]))
        if len(fs): win["state"]=summarize_state(fs)
        out["windows"][label]=win
    # Non-overlapping 3-second full windows, use acquisition timestamp membership.
    windows=[]
    for lo in np.arange(1,97,3):
        s=i[(i.stamp_s>=lo)&(i.stamp_s<lo+3)]
        r=summarize_imu(s)
        windows.append(dict(start_s=float(lo),end_s=float(lo+3),n=len(s),
                            acc_mean_vector_norm=r["acc_mean_vector_norm"],
                            gravity_residual=r["gravity_mean_vector_residual"],
                            max_acc_std=max(r["acc_std_xyz"]),
                            max_gyro_std=max(r["gyro_std_xyz_rad_s"]),gyro_mean_norm=r["gyro_mean_norm_rad_s"]))
    pd.DataFrame(windows).to_csv(HERE / "imu_stationary_3s_windows.csv",index=False)
    out["stationary_3s_windows"]={k:stats([w[k] for w in windows]) for k in ["gravity_residual","max_acc_std","max_gyro_std","gyro_mean_norm"]}
    # Exact timestamp match. Fused failure events can repeat timestamps; only initialized rows here.
    valid=f[f.initialized]
    m=valid.merge(i,on="header.stamp_ns",suffixes=("_f","_i"),validate="one_to_one")
    q=m[["orientation."+x+"_f" for x in "xyzw"]].to_numpy(float)
    q=q/np.linalg.norm(q,axis=1)[:,None]
    v=vec(m,"acceleration")-np.array([0,0,-G])
    qi=-q[:,:3]
    body=v+2*np.cross(qi,np.cross(qi,v)+q[:,3,None]*v)
    ba=vec(m,"linear_acceleration")-body
    bw=vec(m,"angular_velocity")-vec(m,"body_rates")
    derived=pd.DataFrame({"bag_s":m.bag_s_f,"stamp_s":m.stamp_s_f,"reset_counter":m.reset_counter,
                          **{"ba_"+x:ba[:,k] for k,x in enumerate("xyz")},
                          **{"bw_"+x:bw[:,k] for k,x in enumerate("xyz")}})
    derived.iloc[::25].to_csv(HERE/"imu_reconstructed_bias_20hz.csv",index=False)
    out["reconstructed_bias"]={"matched_n":len(m),"initialized_n":len(valid),
                              "assumption":"current EkfImu::vectorToState formulas; values are inferred filter state, not measured sensor calibration","windows":{}}
    for label,lo,hi in [("first100",0,100),("initialization",5.79,6),("10_30",10,30),("30_60",30,60),("60_100",60,100),("210_227p87",210,227.87),("last_reinit",244.79,1e9)]:
        d=derived[(derived.bag_s>=lo)&(derived.bag_s<hi)]
        out["reconstructed_bias"]["windows"][label]={k:stats(d[k]) for k in derived if k.startswith(("ba_","bw_"))}
    out["fusion_processing_s"]=stats(f.published_steady_time-f.imu_receive_time)
    out["fusion_imu_receive_gaps_s"]=stats(f.imu_receive_time.diff())
    out["fusion_same_acquisition_delta_s"]=stats((m.log_ns_f-m.log_ns_i)*1e-9)
    out["imu_covariance_orientation_flags"]=counts(i,["orientation_covariance.0","linear_acceleration_covariance.0","angular_velocity_covariance.0"])
    f["recorder_delay_s"]=(f.log_ns-f["header.stamp_ns"])*1e-9
    f["processing_s"]=f.published_steady_time-f.imu_receive_time
    out["fusion_latency_windows"]={}
    for lo,hi in [(0,100),(100,210),(210,219.7),(219.7,227.872),(227.872,248)]:
        d=f[(f.bag_s>=lo)&(f.bag_s<hi)]
        out["fusion_latency_windows"][f"{lo}_{hi}"]=dict(n=len(d),
            recorder_delay_s=stats(d.recorder_delay_s), processing_s=stats(d.processing_s),
            recorder_delay_gt_10ms_n=int((d.recorder_delay_s>.01).sum()),
            processing_gt_10ms_n=int((d.processing_s>.01).sum()))
    out["largest_fusion_processing_events"]=f.nlargest(12,"processing_s")[["bag_s","stamp_s","recorder_delay_s","processing_s","navigation_rejections","readiness_reason"]].to_dict("records")
    sh=h[(h.bag_s>=5.8)&(h.bag_s<100)]
    out["postinit_first100_health"]=dict(n=len(sh),imu_not_ready_n=int((~sh.imu_ready).sum()),
        not_ready_reasons=counts(sh[~sh.imu_ready],["reason"]))
    topic_names=[x["topic_metadata"]["name"] for x in meta["topics_with_message_count"]]
    out["controller_topics_in_metadata"]={k:k in topic_names for k in ["/control_command","/output_status","/computation_status"]}
    # Heading is an Euler angle: large differences near vertical body X need
    # not be equally large physical attitude rotations. Motion is unlabelled
    # outside the first100 interval; never call these magnetic faults by default.
    n=pd.read_csv(SOURCE/"sensors_navigation.csv.gz")
    n["jump_deg"]=np.rad2deg(np.arctan2(np.sin(n.heading.diff()),np.cos(n.heading.diff())))
    gyro=np.linalg.norm(vec(i,"angular_velocity"),axis=1)
    acc=vec(i,"linear_acceleration")
    anorm=np.linalg.norm(acc,axis=1)
    i["gyro_norm"]=gyro
    i["acc_norm"]=anorm
    i["body_x_gravity_abs_ratio"]=np.abs(acc[:,0])/anorm
    i["body_z_gravity_abs_ratio"]=np.abs(acc[:,2])/anorm
    q=f[["orientation."+x for x in "xyzw"]].to_numpy(float)
    x,y,z,w=q.T
    f["pitch_deg"]=np.rad2deg(np.arcsin(np.clip(2*(w*y-z*x),-1,1)))
    f["roll_deg"]=np.rad2deg(np.arctan2(2*(w*x+y*z),1-2*(x*x+y*y)))
    heading_events=[]
    for k in n.index[n.jump_deg.abs()>20]:
        lo,hi=n.stamp_s.iloc[k-1],n.stamp_s.iloc[k]
        si=i[(i.stamp_s>=lo)&(i.stamp_s<=hi)]
        padded=i[(i.stamp_s>=lo-.15)&(i.stamp_s<=hi+.15)]
        sf=f[(f.stamp_s>=lo)&(f.stamp_s<=hi)&f.initialized]
        heading_events.append(dict(bag_s=float(n.bag_s.iloc[k]),acquisition_lo_hi_s=[float(lo),float(hi)],
            heading_jump_deg=float(n.jump_deg.iloc[k]),gyro_max_deg_s=float(si.gyro_norm.max()*180/np.pi),
            raw_gyro_path_deg=float(np.trapz(si.gyro_norm,si.stamp_s)*180/np.pi),
            raw_gyro_path_padded150ms_deg=float(np.trapz(padded.gyro_norm,padded.stamp_s)*180/np.pi),
            acc_norm=stats(si.acc_norm),body_x_gravity_abs_ratio=stats(si.body_x_gravity_abs_ratio),
            body_z_gravity_abs_ratio=stats(si.body_z_gravity_abs_ratio),
            fused_pitch_deg=stats(sf.pitch_deg),fused_roll_deg=stats(sf.roll_deg)))
    out["heading_jump_crosscheck"]={"events":heading_events,"caveats":[
        "raw gyro path is uncorrected integral of angular-speed norm, approximately attitude rotation path, not a bound on Euler yaw",
        "accelerometer gravity-direction proxy is approximate because translation contributes specific force",
        "fused attitude already uses navigation heading and is not independent heading truth",
        "navigation acquisition stamp represents paired GNSS message; heading timing may differ, so also show padded 150 ms"]}
    pd.DataFrame([{k:v for k,v in r.items() if not isinstance(v,dict)} for r in heading_events]).to_csv(HERE/"imu_heading_jump_events.csv",index=False)
    (HERE/"imu_results.json").write_text(json.dumps(out,indent=2,allow_nan=False))
    print(json.dumps({"first100_imu":out["windows"]["first100"]["imu"],"transitions":out["fused_transitions"],"first100_state":out["windows"]["first100"].get("state")},indent=2))


if __name__ == "__main__":
    main()
