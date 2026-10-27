#!/usr/bin/env python3
"""Same synthetic 100 Hz publisher, phase offsets and 50 Hz health epochs, two binaries.

Runs sequentially with real DDS; this is not replay of the aircraft's scheduler.
FC_TIMER_BASELINE names the executable produced by build_timer_baseline.py.
"""
import collections
import json
import math
import os
from pathlib import Path
import time
import rclpy
from test_fc_control_delivery import DeliveryHarness, distribution

out=Path(os.environ['FC_COMPARISON_REPORT'])
reports={}
rclpy.init()
try:
    for name,executable in [('timer_baseline',os.environ['FC_TIMER_BASELINE']),('sample_driven','control_node')]:
        os.environ['FC_CONTROL_EXECUTABLE']=executable
        h=DeliveryHarness()
        results=[]
        try:
            h.run(1.)
            for age in (.00374,.00524,.00734):
                for phase in (0.,.002,.004,.006,.008):
                    h.age=age
                    h.next_state=time.monotonic()+phase
                    h.run(.08)
                    start=len(h.received['control_command'])
                    comp_start=len(h.received['computation_status'])
                    h.run(.6)
                    commands=h.received['control_command'][start:]
                    comp=h.received['computation_status'][comp_start:]
                    computed=[c for c in commands if math.isfinite(c.evidence.solve_seconds)]
                    stages={
                        'sample_to_receive_ms':[(c.evidence.fc_receive_time-c.evidence.fc_sample_time)*1000 for c in computed],
                        'receive_to_publish_ms':[(c.state_published_steady_time-c.evidence.fc_receive_time)*1000 for c in computed],
                        'publish_to_callback_ms':[(c.state_received_steady_time-c.state_published_steady_time)*1000 for c in computed],
                        'callback_to_start_ms':[(c.control_start_steady_time-c.state_received_steady_time)*1000 for c in computed],
                        'age_before_ms':[(c.control_start_steady_time-c.evidence.fc_sample_time)*1000 for c in computed],
                        'age_after_ms':[(c.evidence.now-c.evidence.fc_sample_time)*1000 for c in computed],
                        'compute_ms':[c.evidence.solve_seconds*1000 for c in computed],
                        'start_to_publish_ms':[(c.published_steady_time-c.control_start_steady_time)*1000 for c in computed],
                        'compute_gap_ms':[(b.control_start_steady_time-a.control_start_steady_time)*1000 for a,b in zip(computed,computed[1:])],
                    }
                    results.append(dict(age_ms=age*1000,phase_ms=phase*1000,
                        notifications=len(commands),computations=len(computed),
                        skipped_compute_requests=sum(not math.isfinite(c.solve_seconds) and (name=='timer_baseline' or c.computation_requested) for c in comp),
                        safety_notifications=sum(not c.computation_requested for c in comp) if name=='sample_driven' else 0,
                        finite_post_deadline=sum(c.evidence.now-c.evidence.fc_sample_time>.015 for c in computed),
                        repeated_compute_samples=len(computed)-len({(c.evidence.fc_session,c.evidence.fc_sample_time) for c in computed}),
                        warm_resets=sum(a.warm_cycles>0 and b.warm_cycles==0 for a,b in zip(comp,comp[1:])),
                        compute_hz=(len(computed)-1)/(computed[-1].control_start_steady_time-computed[0].control_start_steady_time) if len(computed)>1 else None,
                        skip_reasons=dict(collections.Counter(c.reason for c in comp if not math.isfinite(c.solve_seconds))),
                        stages={k:distribution(v) for k,v in stages.items()}, raw_stage_values=stages))
            all_stages={k:[v for r in results for v in r['raw_stage_values'][k]] for k in results[0]['raw_stage_values']}
            reports[name]=dict(cases=[{k:v for k,v in r.items() if k!='raw_stage_values'} for r in results],
                total={k:sum(r[k] for r in results) for k in ['notifications','computations','skipped_compute_requests','safety_notifications','finite_post_deadline','repeated_compute_samples','warm_resets']},
                stages={k:distribution(v) for k,v in all_stages.items()})
            print(name,reports[name]['total'],flush=True)
        finally: h.close()
finally:
    rclpy.shutdown()
    out.write_text(json.dumps(reports,indent=2))
