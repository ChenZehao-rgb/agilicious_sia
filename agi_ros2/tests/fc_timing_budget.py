#!/usr/bin/env python3
"""Independent deterministic time-budget scan, not hardware replay or an FSM model."""
import json
import math
from pathlib import Path
import sys


def stats(values):
    values=sorted(values)
    def q(p):
        k=(len(values)-1)*p
        a=int(k); b=min(a+1,len(values)-1)
        return values[a]+(values[b]-values[a])*(k-a)
    return dict(n=len(values),p50=q(.5),p95=q(.95),p99=q(.99),max=values[-1])


def run():
    outcomes={}
    for trigger in ('independent_10ms_timer','new_sample'):
        pre=[];post=[];wait=[];skips=0;late=0
        for receive_age in (.00374,.00524,.00734):
            for compute in (.001,.00213):
                for phase in range(100):
                    # Explicit assumed post-receipt stages, not measurements.
                    adapter=.0002;delivery=.0004
                    waiting=phase*.0001 if trigger=='independent_10ms_timer' else 0.
                    age=receive_age+adapter+delivery+waiting
                    wait.append(waiting*1000);pre.append(age*1000)
                    if age>.015:
                        skips+=1
                    else:
                        post.append((age+compute)*1000)
                        late+=age+compute>.015
        outcomes[trigger]=dict(cases=len(pre),skipped_before=skips,finite_past_deadline=late,
                              age_before_ms=stats(pre),age_after_ms=stats(post),callback_wait_ms=stats(wait))
    assert outcomes['new_sample']['skipped_before']==0
    assert outcomes['new_sample']['finite_past_deadline']==0
    # A fresh computation does not close the lease gap to the next packet.
    outcomes['lease_gap_ms']=[dict(receive_age_ms=age*1000,
        next_receive_age_ms=(age+.01)*1000,raw_gap_ms=max(0,(age+.01-.015)*1000),
        assumed_next_callback_gap_ms=max(0,(age+.01+.0006-.015)*1000)) for age in (.00374,.00524,.00734)]
    outcomes['assumptions']={'state_period_ms':10,'health_period_ms':20,'adapter_ms':.2,'DDS_plus_executor_ms':.4,
        'compute_ms':[1,2.13],'output_delivery_check_ms':None,'purpose':'logic and time budget only; no OS/UART timing proof'}
    return outcomes

if __name__=='__main__':
    result=run()
    Path(sys.argv[1]).write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))
