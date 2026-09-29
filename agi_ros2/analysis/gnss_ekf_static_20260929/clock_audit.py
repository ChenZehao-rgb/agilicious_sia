#!/usr/bin/env python3
"""Recompute the clock jump and recovery blockers from extracted bag tables.

The msp_events_clock_audit table was decoded directly from /msp/events using
its embedded ROS2 schema. No runtime policy or production configuration is changed.
"""
import ast
import json
from pathlib import Path
import sys

import numpy as np
import pandas as pd
import yaml

BASE = Path(__file__).resolve().parent
BAG = BASE / "hardware_20260929_112215_827325"
ROOT = BASE.parents[2]
sys.path.insert(0, str(ROOT / "agi_ros2/scripts"))
from shadow_support import MspEvidence


def load(name):
    return pd.read_csv(BAG / (name + ".csv.gz"))


def selected(row, cols):
    return {c: row[c].item() if isinstance(row[c], np.generic) else row[c] for c in cols}


def main():
    events = load("msp_events_clock_audit")
    fused = load("fused_state")
    authority = load("authority")
    decoded = load("msp_decoded_state")
    navigation = load("sensors_navigation")
    local = load("sensors_local_navigation")
    origin = load("navigation_origin")
    status = load("sensors_mavlink_status")
    events["wall_request_age_s"] = events.stamp_s - events.request_stamp_s
    events["steady_request_age_s"] = events.steady_time - events.request_steady_time
    events["clock_step_s"] = events.wall_request_age_s - events.steady_request_age_s
    anomalous = events[(events.event == "rx") & (events.clock_step_s.abs() > .05)]
    assert len(anomalous) == 1
    jump = anomalous.iloc[0]
    post = events[events.bag_s > jump.bag_s + .1]
    offsets_before = events[events.bag_s < 43]
    offsets_after = events[events.bag_s > 77]
    offset_before = (offsets_before.stamp_s - offsets_before.steady_time).median()
    offset_after = (offsets_after.stamp_s - offsets_after.steady_time).median()
    source_changes = navigation[navigation.source_session.ne(navigation.source_session.shift())]
    revoked = local[~local.observation_valid].iloc[0]
    recovered = local[(local.bag_s > revoked.bag_s) & local.observation_valid].iloc[0]
    post_authority = authority[authority.bag_s >= revoked.bag_s]
    reset_row = fused[(fused.bag_s >= revoked.bag_s) & ~fused.initialized].iloc[0]
    post_fused = fused[fused.bag_s >= reset_row.bag_s]
    # Counterfactual audit only: replay authentic MSP replies without the ROS-node
    # cross-clock error injection, preserving normal decoder checks and payloads.
    # This does not claim the actual live evidence node recovered.
    config = yaml.safe_load((ROOT / "agi_ros2/config/hardware.yaml").read_text())
    expected = dict(config["bridge"], **config["evidence"])
    decoder = MspEvidence(expected)
    replay_snapshots = []
    for row in events.itertuples():
        decoder.accept(row.code, ast.literal_eval(row.payload), row.request_stamp_s + 1000,
                       row.session_id, "" if pd.isna(row.request_name) else row.request_name,
                       row.event)
        if row.event == "rx" and row.code == 150 and row.bag_s > recovered.bag_s:
            snap = decoder.snapshot(row.stamp_s + 1000)
            replay_snapshots.append(dict(bag_s=row.bag_s, **snap))
    replay = pd.DataFrame(replay_snapshots)
    replay_verified = replay[replay.config_verified]
    assert events.errors.eq(0).all()
    assert set(events.event) == {"rx", "tx"}
    assert events.session_id.nunique() == 1
    assert len(post_authority) and not post_authority.rc_link.any()
    assert post_authority.kill.all() and not post_authority.armed.any()
    assert not post_fused.initialized.any()
    assert local[local.bag_s >= recovered.bag_s].observation_valid.all()
    assert origin.session_id.nunique() == 1
    assert origin[["latitude", "longitude", "altitude"]].nunique().max() == 1
    assert len(replay_verified) and replay_verified.rc_link.all()
    assert not replay_verified.kill.any() and not replay_verified.armed.any()
    result = {
        "clock_jump": {
            "primary_evidence": "Single matched MSP request/reply, wall-clock elapsed minus monotonic elapsed",
            "matched_request_reply": selected(jump, ["bag_s", "stamp_s", "steady_time", "request_stamp_s",
                                                        "request_steady_time", "code", "wall_request_age_s",
                                                        "steady_request_age_s", "clock_step_s", "latency_seconds"]),
            "median_ros_minus_steady_before_s": offset_before,
            "median_ros_minus_steady_after_s": offset_after,
            "median_offset_change_s": offset_after - offset_before,
            "explicit_mavlink_clock_jump_status": status.loc[
                status["status.0.message"].str.contains("ROS clock jumped"),
                ["bag_s", "stamp_s", "status.0.message"]].to_dict("records"),
            "wallclock_gap_does_not_equal_data_outage": True,
            "fused_first_to_last_monotonic_span_s": fused.published_steady_time.iloc[-1] - fused.published_steady_time.iloc[0],
            "fused_first_to_last_bag_wallclock_span_s": fused.bag_s.iloc[-1] - fused.bag_s.iloc[0],
            "fused_max_publication_monotonic_gap_s": fused.published_steady_time.diff().max(),
            "source_epoch_changes": source_changes[["bag_s", "stamp_s", "source_session", "clock_aligned"]].to_dict("records"),
        },
        "navigation_recovery": {
            "first_revocation": selected(revoked, ["bag_s", "stamp_s", "source_session", "reason"]),
            "first_valid_local_navigation_after_revocation": selected(recovered, ["bag_s", "stamp_s", "source_session", "reason"]),
            "revocation_to_valid_local_navigation_wall_s": recovered.bag_s - revoked.bag_s,
            "valid_local_navigation_count_after_recovery": int((local.bag_s >= recovered.bag_s).sum()),
            "origin_coordinates_and_session_constant": True,
        },
        "reinitialization_blocker": {
            "msp_reason_transitions": decoded.loc[decoded.reason.ne(decoded.reason.shift()),
                                                      ["bag_s", "stamp_s", "reason", "rc_link", "kill", "armed"]].to_dict("records"),
            "authority_false_rc_link_from_bag_s": post_authority.bag_s.iloc[0],
            "authority_false_rc_link_persists_to_bag_s": post_authority.bag_s.iloc[-1],
            "all_post_reset_initialized_false": True,
            "mechanism": "msp_evidence.py injects transport_error for the cross-clock request; MspEvidence.failed latches for unchanged session; configuration() returns failure; snapshot returns rc_link=false and kill=true defaults; StateFusionNode requires fresh authority.rc_link and not armed to initialize.",
            "kill_and_rc_link_are_evidence_defaults_not_proof_of_physical_switch_or_rf_loss": True,
        },
        "raw_msp_transport": {
            "event_counts": events.event.value_counts().to_dict(),
            "error_field_nonzero_count": int(events.errors.ne(0).sum()),
            "session_count": int(events.session_id.nunique()),
            "post_jump_rx_counts_by_code": {str(k): int(v) for k, v in post[post.event == "rx"].groupby("code").size().items()},
            "last_rx_bag_s": events[events.event == "rx"].bag_s.iloc[-1],
            "normal_payload_replay_without_wallclock_error_injection": {
                "counterfactual_not_live_recovery": True,
                "first_verified_snapshot_bag_s": replay_verified.bag_s.iloc[0],
                "verified_status_snapshot_count": len(replay_verified),
                "all_rc_link_true": bool(replay_verified.rc_link.all()),
                "all_kill_false": bool((~replay_verified.kill).all()),
                "all_armed_false": bool((~replay_verified.armed).all()),
                "config_file_used": "agi_ros2/config/hardware.yaml",
            },
        },
        "boundaries": [
            "The bag proves a ROS/wall-clock step; the clock-setting process (NTP/chrony/operator) is not recorded.",
            "Normal serial replies and decoder replay refute a need to infer physical RF/serial failure from the latched authority alone.",
            "Exact ROS callback receipt time is not recorded; the matched MSP event has enough independent evidence to violate both 3-second wall age and 50-ms clock disagreement rules.",
            "The origin remains unchanged; source_session reset is a time synchronization epoch change.",
        ],
        "verified": True,
    }
    (BAG / "clock_audit.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
