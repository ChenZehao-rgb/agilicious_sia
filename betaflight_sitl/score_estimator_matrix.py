#!/usr/bin/env python3
"""Score completed estimator ablation logs (requires numpy and pandas)."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import pandas as pd


def score(path):
    frame = pd.read_csv(path)
    trajectory = frame.loc[frame.in_trajectory == 1].copy()
    if trajectory.empty:
        raise ValueError(f"{path}: no trajectory samples")

    def difference(a, b):
        return np.linalg.norm(
            trajectory[[a + axis for axis in "xyz"]].to_numpy()
            - trajectory[[b + axis for axis in "xyz"]].to_numpy(), axis=1
        )

    error = difference("p_", "ref_p_")
    trajectory["position_squared_error"] = error ** 2
    unique = trajectory.drop_duplicates("t", keep="last")
    dt = np.diff(unique.t.to_numpy())
    if len(dt) == 0 or np.any(dt <= 0):
        raise ValueError(f"{path}: insufficient or nonmonotonic timestamps")
    weighted = np.sqrt(np.dot(unique.position_squared_error.iloc[:-1], dt) / dt.sum())
    tilt = trajectory.ahrs_tilt_err_deg.dropna()
    return {
        "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "rows": len(trajectory),
        "log_end_s": float(frame.t.iloc[-1]),
        "trajectory_start_s": float(trajectory.t.iloc[0]),
        "trajectory_end_s": float(trajectory.t.iloc[-1]),
        "rmse_m": float(np.sqrt(np.mean(error ** 2))),
        "time_weighted_rmse_m": float(weighted),
        "max_position_error_m": float(error.max()),
        "position_estimate_rmse_m": float(np.sqrt(np.mean(difference("p_", "est_p_") ** 2))),
        "velocity_estimate_rmse_mps": float(np.sqrt(np.mean(difference("v_", "est_v_") ** 2))),
        "ahrs_tilt_mean_deg": float(tilt.mean()) if len(tilt) else None,
        "ahrs_tilt_max_deg": float(tilt.max()) if len(tilt) else None,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    args = parser.parse_args()
    results = {path.stem: score(path) for path in sorted(args.directory.glob("*.csv"))}
    if not results:
        parser.error("no CSV logs in the supplied directory")
    (args.directory / "scores.json").write_text(json.dumps(results, indent=2) + "\n")
    for name, result in results.items():
        print(f"{name}: RMSE={result['rmse_m']:.4f} m, "
              f"time-weighted={result['time_weighted_rmse_m']:.4f} m, "
              f"max={result['max_position_error_m']:.4f} m")


if __name__ == "__main__":
    main()
