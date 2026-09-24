#!/usr/bin/env python3
"""Read-only Blackbox rate-response analysis; never writes flight parameters.

Units/ACTUAL mapping verified against the log's Betaflight commit 79065c96b.
Positive lag means the measured output follows the input. Correlation peaks
are signal-dependent equivalent lags, not identified transport dead time.
"""

import argparse
import csv
import hashlib
import json
from pathlib import Path
import platform

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import scipy
from scipy import optimize, signal


AXES = ("Roll", "Pitch", "Yaw")


def correlation_lag(u, y, dt, max_lag=0.080):
    """Identical normalization support at every lag; sub-sample parabolic peak."""
    pad = int(round(max_lag / dt))
    center = u[pad:-pad]
    lags = np.arange(-pad, pad + 1)
    values = np.array([np.corrcoef(center, y[pad + k:len(y) - pad + k])[0, 1]
                       for k in lags])
    j = int(np.nanargmax(values))
    correction = 0.0
    if 0 < j < len(lags) - 1:
        curvature = values[j - 1] - 2 * values[j] + values[j + 1]
        if abs(curvature) > 1e-12:
            correction = np.clip(0.5 * (values[j - 1] - values[j + 1]) / curvature, -1, 1)
    return (lags[j] + correction) * dt * 1000, values[j]


def filter_first_order(u, tau, dt):
    if tau < 1e-7:
        return u.copy()
    a = np.exp(-dt / tau)
    # At sample n output has responded to held input from sample n-1.
    return signal.lfilter([0, 1 - a], [1, -a], u, zi=[u[0]])[0]


def fit_models(t, u, y, train, test, dt):
    """Fit input-only simulations on alternate 4 s blocks; score held-out blocks.

    Input history may precede a test block; no test gyro samples initialize the
    simulated response. This is within-flight validation, not another flight.
    """
    def response(delay, tau):
        delayed = np.interp(t - delay, t, u, left=u[0], right=u[-1])
        return filter_first_order(delayed, tau, dt)

    def scale(z):
        zx, yy = z[train], y[train]
        gain = np.dot(zx - zx.mean(), yy - yy.mean()) / np.sum((zx - zx.mean()) ** 2)
        gain = np.clip(gain, 0.5, 1.5)
        offset = yy.mean() - gain * zx.mean()
        return gain, offset

    def evaluate(delay, tau):
        z = response(delay, tau)
        gain, offset = scale(z)
        prediction = gain * z + offset
        loss = np.mean((prediction[train] - y[train]) ** 2)
        return loss, gain, offset, prediction

    output = {}
    predictions = {}
    for name in ("ideal", "static_gain", "pure_delay", "first_order", "first_order_delay"):
        delay, tau = 0.0, 0.0
        if name == "ideal":
            gain, offset, prediction = 1.0, 0.0, u.copy()
        elif name == "static_gain":
            _, gain, offset, prediction = evaluate(0, 0)
        else:
            if name == "pure_delay":
                result = optimize.minimize_scalar(lambda x: evaluate(x, 0)[0], bounds=(0, .060), method="bounded")
                delay = float(result.x)
            elif name == "first_order":
                result = optimize.minimize_scalar(lambda x: evaluate(0, x)[0], bounds=(1e-6, .080), method="bounded")
                tau = float(result.x)
            else:
                candidates = []
                for start in ((.005, .002), (.0, .015), (.015, .001), (.025, .020)):
                    result = optimize.minimize(lambda x: evaluate(x[0], x[1])[0], start,
                                               method="Nelder-Mead", bounds=((0, .060), (0, .080)),
                                               options={"xatol": 1e-6, "fatol": 1e-7, "maxiter": 160})
                    candidates.append(result)
                result = min(candidates, key=lambda r: r.fun)
                delay, tau = map(float, result.x)
            _, gain, offset, prediction = evaluate(delay, tau)
        output[name] = {
            "delay_ms": delay * 1000, "tau_ms": tau * 1000,
            "gain": float(gain), "offset_deg_s": float(offset),
            "train_rmse_deg_s": float(np.sqrt(np.mean((prediction[train] - y[train]) ** 2))),
            "test_rmse_deg_s": float(np.sqrt(np.mean((prediction[test] - y[test]) ** 2))),
        }
        predictions[name] = prediction
    return output, predictions


def run(source, out):
    out.mkdir(parents=True, exist_ok=True)
    metadata = {}
    with source.open() as stream:
        for row_number, row in enumerate(csv.reader(stream)):
            if row and row[0] == "loopIteration":
                header_line = row_number
                break
            if len(row) >= 2:
                metadata[row[0]] = row[1]
    data = pd.read_csv(source, skiprows=header_line)
    original_time = (data.time.to_numpy() - data.time.iloc[0]) * 1e-6
    original_dt = np.diff(original_time)
    keys = ["time", "loopIteration"] + [f"{key}[{a}]" for key in ("setpoint", "gyroADC", "rcCommand") for a in range(3)]
    assert not data[keys].isna().any().any()
    assert np.all(original_dt > 0)
    assert metadata["rates_type"] == "3", "ACTUAL reconstruction requires ACTUAL rate type"
    assert metadata.get("blackbox_high_resolution", "0") == "0", "High-resolution units need separate decoding"
    safe_metadata_names = ("Firmware revision", "Firmware date", "Log start datetime", "Craft name",
                           "looptime", "pid_process_denom", "rollPID", "pitchPID", "yawPID", "rc_rates", "rates",
                           "rc_expo", "deadband", "yaw_deadband", "rateAccelLimit", "yawRateAccelLimit",
                           "rc_smoothing_mode", "rc_smoothing_active_cutoffs_ff_sp_thr", "rc_smoothing_rx_smoothed")
    summary = {
        "source": str(source), "sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "rows": len(data), "columns": len(data.columns), "header_line_1_based": header_line + 1,
        "duration_seconds": float(original_time[-1]), "sample_rate_hz": float(1 / original_dt.mean()),
        "sample_interval_ms_min_median_max": (np.quantile(original_dt, [0, .5, 1]) * 1000).tolist(),
        "gaps_over_2ms": int(np.sum(original_dt > .002)), "duplicate_timestamps": int(np.sum(original_dt == 0)),
        "loop_increment_counts": {str(k): int(v) for k, v in data.loopIteration.diff().value_counts().items()},
        "key_column_missing": data[keys].isna().sum().to_dict(),
        "metadata": {k: metadata[k] for k in safe_metadata_names if k in metadata},
        "software": {"python": platform.python_version(), "numpy": np.__version__, "scipy": scipy.__version__,
                     "pandas": pd.__version__, "matplotlib": matplotlib.__version__},
        "notes": ["Lag quantiles describe windows, not statistical confidence intervals.",
                  "Source gyro/setpoint quantized to integer deg/s; sub-ms peak interpolation is not sub-ms accuracy.",
                  "Source time is flight-controller clock. No CM5/MSP transport or external estimator timing is present.",
                  "Uniform 1 ms interpolation for lag/frequency; 2 ms for model identification.",
                  "Closed-loop pilot data with feedforward: correlation and fitted lag are not motor time constants.",
                  "Inner-loop fitted tau below the 2 ms model grid is not a resolved physical parameter.",
                  "Recorded automatic RC smoothing and approximately 67 Hz RX may differ from future 100 Hz MSP operation.",
                  "Models use disjoint 4 s blocks from one flight; this is not independent-flight validation."],
    }
    summary["flight_mode_counts"] = {str(k): int(v) for k, v in data.flightModeFlags.value_counts().items()}
    summary["slow_field_missing"] = data[["flightModeFlags", "failsafePhase", "rxSignalReceived"]].isna().sum().to_dict()
    dt = .001
    time = np.arange(0, original_time[-1], dt)
    throttle = np.interp(time, original_time, data["setpoint[3]"])
    centers = np.fromstring(metadata["rc_rates"], sep=",") * 10
    maxima = np.fromstring(metadata["rates"], sep=",") * 10
    expos = np.fromstring(metadata["rc_expo"], sep=",") / 100
    selected = (original_time >= 3) & (original_time < 125)
    all_windows, spectra, model_rows = [], [], []
    axis_series = []
    summary["axes"] = {}
    for axis, name in enumerate(AXES):
        sp0 = data[f"setpoint[{axis}]"].to_numpy(float)
        gy0 = data[f"gyroADC[{axis}]"].to_numpy(float)
        deadband = float(metadata["yaw_deadband" if axis == 2 else "deadband"])
        stick = data[f"rcCommand[{axis}]"].to_numpy(float) / (500 - deadband)
        raw0 = centers[axis] * stick + max(0, maxima[axis] - centers[axis]) * np.abs(stick) * (
            stick ** 5 * expos[axis] + stick * (1 - expos[axis]))
        raw, sp, gy = [np.interp(time, original_time, v) for v in (raw0, sp0, gy0)]
        gy_unfilt = np.interp(time, original_time, data[f"gyroUnfilt[{axis}]"])
        axis_series.append((raw, sp, gy))
        active = (time >= 3) & (time < 125)
        err = gy0[selected] - sp0[selected]
        metrics = {
            "rmse_deg_s": float(np.sqrt(np.mean(err ** 2))),
            "absolute_error_p95_deg_s": float(np.quantile(abs(err), .95)),
            "absolute_error_p99_deg_s": float(np.quantile(abs(err), .99)),
            "setpoint_peak_abs_deg_s": float(abs(sp0[selected]).max()),
            "setpoint_rms_deg_s": float(np.sqrt(np.mean(sp0[selected] ** 2))),
            "derived_rcCommands_equals_setpoint": bool(np.array_equal(data[f"rcCommands[{axis}]"], sp0)),
            "axisError_equals_setpoint_minus_gyro": bool(np.array_equal(data[f"axisError[{axis}]"], sp0 - gy0)),
        }
        for stage, u, y in (("inner", sp, gy), ("smoothing", raw, sp), ("rc_to_gyro", raw, gy)):
            lag, corr = correlation_lag(u[active], y[active], dt)
            metrics[stage + "_whole_level_lag_ms"] = lag
            metrics[stage + "_whole_level_correlation"] = corr
        # Paired zero-phase filters do not insert a relative delay.
        level_filter = signal.butter(3, [.5, 20], btype="bandpass", fs=1 / dt, output="sos")
        derivative_filter = signal.butter(3, [1, 15], btype="bandpass", fs=1 / dt, output="sos")
        level = [signal.sosfiltfilt(level_filter, v) for v in (raw, sp, gy)]
        derivative = [np.gradient(signal.sosfiltfilt(derivative_filter, v), dt) for v in (raw, sp, gy)]
        train = np.zeros(len(time), dtype=bool)
        test = np.zeros(len(time), dtype=bool)
        for start in np.arange(3, 122, 1):
            mask = (time >= start) & (time < start + 4)
            if np.std(sp[mask]) < 8 or np.std(level[1][mask]) < 8 or np.mean(throttle[mask]) <= 100:
                continue
            for stage, first, second in (("inner", 1, 2), ("smoothing", 0, 1), ("rc_to_gyro", 0, 2)):
                for method, signals in (("bandpassed_level", level), ("bandpassed_derivative", derivative)):
                    lag, corr = correlation_lag(signals[first][mask], signals[second][mask], dt)
                    all_windows.append({"axis": name, "start_s": start, "end_s": start + 4, "stage": stage,
                                        "method": method, "lag_ms": lag, "correlation": corr,
                                        "input_std_deg_s": float(np.std(sp[mask]))})
        # Nonoverlapping whole blocks prevent train/test overlap.
        for block, start in enumerate(np.arange(3, 122, 4)):
            mask = (time >= start) & (time < start + 4)
            if np.std(sp[mask]) >= 8 and np.mean(throttle[mask]) > 100:
                (train if block % 2 == 0 else test)[mask] = True
        metrics["model_train_seconds"] = float(train.sum() * dt)
        metrics["model_test_seconds"] = float(test.sum() * dt)
        for stage, u in (("inner", sp), ("rc_to_gyro", raw)):
            fitted, _ = fit_models(time[::2], u[::2], gy[::2], train[::2], test[::2], dt * 2)
            metrics[stage + "_models"] = fitted
            for model, values in fitted.items():
                model_rows.append({"axis": name, "stage": stage, "model": model, **values})
            f, puu = signal.welch(u[active], fs=1 / dt, nperseg=4096)
            _, pyy = signal.welch(gy[active], fs=1 / dt, nperseg=4096)
            _, puy = signal.csd(u[active], gy[active], fs=1 / dt, nperseg=4096)
            coherence = np.abs(puy) ** 2 / (puu * pyy)
            transfer = puy / puu
            for j in np.flatnonzero((f >= .5) & (f <= 30)):
                spectra.append({"axis": name, "stage": stage, "frequency_hz": f[j], "input_psd": puu[j],
                                "gain": abs(transfer[j]), "phase_deg": np.angle(transfer[j], deg=True),
                                "coherence": coherence[j]})
        metrics["gyro_filter_level_lag_ms"] = correlation_lag(gy_unfilt[active], gy[active], dt, .02)[0]
        summary["axes"][name] = metrics
    windows = pd.DataFrame(all_windows)
    windows.to_csv(out / "window_lags.csv", index=False)
    pd.DataFrame(spectra).to_csv(out / "frequency_response.csv", index=False)
    pd.DataFrame(model_rows).to_csv(out / "model_validation.csv", index=False)
    for name in AXES:
        for stage in ("inner", "smoothing", "rc_to_gyro"):
            for method in ("bandpassed_level", "bandpassed_derivative"):
                sub = windows[(windows.axis == name) & (windows.stage == stage) & (windows.method == method) &
                              (windows.correlation >= .85)]
                summary["axes"][name][stage + "_" + method] = {
                    "windows": len(sub), "lag_ms_p10_median_p90": sub.lag_ms.quantile([.1, .5, .9]).tolist()}
    summary["pt3_metadata_low_frequency_group_delay_ms"] = 3 / (2 * np.pi * 15 * 1.961459177) * 1000
    (out / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n")

    plt.rcParams.update({"font.size": 10, "axes.grid": True, "grid.alpha": .22})
    fig, axes = plt.subplots(3, 2, figsize=(14, 9), constrained_layout=True)
    for a, name in enumerate(AXES):
        raw, sp, gy = axis_series[a]
        axes[a, 0].plot(time[::10], sp[::10], label="PID setpoint", lw=1, color="#2563eb")
        axes[a, 0].plot(time[::10], gy[::10], label="Filtered gyro", lw=.8, alpha=.8, color="#e07823")
        axes[a, 0].set(title=name + ": complete flight", ylabel="Angular rate (deg/s)", xlabel="Time since log start (s)")
        mask = (time >= 3) & (time < 125)
        # Select largest command change; label as a selected example, not average response.
        change = np.abs(np.gradient(signal.sosfiltfilt(signal.butter(3, 15, fs=1000, output="sos"), sp), dt))
        peak = np.flatnonzero(mask)[np.argmax(change[mask])]
        lo, hi = time[peak] - .25, time[peak] + .4
        cut = (time >= lo) & (time <= hi)
        for val, label, color in ((raw, "Reconstructed pre-smoothing rate", "#64748b"),
                                  (sp, "PID setpoint", "#2563eb"), (gy, "Filtered gyro", "#e07823")):
            axes[a, 1].plot(time[cut], val[cut], label=label, lw=1.4, color=color)
        axes[a, 1].set(title=name + ": selected rapid command change", ylabel="Angular rate (deg/s)", xlabel="Time (s)")
    axes[0, 0].legend(loc="upper left", fontsize=8)
    axes[0, 1].legend(loc="best", fontsize=8)
    fig.suptitle("Measured rate tracking: pre-smoothing RC rate → PID setpoint → gyro", fontsize=15)
    fig.savefig(out / "rate_tracking.png", dpi=170)
    plt.close(fig)
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5), constrained_layout=True)
    for index, stage in enumerate(("inner", "rc_to_gyro")):
        vals = [windows[(windows.axis == a) & (windows.stage == stage) &
                        (windows.method == "bandpassed_derivative") & (windows.correlation >= .85)].lag_ms.to_numpy()
                for a in AXES]
        axes[index].boxplot(vals, labels=AXES, whis=(10, 90), showfliers=False)
        axes[index].set(ylabel="Equivalent lag (ms)", title=("PID setpoint → gyro" if index == 0 else "Pre-smoothing RC rate → gyro"))
        axes[index].axhline(10, color="#64748b", ls="--", lw=1, label="MPC command period: 10 ms")
        axes[index].legend(fontsize=8)
    fig.suptitle("Excited 4 s windows; derivative correlation ≥ 0.85; whiskers are P10/P90, not confidence limits")
    fig.savefig(out / "lag_distribution.png", dpi=170)
    plt.close(fig)
    print(json.dumps({a: {k: v for k, v in summary["axes"][a].items() if k in (
        "rmse_deg_s", "absolute_error_p95_deg_s", "inner_bandpassed_derivative", "rc_to_gyro_bandpassed_derivative",
        "inner_models", "rc_to_gyro_models")} for a in AXES}, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parent)
    args = parser.parse_args()
    run(args.source, args.output)
