"""Extract MSP battery voltage from the original MCAP without modifying it.

Dependencies: mcap, mcap-ros2-support, numpy, matplotlib.
Run from any directory. Outputs are written beside this script.
"""

import csv
import json
from collections import Counter
from datetime import datetime, timedelta, timezone
from pathlib import Path

import matplotlib
import numpy as np
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory

matplotlib.use("Agg")
import matplotlib.pyplot as plt


OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[2]
BAG = ROOT / "bags/hardware_20260928_142552_820897"
TZ = timezone(timedelta(hours=8))


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def local_time(ns):
    return datetime.fromtimestamp(ns / 1e9, TZ).isoformat(timespec="milliseconds")


def write_csv(name, rows):
    with (OUT / name).open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main():
    rows = []
    decoded = []
    configs = Counter()
    battery_events = Counter()
    with next(BAG.glob("*.mcap")).open("rb") as stream:
        reader = make_reader(stream, decoder_factories=[DecoderFactory()])
        statistics = reader.get_summary().statistics
        start_ns = statistics.message_start_time
        expected_count = sum(
            statistics.channel_message_counts.get(channel.id, 0)
            for channel in reader.get_summary().channels.values()
            if channel.topic == "/msp/battery"
        )
        topics = ["/msp/battery", "/msp/decoded_state", "/msp/config", "/msp/events"]
        for _, channel, message, data in reader.iter_decoded_messages(topics=topics):
            if channel.topic == "/msp/battery":
                payload = bytes(data.payload)
                assert data.event == "rx" and data.code == 130
                assert len(payload) >= 11 and payload[0] > 0
                voltage = int.from_bytes(payload[9:11], "little") / 100
                assert voltage > 0
                receive_ns = stamp_ns(data.header.stamp)
                rows.append({
                    "receive_time_cst": local_time(receive_ns),
                    "receive_time_ns": receive_ns,
                    "elapsed_bag_s": (receive_ns - start_ns) / 1e9,
                    "bag_log_time_ns": message.log_time,
                    "steady_time_s": data.steady_time,
                    "request_time_ns": stamp_ns(data.request_stamp),
                    "request_steady_time_s": data.request_steady_time,
                    "voltage_v": voltage,
                    "cell_count": payload[0],
                    "latency_s": data.latency_seconds,
                    "payload_hex": payload.hex(),
                })
            elif channel.topic == "/msp/decoded_state":
                decoded.append((stamp_ns(data.battery_stamp), data.battery_voltage))
            elif channel.topic == "/msp/config":
                configs[data.data] += 1
            elif data.code == 130:
                battery_events[data.event] += 1

    assert len(rows) == expected_count
    t = np.array([row["elapsed_bag_s"] for row in rows])
    voltage = np.array([row["voltage_v"] for row in rows])
    steady = np.array([row["steady_time_s"] for row in rows])
    assert np.all(np.diff(t) > 0) and np.all(np.diff(steady) > 0)
    by_request = {row["request_time_ns"]: row["voltage_v"] for row in rows}
    # MspState request stamps are reconstructed via floating-point ROS seconds.
    # Match within 1 microsecond to tolerate that sub-microsecond conversion.
    request_ns = np.array(sorted(by_request), dtype=np.int64)
    matched = mismatched = unmatched = invalid = 0
    for stamp, value in decoded:
        if not np.isfinite(value):
            invalid += 1
            continue
        idx = int(np.searchsorted(request_ns, stamp))
        candidates = [i for i in (idx - 1, idx) if 0 <= i < len(request_ns)]
        nearest = min(candidates, key=lambda i: abs(int(request_ns[i]) - stamp))
        if abs(int(request_ns[nearest]) - stamp) > 1000:
            unmatched += 1
        elif abs(value - by_request[int(request_ns[nearest])]) > 1e-6:
            mismatched += 1
        else:
            matched += 1

    minute_rows = []
    for lower in range(0, int(t[-1]) + 1, 60):
        selected = voltage[(t >= lower) & (t < lower + 60)]
        if selected.size:
            minute_rows.append({
                "elapsed_start_s": lower,
                "elapsed_end_s": min(lower + 60, (statistics.message_end_time - start_ns) / 1e9),
                "samples": int(selected.size),
                "mean_v": float(selected.mean()),
                "min_v": float(selected.min()),
                "max_v": float(selected.max()),
            })

    summary = {
        "source": str(BAG.relative_to(ROOT)),
        "topic": "/msp/battery",
        "voltage_definition": "MSP code 130 rx payload offset 9, uint16 little-endian / 100 V",
        "time_definition": "ROS receive header; elapsed from MCAP first message; UTC+08:00",
        "bag_duration_s": (statistics.message_end_time - start_ns) / 1e9,
        "samples": len(rows),
        "first": rows[0],
        "last": rows[-1],
        "minimum": rows[int(np.argmin(voltage))],
        "maximum": rows[int(np.argmax(voltage))],
        "mean_v": float(voltage.mean()),
        "change_v": float(voltage[-1] - voltage[0]),
        "change_percent_of_start": float((voltage[-1] / voltage[0] - 1) * 100),
        "max_adjacent_step_v": float(np.max(np.abs(np.diff(voltage)))),
        "cell_counts": sorted(set(row["cell_count"] for row in rows)),
        "receive_intervals_steady_s": dict(zip(
            ["min", "median", "max"], np.quantile(np.diff(steady), [0, 0.5, 1]).tolist()
        )),
        "max_ros_vs_steady_elapsed_difference_s": float(np.max(np.abs((t - t[0]) - (steady - steady[0])))),
        "battery_events": dict(battery_events),
        "decoded_state_validation": {
            "total": len(decoded), "matched": matched, "mismatched": mismatched,
            "unmatched": unmatched, "nonfinite": invalid,
            "stamp_matching_tolerance_ns": 1000,
        },
        "minute_statistics": minute_rows,
        "config_snapshots": [{"count": count, "text": text} for text, count in configs.items()],
    }
    assert mismatched == 0
    write_csv("voltage_samples.csv", rows)
    write_csv("voltage_by_minute.csv", minute_rows)
    (OUT / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")

    plt.rcParams.update({"font.size": 11, "axes.spines.top": False, "axes.spines.right": False})
    fig, ax = plt.subplots(figsize=(11, 4.8), layout="constrained")
    ax.plot(t, voltage, color="#2563a6", linewidth=1.5)
    ax.set(title="MSP battery voltage | 2026-09-28", xlabel="Time since bag start (s)",
           ylabel="Battery pack voltage (V)", xlim=(0, 365), ylim=(21.50, 22.10))
    ax.set_xticks([0, 60, 120, 180, 240, 300, 361.6])
    ax.grid(axis="both", alpha=0.18)
    ax.annotate("Start: 22.05 V", (t[0], voltage[0]), (18, 22.073),
                arrowprops={"arrowstyle": "-", "color": "#555555"})
    ax.annotate("End: 21.55 V\nMin: 21.54 V", (t[-1], voltage[-1]), (270, 21.53),
                arrowprops={"arrowstyle": "-", "color": "#555555"})
    fig.text(0.51, -0.04, "723 received battery frames | approximately 2 Hz | UTC+08:00 14:25:54.617–14:31:55.611",
             ha="center", fontsize=9, color="#555555")
    fig.savefig(OUT / "voltage_curve.png", dpi=180, bbox_inches="tight")
    plt.close(fig)
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
