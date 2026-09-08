#!/usr/bin/env python3
"""Plot reference and actual world-frame trajectories with start/end markers."""

import argparse
import csv
from pathlib import Path

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.ticker import MaxNLocator
from mpl_toolkits.mplot3d import proj3d


ROOT = Path(__file__).resolve().parent
FIELDS = (["t"] + [f"{prefix}{axis}" for prefix in
          ("p_", "ref_p_") for axis in "xyz"])


def read_log(path):
    with path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        missing = set(FIELDS) - set(reader.fieldnames or [])
        if missing:
            raise ValueError(f"Missing columns: {sorted(missing)}")
        rows = [[float(row[key]) for key in FIELDS] for row in reader]
    values = np.asarray(rows, dtype=float)
    if len(values) < 2 or not np.isfinite(values).all():
        raise ValueError("Need at least two rows with finite required fields.")
    data = dict(zip(FIELDS, values.T))
    if np.any(np.diff(data["t"]) < 0):
        raise ValueError("Time moves backwards; split separate runs first.")
    # Retain the last sample at duplicate timestamps.
    keep = np.r_[np.diff(data["t"]) > 0, True]
    print(f"Rows: {len(values)}; duplicate timestamps removed: {np.sum(~keep)}")
    return {key: value[keep] for key, value in data.items()}


def columns(data, prefix, axes="xyz"):
    return np.column_stack([data[prefix + axis] for axis in axes])


def plot(data, output, elev, azim):
    t = data["t"]
    if len(t) < 2:
        raise ValueError("Need at least two distinct timestamps.")
    p, ref = columns(data, "p_"), columns(data, "ref_p_")
    fig = plt.figure(figsize=(13, 10), facecolor="white")
    ax = fig.add_axes([0.05, 0.12, 0.90, 0.77], projection="3d")
    # Full polylines preserve dash continuity.
    ax.plot(*ref.T, color="#777777", linewidth=1.8, linestyle="--")
    ax.plot(*p.T, color="#1F5A99", linewidth=2.2, linestyle="-")
    endpoints = [("Start", p[0], "o", "#1F5A99"),
                 ("End", p[-1], "s", "#B35B00")]
    for label, position, marker, color in endpoints:
        ax.plot(*position[:, None], linestyle="none", marker=marker,
                ms=9, color=color, markeredgecolor="white", markeredgewidth=1,
                zorder=6)
    points = np.vstack((p, ref))
    low, high = points.min(axis=0), points.max(axis=0)
    span = np.maximum(high-low, 1.0)
    center = (high+low)/2
    for setter, mid, extent in zip((ax.set_xlim, ax.set_ylim, ax.set_zlim), center, span):
        setter(mid-extent*0.54, mid+extent*0.54)
    ax.set_box_aspect(span)  # Equal physical scaling of all three axes.
    ax.view_init(elev=elev, azim=azim)
    ax.set(xlabel="X (m)", ylabel="Y (m)", zlabel="Z (m)")
    for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
        axis.set_major_locator(MaxNLocator(nbins=5))
        axis.labelpad = 12
        axis.set_pane_color((0.97, 0.97, 0.97, 1))
        axis._axinfo["grid"]["color"] = (0.83, 0.83, 0.83, 0.5)
    fig.suptitle("Reference and actual 3D trajectory", fontsize=19, y=0.96)
    fig.text(0.5, 0.915, f"{t[0]:.2f}–{t[-1]:.2f} s  |  World coordinates", ha="center", fontsize=11)
    fig.legend(handles=[Line2D([], [], color="#1F5A99", lw=2.2, linestyle="-", label="Actual"),
                        Line2D([], [], color="#777777", lw=1.8, linestyle="--", label="Reference"),
                        Line2D([], [], color="#1F5A99", marker="o", linestyle="none", label="Start"),
                        Line2D([], [], color="#B35B00", marker="s", linestyle="none", label="End")],
               loc="lower center", bbox_to_anchor=(0.5, 0.067), ncol=4, frameon=False)
    fig.text(0.5, 0.04, "Start and end markers indicate the first and last actual positions in the selected time range.", ha="center", fontsize=9)
    fig.canvas.draw()
    for label, position, marker, color in endpoints:
        x, y, _ = proj3d.proj_transform(*position, ax.get_proj())
        ax.annotate(label, xy=(x, y), xytext=(12, 14),
                    textcoords="offset points", fontsize=11, color=color,
                    arrowprops=dict(arrowstyle="-", color=color, lw=0.8, shrinkB=5))
    output.parent.mkdir(parents=True, exist_ok=True)
    for extension in ("png", "pdf"):
        path = output.with_suffix("."+extension)
        fig.savefig(path, dpi=220)
        print(f"Saved {path}")
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", nargs="?", type=Path, default=ROOT.parent/"build/loop50.csv")
    parser.add_argument("--output", type=Path, help="Output base path without extension")
    parser.add_argument("--start", type=float, help="Start log time (s)")
    parser.add_argument("--end", type=float, help="End log time (s)")
    parser.add_argument("--elev", type=float, default=24)
    parser.add_argument("--azim", type=float, default=-58)
    args = parser.parse_args()
    data = read_log(args.csv)
    keep = np.ones(len(data["t"]), dtype=bool)
    if args.start is not None:
        keep &= data["t"] >= args.start
    if args.end is not None:
        keep &= data["t"] <= args.end
    data = {key: value[keep] for key, value in data.items()}
    plot(data, args.output or ROOT/"output"/(args.csv.stem+"_trajectory"),
         args.elev, args.azim)


if __name__ == "__main__":
    main()
