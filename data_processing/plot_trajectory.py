#!/usr/bin/env python3
"""Plot world-frame trajectories and actual-minus-reference position errors."""

import argparse
import csv
from pathlib import Path

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Patch
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
        fields = FIELDS + (["in_trajectory"] if "in_trajectory" in reader.fieldnames else [])
        rows = [[float(row[key]) for key in fields] for row in reader]
    values = np.asarray(rows, dtype=float)
    if len(values) < 2 or not np.isfinite(values).all():
        raise ValueError("Need at least two rows with finite required fields.")
    data = dict(zip(fields, values.T))
    if "in_trajectory" in data and not np.isin(data["in_trajectory"], [0, 1]).all():
        raise ValueError("in_trajectory must contain only 0 or 1.")
    if np.any(np.diff(data["t"]) < 0):
        raise ValueError("Time moves backwards; split separate runs first.")
    # Retain the last sample at duplicate timestamps.
    keep = np.r_[np.diff(data["t"]) > 0, True]
    print(f"Rows: {len(values)}; duplicate timestamps removed: {np.sum(~keep)}")
    return {key: value[keep] for key, value in data.items()}


def columns(data, prefix, axes="xyz"):
    return np.column_stack([data[prefix + axis] for axis in axes])


def trajectory_intervals(data):
    """Return sampled execution intervals, excluding hover outside the log flag."""
    if "in_trajectory" not in data:
        print("No in_trajectory column; trajectory shading omitted.")
        return []
    active = data["in_trajectory"].astype(bool)
    boundaries = np.flatnonzero(np.diff(np.r_[False, active, False]))
    return [(data["t"][start], data["t"][stop - 1])
            for start, stop in boundaries.reshape(-1, 2)]


@plt.rc_context({"font.family": "serif", "font.serif": ["DejaVu Serif"],
                 "font.size": 11, "mathtext.fontset": "dejavuserif",
                 "axes.labelsize": 12, "axes.titlesize": 13,
                 "axes.linewidth": 0.8, "pdf.fonttype": 42})
def plot(data, output, elev, azim):
    t = data["t"]
    if len(t) < 2:
        raise ValueError("Need at least two distinct timestamps.")
    p, ref = columns(data, "p_"), columns(data, "ref_p_")
    error = p - ref  # Signed world-frame tracking error, in metres.
    intervals = trajectory_intervals(data)
    fig = plt.figure(figsize=(16, 9), facecolor="white")
    layout = fig.add_gridspec(3, 2, width_ratios=(1.35, 1),
                             left=0.04, right=0.97, bottom=0.17, top=0.85,
                             wspace=0.24, hspace=0.14)
    ax = fig.add_subplot(layout[:, 0], projection="3d")
    ax.set_title("(a) 3D trajectory", pad=16)
    # Shared symmetric limits allow direct comparison across all three axes.
    error_limit = max(float(np.max(np.abs(error))) * 1.1, 0.01)
    error_axes = []
    for index, axis_name in enumerate("xyz"):
        error_ax = fig.add_subplot(layout[index, 1],
                                   sharex=error_axes[0] if error_axes else None,
                                   sharey=error_axes[0] if error_axes else None)
        for start, end in intervals:
            error_ax.axvspan(start, end, color="#1F5A99", alpha=0.08,
                            linewidth=0, zorder=0)
            for boundary in (start, end):
                error_ax.axvline(boundary, color="#777777", lw=0.9,
                                 linestyle=":", zorder=1)
            if index == 0:
                for boundary, label, align, offset in (
                        (start, "Trajectory start", "left", 4),
                        (end, "Trajectory end", "right", -4)):
                    error_ax.annotate(label, xy=(boundary, 0.96),
                                      xycoords=("data", "axes fraction"),
                                      xytext=(offset, 0), textcoords="offset points",
                                      ha=align, va="top", fontsize=9, color="#555555")
        error_ax.axhline(0, color="#777777", lw=0.9, linestyle="--")
        error_ax.plot(t, error[:, index], color="#1F5A99", lw=1.5)
        error_ax.set_ylabel(rf"$e_{axis_name}$ (m)")
        error_ax.set_ylim(-error_limit, error_limit)
        error_ax.set_xlim(t[0], t[-1])
        error_ax.yaxis.set_major_locator(MaxNLocator(nbins=5))
        error_ax.xaxis.set_major_locator(MaxNLocator(nbins=6))
        error_ax.set_axisbelow(True)
        error_ax.grid(True, color="#D4D4D4", lw=0.6, alpha=0.65)
        error_ax.spines[["top", "right"]].set_visible(False)
        error_ax.tick_params(direction="in", labelbottom=index == 2)
        error_axes.append(error_ax)
    error_axes[0].set_title("(b) Position error (actual − reference)", pad=16)
    error_axes[-1].set_xlabel("Time (s)")
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
    fig.suptitle("Trajectory tracking and position error", fontsize=19, y=0.96)
    fig.text(0.5, 0.915, f"{t[0]:.2f}–{t[-1]:.2f} s  |  World coordinates", ha="center", fontsize=11)
    fig.legend(handles=[Line2D([], [], color="#1F5A99", lw=2.2, linestyle="-", label="Actual"),
                        Line2D([], [], color="#777777", lw=1.8, linestyle="--", label="Reference"),
                        Line2D([], [], color="#1F5A99", marker="o", linestyle="none", label="Start"),
                        Line2D([], [], color="#B35B00", marker="s", linestyle="none", label="End")]
               + ([Patch(facecolor="#1F5A99", alpha=0.08,
                         label="Trajectory execution (excluding hover)")] if intervals else []),
               loc="lower center", bbox_to_anchor=(0.5, 0.067), ncol=5, frameon=False)
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
    parser.add_argument("csv", nargs="?", type=Path, default=ROOT.parent/"build/helix20_35.csv")
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
