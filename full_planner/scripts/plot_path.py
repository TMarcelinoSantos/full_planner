#!/usr/bin/env python3
"""Plot the cone map and centerline path written by full_planner_node
(cones.csv and path.csv, see PlannerNode::writeConesCsv/writePathCsv).

The path is drawn twice, side by side, as two separate heat maps rather than
one chart trying to carry both quantities at once: curvature (diverging
blue/red, centered on zero - blue for a left turn, red for a right turn) and
target speed (sequential green, light = slow, dark = fast). Tight corners
jump out on the left, and where the speed profile actually slows down for
them is checkable at a glance on the right.

Usage:
    python3 plot_path.py --cones cones.csv --path path.csv --out path.png
"""
import argparse
import csv

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection
from matplotlib.colors import LinearSegmentedColormap, Normalize

# Must match the class_type constants in lart_msgs/msg/Cone.msg.
CONE_COLORS = {
    0: ("black", "unknown"),
    1: ("gold", "yellow"),
    2: ("royalblue", "blue"),
    3: ("darkorange", "orange (small)"),
    4: ("darkorange", "orange (big)"),
}

# Diverging pair (blue <-> red) with a neutral gray midpoint, per the
# project's data-viz palette: polarity encoding (which way the path turns),
# not identity, so this is deliberately not a rainbow and not reused from
# the cone/categorical colors above.
CURVATURE_CMAP = LinearSegmentedColormap.from_list(
    "curvature_diverging", ["#2a78d6", "#f0efec", "#e34948"]
)

# Sequential single-hue ramp (light = slow -> dark = fast) for speed. Green,
# not blue, deliberately: blue is already carrying two meanings on this
# figure (one curvature pole, and the "blue cone" marker color) - a third
# blue channel for speed would collide with both.
SPEED_CMAP = LinearSegmentedColormap.from_list(
    "speed_sequential", ["#e3f2e3", "#4fa34f", "#008300"]
)

START_COLOR = "#008300"  # categorical green - landmark, not a curvature/speed value
END_COLOR = "#4a3aa7"    # categorical violet - landmark, not a curvature/speed value


def read_cones(path):
    with open(path, newline="") as f:
        return [
            (float(row["x"]), float(row["y"]), int(row["class_type"]))
            for row in csv.DictReader(f)
        ]


def read_path(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    xs = [float(r["x"]) for r in rows]
    ys = [float(r["y"]) for r in rows]
    curvature = [float(r["curvature"]) for r in rows]
    velocity = [float(r["velocity"]) for r in rows]
    return xs, ys, curvature, velocity


def draw_cones(ax, cones, with_legend_labels):
    seen_labels = set()
    for x, y, class_type in cones:
        color, label = CONE_COLORS.get(class_type, ("gray", f"class {class_type}"))
        show_label = with_legend_labels and label not in seen_labels
        ax.scatter(
            x, y, c=color, s=40, edgecolors="black", linewidths=0.5, zorder=2,
            label=label if show_label else None,
        )
        seen_labels.add(label)


def draw_path_heatmap(ax, xs, ys, values, cmap, norm, start_end_labels):
    if len(xs) < 2:
        return None

    points = np.array([xs, ys]).T.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)
    segment_values = [(values[i] + values[i + 1]) / 2.0 for i in range(len(values) - 1)]

    path_line = LineCollection(segments, cmap=cmap, norm=norm, zorder=3)
    path_line.set_array(np.array(segment_values))
    path_line.set_linewidth(3)
    ax.add_collection(path_line)

    ax.scatter(
        xs[0], ys[0], c=START_COLOR, s=120, marker="*", edgecolors="black", zorder=4,
        label="path start" if start_end_labels else None,
    )
    ax.scatter(
        xs[-1], ys[-1], c=END_COLOR, s=120, marker="X", edgecolors="black", zorder=4,
        label="path end" if start_end_labels else None,
    )
    return path_line


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cones", required=True, help="cones.csv written by full_planner_node")
    parser.add_argument("--path", required=True, help="path.csv written by full_planner_node")
    parser.add_argument("--out", default="path.png", help="Output image file")
    parser.add_argument("--show", action="store_true", help="Also open an interactive window")
    args = parser.parse_args()

    cones = read_cones(args.cones)
    xs, ys, curvature, velocity = read_path(args.path)

    fig, (ax_curvature, ax_speed) = plt.subplots(1, 2, figsize=(20, 8), sharex=True, sharey=True)

    draw_cones(ax_curvature, cones, with_legend_labels=True)
    draw_cones(ax_speed, cones, with_legend_labels=False)

    max_abs_curvature = max((abs(c) for c in curvature), default=0.0) or 1.0
    curvature_norm = Normalize(vmin=-max_abs_curvature, vmax=max_abs_curvature)
    curvature_line = draw_path_heatmap(
        ax_curvature, xs, ys, curvature, CURVATURE_CMAP, curvature_norm, start_end_labels=True,
    )

    min_speed = min(velocity, default=0.0)
    max_speed = max(velocity, default=0.0)
    if max_speed <= min_speed:
        max_speed = min_speed + 1.0
    speed_norm = Normalize(vmin=min_speed, vmax=max_speed)
    speed_line = draw_path_heatmap(
        ax_speed, xs, ys, velocity, SPEED_CMAP, speed_norm, start_end_labels=False,
    )

    all_x = xs + [c[0] for c in cones]
    all_y = ys + [c[1] for c in cones]
    if all_x:
        margin = 2.0
        ax_curvature.set_xlim(min(all_x) - margin, max(all_x) + margin)
        ax_curvature.set_ylim(min(all_y) - margin, max(all_y) + margin)

    for ax, title in (
        (ax_curvature, f"curvature ({len(xs)} points, {len(cones)} cones)"),
        (ax_speed, "target speed"),
    ):
        ax.set_aspect("equal", adjustable="box")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_title(title)
        ax.grid(True, alpha=0.3)

    ax_curvature.legend(loc="best", fontsize=8)

    if curvature_line is not None:
        cbar = fig.colorbar(curvature_line, ax=ax_curvature, pad=0.02)
        cbar.set_label("curvature [1/m]  (+ left turn / - right turn)")
    if speed_line is not None:
        cbar = fig.colorbar(speed_line, ax=ax_speed, pad=0.02)
        cbar.set_label("target speed [m/s]")

    fig.suptitle("full_planner midline path")
    fig.tight_layout()
    fig.savefig(args.out, dpi=150)
    print(f"Saved plot to {args.out}")

    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
