#!/usr/bin/env python3
"""Plot the cone map and centerline path written by full_planner_node
(cones.csv and path.csv, see PlannerNode::writeConesCsv/writePathCsv).

The path is drawn as a curvature heat map: a diverging blue/red colormap
centered on zero, blue for a left turn, red for a right turn, gray for
straight - so tight corners jump out at a glance.

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

START_COLOR = "#008300"  # categorical green - landmark, not a curvature value
END_COLOR = "#4a3aa7"    # categorical violet - landmark, not a curvature value


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
    return xs, ys, curvature


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cones", required=True, help="cones.csv written by full_planner_node")
    parser.add_argument("--path", required=True, help="path.csv written by full_planner_node")
    parser.add_argument("--out", default="path.png", help="Output image file")
    parser.add_argument("--show", action="store_true", help="Also open an interactive window")
    args = parser.parse_args()

    cones = read_cones(args.cones)
    xs, ys, curvature = read_path(args.path)

    fig, ax = plt.subplots(figsize=(12, 8))

    seen_labels = set()
    for x, y, class_type in cones:
        color, label = CONE_COLORS.get(class_type, ("gray", f"class {class_type}"))
        ax.scatter(
            x, y, c=color, s=40, edgecolors="black", linewidths=0.5, zorder=2,
            label=label if label not in seen_labels else None,
        )
        seen_labels.add(label)

    path_line = None
    if len(xs) > 1:
        points = np.array([xs, ys]).T.reshape(-1, 1, 2)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)
        segment_curvature = [(curvature[i] + curvature[i + 1]) / 2.0 for i in range(len(curvature) - 1)]

        max_abs_curvature = max((abs(c) for c in curvature), default=0.0) or 1.0
        norm = Normalize(vmin=-max_abs_curvature, vmax=max_abs_curvature)

        path_line = LineCollection(segments, cmap=CURVATURE_CMAP, norm=norm, zorder=3)
        path_line.set_array(np.array(segment_curvature))
        path_line.set_linewidth(3)
        ax.add_collection(path_line)

        ax.scatter(
            xs[0], ys[0], c=START_COLOR, s=120, marker="*", edgecolors="black", zorder=4, label="path start",
        )
        ax.scatter(
            xs[-1], ys[-1], c=END_COLOR, s=120, marker="X", edgecolors="black", zorder=4, label="path end",
        )

    all_x = xs + [c[0] for c in cones]
    all_y = ys + [c[1] for c in cones]
    if all_x:
        margin = 2.0
        ax.set_xlim(min(all_x) - margin, max(all_x) + margin)
        ax.set_ylim(min(all_y) - margin, max(all_y) + margin)

    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(f"full_planner midline path curvature ({len(xs)} points, {len(cones)} cones)")
    ax.legend(loc="best", fontsize=8)
    ax.grid(True, alpha=0.3)

    if path_line is not None:
        cbar = fig.colorbar(path_line, ax=ax, pad=0.02)
        cbar.set_label("curvature [1/m]  (+ left turn / - right turn)")

    fig.tight_layout()
    fig.savefig(args.out, dpi=150)
    print(f"Saved plot to {args.out}")

    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
