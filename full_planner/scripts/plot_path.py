#!/usr/bin/env python3
"""Plot the cone map and centerline path written by full_planner_node
(cones.csv and path.csv, see PlannerNode::writeConesCsv/writePathCsv).

Usage:
    python3 plot_path.py --cones cones.csv --path path.csv --out path.png
"""
import argparse
import csv

import matplotlib.pyplot as plt

# Must match the class_type constants in lart_msgs/msg/Cone.msg.
CONE_COLORS = {
    0: ("black", "unknown"),
    1: ("gold", "yellow"),
    2: ("royalblue", "blue"),
    3: ("darkorange", "orange (small)"),
    4: ("darkorange", "orange (big)"),
}


def read_cones(path):
    with open(path, newline="") as f:
        return [
            (float(row["x"]), float(row["y"]), int(row["class_type"]))
            for row in csv.DictReader(f)
        ]


def read_path(path):
    with open(path, newline="") as f:
        return [(float(row["x"]), float(row["y"])) for row in csv.DictReader(f)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cones", required=True, help="cones.csv written by full_planner_node")
    parser.add_argument("--path", required=True, help="path.csv written by full_planner_node")
    parser.add_argument("--out", default="path.png", help="Output image file")
    parser.add_argument("--show", action="store_true", help="Also open an interactive window")
    args = parser.parse_args()

    cones = read_cones(args.cones)
    path = read_path(args.path)

    fig, ax = plt.subplots(figsize=(12, 8))

    seen_labels = set()
    for x, y, class_type in cones:
        color, label = CONE_COLORS.get(class_type, ("gray", f"class {class_type}"))
        ax.scatter(
            x, y, c=color, s=40, edgecolors="black", linewidths=0.5,
            label=label if label not in seen_labels else None,
        )
        seen_labels.add(label)

    if path:
        xs = [p[0] for p in path]
        ys = [p[1] for p in path]
        ax.plot(xs, ys, "-", color="limegreen", linewidth=2, label="midline path", zorder=3)
        ax.scatter(xs[0], ys[0], c="lime", s=120, marker="*", edgecolors="black", zorder=4, label="path start")
        ax.scatter(xs[-1], ys[-1], c="red", s=120, marker="X", edgecolors="black", zorder=4, label="path end")

    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(f"full_planner midline path ({len(path)} points, {len(cones)} cones)")
    ax.legend(loc="best", fontsize=8)
    ax.grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(args.out, dpi=150)
    print(f"Saved plot to {args.out}")

    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
