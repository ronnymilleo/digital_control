#!/usr/bin/env python3
import argparse
import csv
import os
from typing import List, Tuple

import matplotlib.pyplot as plt


def read_csv(path: str) -> Tuple[List[float], List[float], List[float], List[float]]:
    t: List[float] = []
    r: List[float] = []
    y: List[float] = []
    u: List[float] = []
    with open(path, "r", newline="") as f:
        reader = csv.DictReader(f)
        expected = {"t", "r", "y", "u"}
        if set(reader.fieldnames or []) != expected:
            raise ValueError(f"CSV must have columns exactly: {sorted(expected)}; got {reader.fieldnames}")
        for row in reader:
            t.append(float(row["t"]))
            r.append(float(row["r"]))
            y.append(float(row["y"]))
            u.append(float(row["u"]))
    return t, r, y, u


def plot(t: List[float], r: List[float], y: List[float], u: List[float], title: str):
    fig, ax1 = plt.subplots(figsize=(10, 6))

    ax1.plot(t, r, label="setpoint r", color="tab:gray", linestyle="--", linewidth=1.5)
    ax1.plot(t, y, label="output y", color="tab:blue", linewidth=2.0)
    ax1.set_xlabel("Time [s]")
    ax1.set_ylabel("Output")
    ax1.grid(True, which="both", alpha=0.3)

    ax2 = ax1.twinx()
    ax2.plot(t, u, label="control u", color="tab:orange", linewidth=1.5, alpha=0.8)
    ax2.set_ylabel("Control")

    # Combine legends from both axes
    lines, labels = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines + lines2, labels + labels2, loc="best")

    fig.suptitle(title)
    fig.tight_layout()
    return fig


def main():
    parser = argparse.ArgumentParser(description="Plot PID simulation CSV (t,r,y,u)")
    parser.add_argument(
        "--csv",
        default="sim_pid_first_order.csv",
        help="Path to CSV file produced by sim_pid_first_order (default: %(default)s)",
    )
    parser.add_argument(
        "--out",
        default=None,
        help="Optional output image path (e.g., plot.png). If omitted, only shows the plot.",
    )
    parser.add_argument(
        "--title",
        default="PID step response (first-order plant)",
        help="Plot title",
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="Do not display the interactive window (useful in CI)",
    )

    args = parser.parse_args()

    if not os.path.isfile(args.csv):
        raise FileNotFoundError(f"CSV not found: {args.csv}. Run the simulator first.")

    t, r, y, u = read_csv(args.csv)
    fig = plot(t, r, y, u, args.title)

    if args.out:
        fig.savefig(args.out, dpi=150)
        print(f"Saved plot to {args.out}")

    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()

