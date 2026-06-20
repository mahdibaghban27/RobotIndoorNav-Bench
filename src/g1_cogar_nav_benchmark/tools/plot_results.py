#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path
from typing import Optional

import matplotlib.pyplot as plt
import pandas as pd


def _column(df: pd.DataFrame, *names: str) -> Optional[str]:
    for name in names:
        if name in df.columns:
            return name
    return None


def _bar(df: pd.DataFrame, value_col: str, out: Path, title: str, ylabel: str, err_col: Optional[str] = None) -> None:
    pivot = df.pivot(index="scenario_id", columns="planner_id", values=value_col).sort_index()
    errors = None
    if err_col and err_col in df.columns:
        errors = df.pivot(index="scenario_id", columns="planner_id", values=err_col).reindex(index=pivot.index, columns=pivot.columns)
    ax = pivot.plot(kind="bar", yerr=errors, capsize=4 if errors is not None else 0)
    ax.set_title(title)
    ax.set_xlabel("Scenario")
    ax.set_ylabel(ylabel)
    ax.legend(title="Planner")
    ax.grid(axis="y", alpha=0.3)
    plt.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(out, dpi=180)
    plt.close()


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate benchmark plots from an aggregate summary CSV.")
    parser.add_argument("--summary", required=True)
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args()

    df = pd.read_csv(args.summary)
    out_dir = Path(args.output_dir)

    plots = [
        ("success_rate", None, "Navigation success rate", "Success rate", "success_rate.png"),
        (_column(df, "completion_time_s_mean", "elapsed_s_mean"), _column(df, "completion_time_s_std", "elapsed_s_std"), "Completion time", "Time [s]", "completion_time.png"),
        ("path_efficiency_mean", "path_efficiency_std", "Path efficiency", "Reference / executed path", "path_efficiency.png"),
        (_column(df, "near_collision_count_mean", "collision_count_mean"), _column(df, "near_collision_count_std", "collision_count_std"), "Near-collision events", "Count", "collision_count.png"),
        ("recovery_count_mean", "recovery_count_std", "Nav2 recovery count", "Count", "recovery_count.png"),
        ("min_clearance_m_mean", "min_clearance_m_std", "Minimum obstacle clearance", "Distance [m]", "min_clearance.png"),
        ("final_goal_error_m_mean", "final_goal_error_m_std", "Final goal error", "Distance [m]", "final_goal_error.png"),
    ]

    for value_col, err_col, title, ylabel, filename in plots:
        if value_col and value_col in df.columns:
            _bar(df, value_col, out_dir / filename, title, ylabel, err_col)

    print(f"Plots written to {out_dir}")


if __name__ == "__main__":
    main()
