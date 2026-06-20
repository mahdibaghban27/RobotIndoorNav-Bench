#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path
from typing import Optional

import matplotlib.pyplot as plt
import pandas as pd


LATEX_TABLE_TEMPLATE = r"""
\begin{table}[H]
\centering
\caption{Benchmark summary across planners and scenarios.}
\label{tab:benchmark_summary}
\small
\begin{tabular}{llrrrrrr}
\toprule
Scenario & Planner & Runs & Success & Time [s] & Efficiency & Near-coll. & Recoveries \\
\midrule
%s
\bottomrule
\end{tabular}
\end{table}
"""


def latex_escape(s: str) -> str:
    return str(s).replace('&', r'\&').replace('%', r'\%').replace('_', r'\_')


def col(df: pd.DataFrame, *names: str) -> Optional[str]:
    for name in names:
        if name in df.columns:
            return name
    return None


def main() -> None:
    parser = argparse.ArgumentParser(description='Generate LaTeX table and plots from benchmark summary CSV.')
    parser.add_argument('--summary-csv', required=True)
    parser.add_argument('--out-dir', required=True)
    args = parser.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    df = pd.read_csv(args.summary_csv)

    time_col = col(df, 'completion_time_s_mean', 'elapsed_s_mean')
    collision_col = col(df, 'near_collision_count_mean', 'collision_count_mean')
    recovery_col = col(df, 'recovery_count_mean')

    rows = []
    for _, row in df.iterrows():
        rows.append(
            f"{latex_escape(row['scenario_id'])} & {latex_escape(row['planner_id'])} & "
            f"{int(row['runs'])} & {row['success_rate']:.2f} & "
            f"{row[time_col]:.2f} & {row['path_efficiency_mean']:.2f} & "
            f"{row[collision_col]:.2f} & {row[recovery_col]:.2f} \\\\" if time_col and collision_col and recovery_col else ""
        )
    rows = [r for r in rows if r]
    (out_dir / 'benchmark_table.tex').write_text(LATEX_TABLE_TEMPLATE % '\n'.join(rows), encoding='utf-8')

    plot_items = [
        ('success_rate', 'success_rate.png', 'Success rate'),
        (time_col, 'completion_time.png', 'Completion time [s]'),
        ('path_efficiency_mean', 'path_efficiency.png', 'Path efficiency'),
        (collision_col, 'collision_count.png', 'Mean near-collision count'),
        (recovery_col, 'recovery_count.png', 'Mean recovery count'),
    ]
    for metric, filename, ylabel in plot_items:
        if not metric or metric not in df.columns:
            continue
        pivot = df.pivot(index='scenario_id', columns='planner_id', values=metric)
        ax = pivot.plot(kind='bar', rot=20, figsize=(8, 4.8))
        ax.set_ylabel(ylabel)
        ax.set_xlabel('Scenario')
        ax.grid(True, axis='y', alpha=0.25)
        ax.figure.tight_layout()
        ax.figure.savefig(out_dir / filename, dpi=180)
        plt.close(ax.figure)

    print(f'Wrote report assets to {out_dir}')


if __name__ == '__main__':
    main()
