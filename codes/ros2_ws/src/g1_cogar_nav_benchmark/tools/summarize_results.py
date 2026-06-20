#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path
import pandas as pd

from g1_cogar_nav_benchmark.metrics import compute_metrics


def main() -> None:
    parser = argparse.ArgumentParser(description="Summarize one benchmark run from logger CSV.")
    parser.add_argument("--log-csv", required=True)
    parser.add_argument("--scenario-id", required=True)
    parser.add_argument("--planner-id", required=True)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--goal-x", type=float, required=True)
    parser.add_argument("--goal-y", type=float, required=True)
    parser.add_argument("--success", type=int, choices=[0, 1], required=True)
    parser.add_argument("--scenario-file", default=None)
    parser.add_argument("--metadata-yaml", default=None)
    parser.add_argument("--output-csv", required=True)
    args = parser.parse_args()

    metrics = compute_metrics(
        log_csv=args.log_csv,
        scenario_id=args.scenario_id,
        planner_id=args.planner_id,
        run_id=args.run_id,
        goal_xy=[args.goal_x, args.goal_y],
        success=bool(args.success),
        scenario_file=args.scenario_file,
        metadata_yaml=args.metadata_yaml,
    )

    df = pd.DataFrame([metrics.to_dict()])
    Path(args.output_csv).parent.mkdir(parents=True, exist_ok=True)
    df.to_csv(args.output_csv, index=False)
    print(f"Wrote {args.output_csv}")


if __name__ == "__main__":
    main()
