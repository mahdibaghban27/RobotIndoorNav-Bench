#!/usr/bin/env python3
from __future__ import annotations

import argparse

from g1_cogar_nav_benchmark.metrics import aggregate_summary


def main() -> None:
    parser = argparse.ArgumentParser(description="Aggregate multiple per-run summary CSV files.")
    parser.add_argument("--inputs", nargs="+", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--raw-output", default=None)
    args = parser.parse_args()
    aggregate_summary(args.inputs, args.output, args.raw_output)
    print(f"Wrote {args.output}")
    if args.raw_output:
        print(f"Wrote {args.raw_output}")


if __name__ == "__main__":
    main()
