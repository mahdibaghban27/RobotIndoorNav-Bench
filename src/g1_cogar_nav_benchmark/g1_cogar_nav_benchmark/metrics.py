"""Offline metric computation for benchmark runs."""
from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple
import math

import pandas as pd
import yaml


@dataclass
class BenchmarkMetrics:
    run_id: str
    scenario_id: str
    planner: str
    planner_id: str
    success: int
    final_status: str
    failure_reason: str
    elapsed_s: float
    completion_time_s: float
    path_length_m: float
    executed_path_length_m: float
    straight_line_m: float
    reference_path_length_m: float
    path_efficiency: float
    collision_count: int
    near_collision_count: int
    recovery_count: int
    timeout: int
    recovery_success: int
    reflex_events: int
    reflex_time_s: float
    stopped_time_s: float
    min_clearance_m: float
    mean_clearance_m: float
    final_goal_error_m: float
    mean_speed_mps: float
    num_goals: int
    goals_reached: int

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


def _pairwise_distance(xs: Iterable[float], ys: Iterable[float]) -> float:
    xs = list(xs)
    ys = list(ys)
    if len(xs) < 2:
        return 0.0
    total = 0.0
    for i in range(1, len(xs)):
        total += math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1])
    return total


def _safe_float(value: Any, default: float = 0.0) -> float:
    try:
        if value is None:
            return default
        return float(value)
    except (TypeError, ValueError):
        return default


def _safe_int(value: Any, default: int = 0) -> int:
    try:
        if value is None:
            return default
        return int(value)
    except (TypeError, ValueError):
        return default


def _load_yaml(path: Optional[str]) -> Dict[str, Any]:
    if not path:
        return {}
    p = Path(path)
    if not p.exists():
        return {}
    raw = yaml.safe_load(p.read_text(encoding="utf-8"))
    return raw if isinstance(raw, dict) else {}


def _resolve(base: Path, value: str) -> Path:
    p = Path(value)
    return p if p.is_absolute() else (base / p).resolve()


def _shortest_route(edges: Dict[str, List[str]], start: str, goal: str) -> List[str]:
    queue: List[List[str]] = [[start]]
    seen = {start}
    while queue:
        route = queue.pop(0)
        node = route[-1]
        if node == goal:
            return route
        for nxt in edges.get(node, []):
            if nxt not in seen:
                seen.add(nxt)
                queue.append(route + [nxt])
    return []


def _route_length(graph_path: Path, start_area: str, goal_area: str) -> Optional[float]:
    raw = _load_yaml(str(graph_path))
    areas = raw.get("areas", {})
    edges = raw.get("edges", {})
    if start_area not in areas or goal_area not in areas:
        return None
    route = _shortest_route(edges, start_area, goal_area)
    if len(route) < 2:
        return None
    total = 0.0
    for a, b in zip(route[:-1], route[1:]):
        pa = areas[a].get("pose", [])
        pb = areas[b].get("pose", [])
        if len(pa) < 2 or len(pb) < 2:
            return None
        total += math.hypot(float(pb[0]) - float(pa[0]), float(pb[1]) - float(pa[1]))
    return total


def reference_path_length(scenario_file: Optional[str], scenario_id: str, fallback: float) -> float:
    if not scenario_file:
        return fallback
    config_path = Path(scenario_file).resolve()
    raw = _load_yaml(str(config_path))
    cfg = raw.get("scenarios", {}).get(scenario_id)
    if not isinstance(cfg, dict):
        return fallback
    graph = cfg.get("topological_graph")
    if not graph:
        return fallback
    graph_path = _resolve(config_path.parent, graph)
    value = _route_length(graph_path, str(cfg.get("start_area", "")), str(cfg.get("goal_area", "")))
    return value if value and value > 0.0 else fallback


def compute_metrics(
    log_csv: str,
    scenario_id: str,
    planner_id: str,
    run_id: str,
    goal_xy: List[float],
    success: bool,
    scenario_file: Optional[str] = None,
    metadata_yaml: Optional[str] = None,
) -> BenchmarkMetrics:
    df = pd.read_csv(log_csv)
    if df.empty:
        raise ValueError(f"Log file {log_csv} is empty")

    metadata = _load_yaml(metadata_yaml)
    elapsed_s = float(df["t"].iloc[-1] - df["t"].iloc[0])
    path_length = _pairwise_distance(df["x"].tolist(), df["y"].tolist())
    straight_line = math.hypot(df["x"].iloc[0] - goal_xy[0], df["y"].iloc[0] - goal_xy[1])
    final_goal_error = math.hypot(df["x"].iloc[-1] - goal_xy[0], df["y"].iloc[-1] - goal_xy[1])
    reference_length = reference_path_length(scenario_file, scenario_id, straight_line)

    min_scan = pd.to_numeric(df.get("min_scan"), errors="coerce").replace([math.inf, -math.inf], pd.NA)
    collision_distance = _safe_float(df.get("collision_distance", pd.Series([0.18])).iloc[0], 0.18)
    near_collision = (min_scan < collision_distance).fillna(False)
    near_collision_edges = (near_collision & ~near_collision.shift(1, fill_value=False)).sum()

    reflex_active = pd.to_numeric(df.get("reflex_active", pd.Series([0.0] * len(df))), errors="coerce").fillna(0.0) > 0.5
    reflex_edges = (reflex_active & ~reflex_active.shift(1, fill_value=False)).sum()
    positive_dt = pd.to_numeric(df.get("dt", pd.Series([])), errors="coerce")
    positive_dt = positive_dt[positive_dt > 0.0]
    dt = float(positive_dt.median()) if not positive_dt.empty else (elapsed_s / max(len(df) - 1, 1))

    linear_x = pd.to_numeric(df.get("linear_x", pd.Series([0.0] * len(df))), errors="coerce").fillna(0.0)
    clearance = min_scan.dropna().clip(lower=0.0)
    min_clearance = float(clearance.min()) if not clearance.empty else 999.0
    mean_clearance = float(clearance.mean()) if not clearance.empty else 999.0
    recovery_count = _safe_int(metadata.get("recovery_count"), 0)
    recovery_success = _safe_int(metadata.get("recovery_success"), 1 if success else 0)
    failure_reason = str(metadata.get("failure_reason", "none" if success else "navigation_failed"))
    timeout = 1 if failure_reason in {"timeout", "goal_timeout", "navigation_timeout"} else 0
    final_status = "succeeded" if success else ("timeout" if timeout else failure_reason)

    efficiency = 0.0 if path_length <= 1e-6 else min(1.0, reference_length / path_length)

    return BenchmarkMetrics(
        run_id=run_id,
        scenario_id=scenario_id,
        planner=planner_id,
        planner_id=planner_id,
        success=1 if success else 0,
        final_status=final_status,
        failure_reason=failure_reason,
        elapsed_s=elapsed_s,
        completion_time_s=elapsed_s,
        path_length_m=path_length,
        executed_path_length_m=path_length,
        straight_line_m=straight_line,
        reference_path_length_m=reference_length,
        path_efficiency=efficiency,
        collision_count=int(near_collision_edges),
        near_collision_count=int(near_collision_edges),
        recovery_count=recovery_count,
        timeout=timeout,
        recovery_success=recovery_success,
        reflex_events=int(reflex_edges),
        reflex_time_s=float(reflex_active.sum() * dt),
        stopped_time_s=float((linear_x.abs() < 0.02).sum() * dt),
        min_clearance_m=min_clearance,
        mean_clearance_m=mean_clearance,
        final_goal_error_m=final_goal_error,
        mean_speed_mps=float(linear_x.abs().mean()),
        num_goals=_safe_int(metadata.get("num_goals"), 1),
        goals_reached=_safe_int(metadata.get("goals_reached"), 1 if success else 0),
    )


def aggregate_summary(input_csvs: List[str], output_csv: str, raw_output_csv: Optional[str] = None) -> None:
    rows = [pd.read_csv(csv_path) for csv_path in input_csvs]
    if not rows:
        raise ValueError("No summary csvs were provided.")
    full = pd.concat(rows, ignore_index=True)
    full = full.sort_values(["scenario_id", "planner_id", "run_id"])

    if raw_output_csv:
        Path(raw_output_csv).parent.mkdir(parents=True, exist_ok=True)
        full.to_csv(raw_output_csv, index=False)

    grouped = full.groupby(["scenario_id", "planner_id"], as_index=False)
    agg_spec = {
        "runs": ("run_id", "count"),
        "success_rate": ("success", "mean"),
    }
    for col, prefix in [
        ("completion_time_s", "completion_time_s"),
        ("elapsed_s", "elapsed_s"),
        ("path_length_m", "path_length_m"),
        ("executed_path_length_m", "executed_path_length_m"),
        ("reference_path_length_m", "reference_path_length_m"),
        ("path_efficiency", "path_efficiency"),
        ("collision_count", "collision_count"),
        ("near_collision_count", "near_collision_count"),
        ("recovery_count", "recovery_count"),
        ("timeout", "timeout"),
        ("recovery_success", "recovery_success"),
        ("reflex_events", "reflex_events"),
        ("reflex_time_s", "reflex_time_s"),
        ("stopped_time_s", "stopped_time_s"),
        ("min_clearance_m", "min_clearance_m"),
        ("mean_clearance_m", "mean_clearance_m"),
        ("final_goal_error_m", "final_goal_error_m"),
        ("mean_speed_mps", "mean_speed_mps"),
        ("goals_reached", "goals_reached"),
    ]:
        if col in full.columns:
            agg_spec[f"{prefix}_mean"] = (col, "mean")
            agg_spec[f"{prefix}_std"] = (col, "std")

    summary = grouped.agg(**agg_spec).fillna(0.0)
    summary = summary.sort_values(["scenario_id", "planner_id"])
    Path(output_csv).parent.mkdir(parents=True, exist_ok=True)
    summary.to_csv(output_csv, index=False)
