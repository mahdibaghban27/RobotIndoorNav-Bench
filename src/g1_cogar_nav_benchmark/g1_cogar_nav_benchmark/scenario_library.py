"""Scenario loading helpers."""
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Tuple, Optional
import yaml


@dataclass
class Scenario:
    scenario_id: str
    world: str
    map_yaml: str
    start_pose: Tuple[float, float, float]
    goal_pose: Tuple[float, float, float]
    start_area: str
    goal_area: str
    description: str
    dynamic_obstacles: bool = False
    dynamic_obstacle_cmd: str = "dynamic_obstacle_commander.py"
    use_topological_route: bool = False
    notes: str = ""
    topological_graph: Optional[str] = None


def _resolve(base_dir: Path, value: str) -> str:
    p = Path(value)
    if p.is_absolute():
        return str(p)
    return str((base_dir / p).resolve())


def load_scenarios(path: str) -> Dict[str, Scenario]:
    config_path = Path(path).resolve()
    base_dir = config_path.parent

    raw = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    scenarios: Dict[str, Scenario] = {}

    for sid, cfg in raw["scenarios"].items():
        graph = cfg.get("topological_graph")
        if graph is not None:
            graph = _resolve(base_dir, graph)

        scenarios[sid] = Scenario(
            scenario_id=sid,
            world=_resolve(base_dir, cfg["world"]),
            map_yaml=_resolve(base_dir, cfg["map_yaml"]),
            start_pose=tuple(cfg["start_pose"]),
            goal_pose=tuple(cfg["goal_pose"]),
            start_area=cfg["start_area"],
            goal_area=cfg["goal_area"],
            description=cfg["description"],
            dynamic_obstacles=bool(cfg.get("dynamic_obstacles", False)),
            dynamic_obstacle_cmd=cfg.get("dynamic_obstacle_cmd", "dynamic_obstacle_commander.py"),
            use_topological_route=bool(cfg.get("use_topological_route", False)),
            notes=cfg.get("notes", ""),
            topological_graph=graph,
        )

    return scenarios