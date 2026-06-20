"""Planner profile utilities for the G1 navigation benchmark."""
from dataclasses import dataclass
from typing import Dict


@dataclass(frozen=True)
class PlannerProfile:
    profile_id: str
    params_file: str
    controller_id: str
    notes: str
    built_in: bool = True


DEFAULT_PROFILES: Dict[str, PlannerProfile] = {
    'dwb': PlannerProfile(
        profile_id='dwb',
        params_file='config/nav2_dwb.yaml',
        controller_id='FollowPathDWB',
        notes='Baseline Nav2 Dynamic Window Based controller for structured indoor navigation.',
        built_in=True,
    ),
    'mppi': PlannerProfile(
        profile_id='mppi',
        params_file='config/nav2_mppi.yaml',
        controller_id='FollowPathMPPI',
        notes='Core Nav2 MPPI controller for more predictive local obstacle avoidance.',
        built_in=True,
    ),
    'rpp': PlannerProfile(
        profile_id='rpp',
        params_file='config/nav2_rpp.yaml',
        controller_id='FollowPathRPP',
        notes='Nav2 Regulated Pure Pursuit controller tuned for indoor structured navigation.',
        built_in=True,
    ),
    'teb': PlannerProfile(
        profile_id='teb',
        params_file='config/nav2_teb.yaml',
        controller_id='FollowPathTEB',
        notes='External TEB Nav2 profile. Requires teb_plugin_class to name the installed Humble-compatible TEB controller plugin.',
        built_in=False,
    ),
}
