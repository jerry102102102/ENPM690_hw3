from __future__ import annotations

import json

from .constants import FishState, GameSnapshot


def fish_states_to_json(fish_states: list[FishState]) -> str:
    return json.dumps([fish.to_dict() for fish in fish_states], separators=(",", ":"))


def fish_states_from_json(payload: str) -> list[dict]:
    if not payload:
        return []
    return json.loads(payload)


def game_snapshot_to_json(snapshot: GameSnapshot) -> str:
    return json.dumps(snapshot.to_dict(), separators=(",", ":"))


def game_snapshot_from_json(payload: str) -> dict:
    if not payload:
        return {}
    return json.loads(payload)
