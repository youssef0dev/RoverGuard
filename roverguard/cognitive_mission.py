"""High-level orchestration helpers for AI-generated missions."""

from __future__ import annotations

import json
from datetime import datetime, timezone
from pathlib import Path

from .ai.schema import MissionPlan


def save_mission_audit(
    logs_directory: Path,
    *,
    user_command: str,
    model: str,
    plan: MissionPlan,
) -> Path:
    """Persist the operator request and approved plan for traceability."""
    logs_directory.mkdir(parents=True, exist_ok=True)
    now = datetime.now(timezone.utc)
    path = logs_directory / f"ai_mission_{now:%Y%m%d_%H%M%S}.json"
    payload = {
        "schema_version": 1,
        "created_at_utc": now.isoformat(),
        "provider": "ollama-local",
        "model": model,
        "operator_command": user_command,
        "validation_status": "APPROVED",
        "mission": plan.to_dict(),
    }
    path.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )
    return path
