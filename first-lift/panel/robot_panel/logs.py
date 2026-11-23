"""Transaction and guided-test logs."""

from __future__ import annotations

import json
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

from .command_codes import COMMAND_NAMES


@dataclass(frozen=True)
class TraceEntry:
    timestamp: str
    action: str
    command: str
    command_code: str
    tx: str
    rx: str
    result: str
    latency_ms: float
    detail: str


class TraceLog:
    """In-memory transaction history mirrored to an append-only JSONL file."""

    def __init__(self, path: Path, limit: int = 500):
        self.path = path
        self.entries: deque[TraceEntry] = deque(maxlen=limit)
        self.file = path.open("a", encoding="utf-8", buffering=1)

    def add(
        self,
        *,
        action: str,
        command: int,
        tx: bytes,
        rx: bytes,
        result: str,
        latency_ms: float,
        detail: str = "",
    ) -> None:
        entry = TraceEntry(
            timestamp=datetime.now().astimezone().isoformat(timespec="milliseconds"),
            action=action,
            command=COMMAND_NAMES.get(command, f"UNKNOWN_{command:02X}"),
            command_code=f"0x{command:02X}",
            tx=tx.hex(" "),
            rx=rx.hex(" "),
            result=result,
            latency_ms=round(latency_ms, 2),
            detail=detail,
        )
        self.entries.appendleft(entry)
        self.file.write(json.dumps(entry.__dict__, ensure_ascii=False) + "\n")

    def close(self) -> None:
        self.file.close()


class TestResultLog:
    """Append-only journal for guided checks and the operator's verdicts."""

    def __init__(self, path: Path):
        self.path = path
        self.file = path.open("a", encoding="utf-8", buffering=1)

    def add(
        self,
        *,
        test_name: str,
        values: dict[str, object],
        command_result: str,
        verdict: str,
        note: str,
    ) -> None:
        record = {
            "timestamp": datetime.now().astimezone().isoformat(timespec="milliseconds"),
            "record_type": "guided_test",
            "test": test_name,
            "values": values,
            "command_result": command_result,
            "operator_verdict": verdict,
            "note": note,
        }
        self.file.write(json.dumps(record, ensure_ascii=False) + "\n")

    def close(self) -> None:
        self.file.close()
