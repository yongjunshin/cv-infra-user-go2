#!/usr/bin/env python3
"""Judge evidence emitted by verify/run; no simulator or ROS dependency."""

from __future__ import annotations

import json
import os
import re
from pathlib import Path


def main() -> int:
    out = Path(os.environ["OUT"])
    mission = (out / "mission.txt").read_text(encoding="utf-8")
    found = bool(re.search(r"found:\s*true", mission, re.IGNORECASE))
    succeeded = bool(re.search(r"Goal finished with status: SUCCEEDED", mission))
    print(json.dumps({"action_succeeded": succeeded, "target_found": found, "mission_output": mission[-500:]}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
