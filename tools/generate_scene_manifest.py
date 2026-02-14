#!/usr/bin/env python3
"""Generate assets/scenes/manifest.json for browser FS mounting."""

from __future__ import annotations

import argparse
import json
import pathlib


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenes-root", default="assets/scenes")
    parser.add_argument("--output", default="assets/scenes/manifest.json")
    args = parser.parse_args()

    root = pathlib.Path(args.scenes_root).resolve()
    out = pathlib.Path(args.output).resolve()

    files: list[str] = []
    for path in sorted(root.rglob("*")):
        if not path.is_file():
            continue
        if path.name == pathlib.Path(args.output).name and path.resolve() == out:
            continue
        files.append(path.relative_to(root).as_posix())

    payload = {"files": files}
    out.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(f"Wrote {len(files)} entries to {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
