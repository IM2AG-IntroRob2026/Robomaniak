#!/usr/bin/env python3
from __future__ import annotations

import json
from datetime import datetime, timezone
from pathlib import Path

TEST_RESULTS = Path("public/test-results")


def main() -> int:
    runs: list[tuple[int, dict]] = []
    for d in TEST_RESULTS.glob("run-*"):
        if not d.is_dir():
            continue
        try:
            num = int(d.name.removeprefix("run-"))
        except ValueError:
            continue
        meta_file = d / "meta.json"
        meta = json.loads(meta_file.read_text()) if meta_file.exists() else {}
        runs.append((num, meta))

    runs.sort(key=lambda x: x[0], reverse=True)

    items: list[str] = []
    for num, meta in runs:
        ts = meta.get("timestamp")
        ts_str = (
            datetime.fromtimestamp(ts, tz=timezone.utc).strftime("%Y-%m-%d %H:%M UTC")
            if ts else "--"
        )
        sha = (meta.get("sha") or "")[:7]
        actor = meta.get("actor", "")
        ref = meta.get("ref", "")
        items.append(
            f'<li>'
            f'<a href="./run-{num}/index.html">Run #{num}</a>'
            f'<div class="meta">{ts_str} · {ref} · <code>{sha}</code> · {actor}</div>'
            f'</li>'
        )

    html = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<title>Robomaniak - Test History</title>
<link rel="stylesheet" href="../style.css">
<style>
  .run-list {{ list-style: none; padding: 0; }}
  .run-list li {{ padding: 0.8rem 1rem; margin: 0.4rem 0;
                  background: #f5f5f5; border-radius: 6px; }}
  .meta {{ font-size: 0.85rem; color: #666; margin-top: 0.25rem; }}
  code {{ background: #e5e5e5; padding: 0 0.25rem; border-radius: 3px; }}
</style>
</head>
<body>
<div class="container">
<header><h1>Test Runs</h1></header>
<ul class="run-list">
{chr(10).join(items) if items else "<li>No runs available yet.</li>"}
</ul>
<p><a href="../index.html">← Back to hub</a></p>
</div>
</body>
</html>
"""
    (TEST_RESULTS / "index.html").write_text(html, encoding="utf-8")
    print(f"Generated index with {len(runs)} runs")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())