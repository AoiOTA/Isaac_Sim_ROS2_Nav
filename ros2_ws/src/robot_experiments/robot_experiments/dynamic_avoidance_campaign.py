"""Read-only acceptance report for the dynamic avoidance benchmark."""
from __future__ import annotations

import argparse, csv, json, struct, zlib
from pathlib import Path


def _runs(root: Path) -> list[dict]:
    return [json.loads(path.read_text(encoding="utf-8")) for path in sorted(root.rglob("run_manifest.json"))]


def _valid(run: dict) -> bool:
    interaction = run.get("dynamic_interaction", {})
    return bool(interaction.get("complete")) and not bool(interaction.get("guard_aborted"))


def _html(payload: dict) -> str:
    rows = "".join(f"<tr><td>{r.get('run_index')}</td><td>{r.get('random_seed')}</td><td>{r.get('dynamic_selection',{}).get('case_id')}</td><td>{r.get('result')}</td><td>{r.get('failure_reason','')}</td></tr>" for r in payload["runs"])
    return f"<!doctype html><meta charset=utf-8><title>Dynamic avoidance acceptance</title><h1>Dynamic avoidance acceptance: {'PASS' if payload['passed'] else 'FAIL'}</h1><pre>{json.dumps(payload['gates'], indent=2)}</pre><table border=1><tr><th>run</th><th>seed</th><th>case</th><th>result</th><th>reason</th></tr>{rows}</table>"


def _write_overview_png(path: Path, success: int, total: int) -> None:
    """Dependency-free compact pass-rate strip for offline evidence bundles."""
    width, height = 400, 32; passed = 0 if total == 0 else round(width * success / total)
    rows = []
    for _ in range(height):
        row = bytearray([0])
        for x in range(width): row.extend((36, 190, 105) if x < passed else (220, 65, 65))
        rows.append(bytes(row))
    chunk = lambda kind, data: struct.pack(">I", len(data)) + kind + data + struct.pack(">I", zlib.crc32(kind + data) & 0xffffffff)
    path.write_bytes(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)) + chunk(b"IDAT", zlib.compress(b"".join(rows))) + chunk(b"IEND", b""))


def main(args=None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--controlled-directory", required=True)
    parser.add_argument("--full-route-directory", required=True)
    parser.add_argument("--output-directory", required=True)
    parsed = parser.parse_args(args)
    controlled, full = _runs(Path(parsed.controlled_directory)), _runs(Path(parsed.full_route_directory))
    cases = {name: [r for r in controlled if r.get("dynamic_selection", {}).get("case_id") == name] for name in ("crossing", "oncoming", "same_direction_slow", "temporary_block")}
    c_valid, c_success = sum(_valid(r) for r in controlled), sum(r.get("result") == "success" for r in controlled)
    f_valid, f_success = sum(_valid(r) for r in full), sum(r.get("result") == "success" for r in full)
    guard_or_contact = any(r.get("dynamic_interaction", {}).get("guard_aborted") or r.get("collision_detected") for r in controlled + full)
    gates = {"controlled_count_20": len(controlled) == 20, "controlled_valid_20": c_valid == 20, "controlled_success_18": c_success >= 18,
             "each_case_success_4": all(sum(r.get("result") == "success" for r in values) >= 4 for values in cases.values()),
             "full_count_5": len(full) == 5, "full_valid_5": f_valid == 5, "full_success_4": f_success >= 4, "no_contact_or_guard_abort": not guard_or_contact}
    payload = {"passed": all(gates.values()), "gates": gates, "controlled": {"valid": c_valid, "success": c_success}, "full_route": {"valid": f_valid, "success": f_success}, "runs": controlled + full}
    output = Path(parsed.output_directory)
    if output.exists(): raise SystemExit(f"refusing to overwrite {output}")
    output.mkdir(parents=True); (output / "summary.json").write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8"); (output / "index.html").write_text(_html(payload), encoding="utf-8")
    with (output / "runs.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=["run_index", "seed", "case_id", "variant_id", "result", "valid_interaction", "failure_reason"])
        writer.writeheader()
        for run in payload["runs"]:
            selection = run.get("dynamic_selection", {})
            writer.writerow({"run_index": run.get("run_index"), "seed": run.get("random_seed"), "case_id": selection.get("case_id"), "variant_id": selection.get("variant_id"), "result": run.get("result"), "valid_interaction": _valid(run), "failure_reason": run.get("failure_reason", "")})
    _write_overview_png(output / "success_overview.png", c_success + f_success, len(controlled) + len(full))
    raise SystemExit(0 if payload["passed"] else 2)
