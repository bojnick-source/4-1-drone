"""
Fragment 2.9 — Python Orchestrator: Closeout Runner Wrapper (Hardened)

Purpose
-------
Invoke the compiled closeout_cli to run closeout evaluation from Python/CI
without rewriting the logic. Handles options, timeouts, and summary printing.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Optional


@dataclass(frozen=True)
class RunResult:
  exit_code: int
  status: str
  out_path: Optional[str]
  stdout: str
  stderr: str
  out_json: Optional[Dict[str, Any]]


def _status_from_exit_code(code: int) -> str:
  if code == 0:
    return "GO"
  if code == 2:
    return "NO_GO"
  if code == 3:
    return "NEEDS_DATA"
  return "ERROR"


def _read_json_file(path: Path) -> Dict[str, Any]:
  with path.open("r", encoding="utf-8") as f:
    return json.load(f)


def _safe_str(x: Any, max_len: int = 500) -> str:
  s = str(x)
  return s if len(s) <= max_len else s[: max_len - 3] + "..."


def _summarize_report(report: Dict[str, Any]) -> str:
  gates = report.get("gates", {})
  issues = report.get("issues", [])

  mass_gate = gates.get("mass_gate", "Unknown")
  disk_gate = gates.get("disk_area_gate", "Unknown")
  power_gate = gates.get("power_gate", "Unknown")

  counts: Dict[str, int] = {}
  for it in issues if isinstance(issues, list) else []:
    k = it.get("kind", "Unknown") if isinstance(it, dict) else "Unknown"
    counts[k] = counts.get(k, 0) + 1

  top_lines = []
  if isinstance(issues, list):
    for it in issues[:5]:
      if not isinstance(it, dict):
        continue
      code = _safe_str(it.get("code", ""))
      msg = _safe_str(it.get("message", ""), max_len=120)
      if code or msg:
        top_lines.append(f"- {code}: {msg}".strip())
  top_txt = "\n".join(top_lines) if top_lines else "- (none)"

  counts_txt = ", ".join(f"{k}={v}" for k, v in sorted(counts.items())) if counts else "none"

  return (
    "Closeout Summary\n"
    f"- Gates: mass={mass_gate}, disk_area={disk_gate}, power={power_gate}\n"
    f"- Issue counts: {counts_txt}\n"
    f"- Top issues:\n{top_txt}\n"
  )


def run_closeout(
    bin_path: Path,
    in_path: Path,
    out_path: Optional[Path],
    pretty: bool = True,
    emit_null: bool = True,
    require_mass_breakdown: bool = True,
    timeout_s: int = 120,
) -> RunResult:
  if not bin_path.exists():
    raise FileNotFoundError(f"Binary not found: {bin_path}")

  cmd = [
    str(bin_path),
    "--in",
    str(in_path),
    "--out",
    str(out_path) if out_path else "-",
    "--pretty",
    "1" if pretty else "0",
    "--emit-null",
    "1" if emit_null else "0",
    "--require-mass-breakdown",
    "1" if require_mass_breakdown else "0",
  ]

  proc = subprocess.run(
    cmd,
    capture_output=True,
    text=True,
    timeout=timeout_s,
    check=False,
  )

  status = _status_from_exit_code(proc.returncode)
  report_json: Optional[Dict[str, Any]] = None

  if out_path:
    try:
      report_json = _read_json_file(out_path)
    except Exception:
      report_json = None
  else:
    try:
      report_json = json.loads(proc.stdout)
    except Exception:
      report_json = None

  return RunResult(
    exit_code=proc.returncode,
    status=status,
    out_path=str(out_path) if out_path else None,
    stdout=proc.stdout,
    stderr=proc.stderr,
    out_json=report_json,
  )


def main(argv: Optional[list[str]] = None) -> int:
  p = argparse.ArgumentParser(add_help=True)
  p.add_argument("--bin", required=True, help="Path to compiled closeout_cli binary")
  p.add_argument("--in", dest="inp", required=True, help="Input JSON path")
  p.add_argument("--out", default=None, help="Output JSON path (optional; default stdout)")
  p.add_argument("--pretty", action="store_true", help="Pretty JSON output (default)")
  p.add_argument("--compact", action="store_true", help="Compact JSON output")
  p.add_argument("--emit-null", action="store_true", help="Emit null for unset numeric fields (default)")
  p.add_argument("--omit-null", action="store_true", help="Omit unset numeric fields instead of null")
  p.add_argument("--no-mass-breakdown", action="store_true", help="Do not require mass breakdown gate")
  p.add_argument("--timeout", type=int, default=120, help="Timeout seconds (default 120)")

  args = p.parse_args(argv)

  bin_path = Path(args.bin)
  in_path = Path(args.inp)
  out_path = Path(args.out) if args.out else None

  pretty = not args.compact
  emit_null = not args.omit_null
  require_mass_breakdown = not args.no_mass_breakdown

  try:
    res = run_closeout(
      bin_path=bin_path,
      in_path=in_path,
      out_path=out_path,
      pretty=pretty,
      emit_null=emit_null,
      require_mass_breakdown=require_mass_breakdown,
      timeout_s=args.timeout,
    )
  except subprocess.TimeoutExpired:
    sys.stderr.write("ERROR: closeout_cli timed out\n")
    return 1
  except Exception as e:
    sys.stderr.write(f"ERROR: {_safe_str(e)}\n")
    return 1

  if res.out_json is not None:
    sys.stderr.write(_summarize_report(res.out_json))
  else:
    sys.stderr.write("Closeout Summary\n- (no JSON parsed)\n")

  if res.stderr.strip():
    sys.stderr.write("\n[closeout_cli stderr]\n")
    snippet = res.stderr[:2000]
    sys.stderr.write(snippet)
    if len(res.stderr) > len(snippet):
      sys.stderr.write("\n...")
    sys.stderr.write("\n")

  return res.exit_code


if __name__ == "__main__":
  raise SystemExit(main())
