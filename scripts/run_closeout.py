#!/usr/bin/env python3
"""
================================================================================
Fragment 4.1 — Python Wrapper: Run Closeout Pipeline (CI Integration)
FILE: scripts/run_closeout.py

Purpose:
  - Python wrapper for running closeout_demo from CI/CD pipelines
  - Parse exit codes and generate CI-friendly output
  - Generate artifacts (JSON, CSV) with deterministic paths
  - Support batch runs and result aggregation

Usage:
  python scripts/run_closeout.py [options]
  
Options:
  --output-dir DIR    Output directory for artifacts (default: ./artifacts)
  --json-path PATH    Path to JSON output (default: closeout.json)
  --csv-path PATH     Path to CSV output (default: closeout.csv)
  --verbose           Enable verbose output
  
Exit codes:
  0 - Success (Go decision)
  1 - Script error
  2 - NoGo decision
  3 - NeedsData decision
  10 - I/O error

================================================================================
"""

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path
from typing import Dict, Any, Optional


def parse_args():
    parser = argparse.ArgumentParser(
        description='Run DARPA LIFT closeout pipeline'
    )
    parser.add_argument(
        '--output-dir',
        type=str,
        default='./artifacts',
        help='Output directory for artifacts'
    )
    parser.add_argument(
        '--json-path',
        type=str,
        default='closeout.json',
        help='Path to JSON output'
    )
    parser.add_argument(
        '--verbose',
        action='store_true',
        help='Enable verbose output'
    )
    parser.add_argument(
        '--closeout-demo-bin',
        type=str,
        default='./build/closeout_demo',
        help='Path to closeout_demo binary'
    )
    return parser.parse_args()


def ensure_dir(path: Path) -> None:
    """Create directory if it doesn't exist."""
    path.mkdir(parents=True, exist_ok=True)


def run_closeout_demo(bin_path: str, json_path: str, verbose: bool = False) -> int:
    """
    Run closeout_demo binary.
    
    Returns:
        Exit code from closeout_demo
    """
    cmd = [bin_path, json_path]
    
    if verbose:
        print(f"Running: {' '.join(cmd)}")
    
    try:
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            check=False
        )
        
        if verbose or result.returncode != 0:
            print("STDOUT:")
            print(result.stdout)
            if result.stderr:
                print("STDERR:")
                print(result.stderr)
        
        return result.returncode
        
    except FileNotFoundError:
        print(f"ERROR: Binary not found: {bin_path}", file=sys.stderr)
        return 1
    except Exception as e:
        print(f"ERROR: Failed to run closeout_demo: {e}", file=sys.stderr)
        return 1


def parse_closeout_json(json_path: str, verbose: bool = False) -> Optional[Dict[str, Any]]:
    """
    Parse closeout JSON file.
    
    Returns:
        Parsed JSON dict, or None on error
    """
    try:
        with open(json_path, 'r') as f:
            data = json.load(f)
        
        if verbose:
            print(f"Loaded JSON from: {json_path}")
            print(f"Variant: {data.get('variant_name', 'unknown')}")
            gate_result = data.get('gate_result', {})
            print(f"Decision: {gate_result.get('decision', 'unknown')}")
            print(f"Failed gates: {len(gate_result.get('failed_gates', []))}")
            print(f"Missing data: {len(gate_result.get('missing_data', []))}")
        
        return data
        
    except FileNotFoundError:
        print(f"ERROR: JSON file not found: {json_path}", file=sys.stderr)
        return None
    except json.JSONDecodeError as e:
        print(f"ERROR: Invalid JSON in {json_path}: {e}", file=sys.stderr)
        return None
    except Exception as e:
        print(f"ERROR: Failed to parse JSON: {e}", file=sys.stderr)
        return None


def generate_summary(data: Dict[str, Any]) -> str:
    """Generate human-readable summary."""
    gate_result = data.get('gate_result', {})
    mass_delta = data.get('mass_delta', {})
    
    lines = [
        "=" * 60,
        "CLOSEOUT SUMMARY",
        "=" * 60,
        f"Variant: {data.get('variant_name', 'unknown')}",
        f"Concept: {data.get('variant_concept', 'unknown')}",
        "",
        f"Gate Decision: {gate_result.get('decision', 'unknown')}",
        f"Failed gates: {len(gate_result.get('failed_gates', []))}",
        f"Missing data: {len(gate_result.get('missing_data', []))}",
        "",
        f"Delta mass: {mass_delta.get('delta_mass_total_kg', 'N/A')} kg",
        f"Resulting mass: {mass_delta.get('resulting_aircraft_mass_kg', 'N/A')} kg",
        f"Payload ratio: {mass_delta.get('resulting_payload_ratio', 'N/A')}",
        "=" * 60,
    ]
    
    return "\n".join(lines)


def main() -> int:
    args = parse_args()
    
    # Ensure output directory exists
    output_dir = Path(args.output_dir)
    ensure_dir(output_dir)
    
    # Determine output path
    if os.path.isabs(args.json_path):
        json_path = args.json_path
    else:
        json_path = str(output_dir / args.json_path)
    
    # Run closeout_demo
    exit_code = run_closeout_demo(args.closeout_demo_bin, json_path, args.verbose)
    
    if exit_code != 0:
        print(f"closeout_demo exited with code {exit_code}", file=sys.stderr)
    
    # Parse and display results
    if os.path.exists(json_path):
        data = parse_closeout_json(json_path, args.verbose)
        if data:
            print(generate_summary(data))
            
            # Copy to output directory if not already there
            final_path = output_dir / 'closeout.json'
            if str(final_path) != json_path:
                import shutil
                shutil.copy2(json_path, final_path)
                if args.verbose:
                    print(f"Copied JSON to: {final_path}")
    
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
