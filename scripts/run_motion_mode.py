#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
from typing import Iterable


REPO_ROOT = Path(__file__).resolve().parents[1]
MODES_ROOT = REPO_ROOT / "experiments" / "motion_modes"
DEFAULT_OUTPUT_ROOT = REPO_ROOT / "output" / "motion_modes"
VO_BINARY = REPO_ROOT / "build" / "VO"
GT_PATH = REPO_ROOT / "Kitti" / "gt-tum07.txt"


def load_segments(mode_dir: Path) -> list[dict[str, str]]:
    csv_path = mode_dir / "segments.csv"
    with csv_path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        rows = list(reader)
    required = {"segment_id", "warmup_start_frame", "start_frame", "end_frame", "sequence", "is_primary", "notes"}
    missing = required.difference(reader.fieldnames or [])
    if missing:
        raise ValueError(f"{csv_path} is missing fields: {sorted(missing)}")
    return rows


def list_mode_dirs() -> list[Path]:
    return sorted(path for path in MODES_ROOT.iterdir() if path.is_dir() and (path / "segments.csv").exists())


def load_gt_lines() -> list[str]:
    return [
        line
        for line in GT_PATH.read_text(encoding="utf-8").splitlines()
        if line and not line.startswith("#")
    ]


def ensure_vo_binary() -> None:
    if VO_BINARY.exists():
        return
    raise SystemExit(
        "VO binary not found. Build it first with:\n"
        "  cmake -S Base -B build\n"
        "  cmake --build build -j"
    )


def select_segments(rows: Iterable[dict[str, str]], segment_id: str | None, primary_only: bool) -> list[dict[str, str]]:
    selected: list[dict[str, str]] = []
    for row in rows:
        if segment_id and row["segment_id"] != segment_id:
            continue
        if primary_only and row["is_primary"] != "1":
            continue
        selected.append(row)
    return selected


def write_gt_slice(gt_lines: list[str], start_frame: int, end_frame: int, output_path: Path) -> Path:
    clipped = gt_lines[start_frame:end_frame + 1]
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text("\n".join(clipped) + ("\n" if clipped else ""), encoding="utf-8")
    return output_path


def get_eval_timestamps(gt_lines: list[str], start_frame: int, end_frame: int) -> list[float]:
    timestamps: list[float] = []
    for line in gt_lines[start_frame:end_frame + 1]:
        timestamps.append(float(line.split()[0]))
    return timestamps


def slice_estimate_tum(estimate_path: Path, allowed_timestamps: list[float], tolerance: float = 1e-6) -> None:
    remaining = sorted(allowed_timestamps)
    lines: list[str] = []
    for line in estimate_path.read_text(encoding="utf-8").splitlines():
        if not line or not remaining:
            continue
        timestamp = float(line.split()[0])
        while remaining and remaining[0] + tolerance < timestamp:
            remaining.pop(0)
        if remaining and abs(timestamp - remaining[0]) <= tolerance:
            lines.append(line)
            remaining.pop(0)
    estimate_path.write_text("\n".join(lines) + ("\n" if lines else ""), encoding="utf-8")


def maybe_run(command: list[str], output_path: Path | None = None, env: dict[str, str] | None = None) -> bool:
    try:
        completed = subprocess.run(
            command,
            cwd=REPO_ROOT,
            check=True,
            text=True,
            capture_output=output_path is not None,
            env=env,
        )
    except FileNotFoundError:
        return False
    except subprocess.CalledProcessError as exc:
        if output_path is not None:
            output_path.write_text((exc.stdout or "") + "\n" + (exc.stderr or ""), encoding="utf-8")
        raise
    if output_path is not None:
        output_path.write_text(completed.stdout, encoding="utf-8")
    return True


def run_evo_step(
    name: str,
    command: list[str],
    output_path: Path,
    env: dict[str, str],
) -> dict[str, str | int]:
    try:
        maybe_run(command, output_path=output_path, env=env)
    except subprocess.CalledProcessError as exc:
        return {
            "status": "failed",
            "returncode": exc.returncode,
            "output": str(output_path.relative_to(REPO_ROOT)),
        }
    return {
        "status": "completed",
        "output": str(output_path.relative_to(REPO_ROOT)),
    }


def read_text_if_exists(path: Path) -> str:
    if not path.exists():
        return ""
    return path.read_text(encoding="utf-8")


def remove_flag(command: list[str], flag: str) -> list[str]:
    return [item for item in command if item != flag]


def write_run_log(path: Path, stdout: str | None, stderr: str | None) -> None:
    chunks: list[str] = []
    if stdout:
        chunks.append(stdout.rstrip())
    if stderr:
        chunks.append(stderr.rstrip())
    path.write_text("\n\n".join(chunk for chunk in chunks if chunk) + ("\n" if chunks else ""), encoding="utf-8")


def run_segment(
    mode_dir: Path,
    row: dict[str, str],
    gt_lines: list[str],
    skip_evo: bool,
    gui: bool,
    output_root: Path,
) -> None:
    mode_name = mode_dir.name
    segment_id = row["segment_id"]
    warmup_start_frame = int(row["warmup_start_frame"])
    start_frame = int(row["start_frame"])
    end_frame = int(row["end_frame"])
    eval_start_frame = max(start_frame, warmup_start_frame + 1)
    segment_output = output_root / mode_name / segment_id
    segment_output.mkdir(parents=True, exist_ok=True)

    output_root_arg = str(segment_output.relative_to(REPO_ROOT))
    env = os.environ.copy()
    if not gui:
        env["VO_ENABLE_GUI"] = "0"
    env.setdefault("VO_VERBOSE_LOG", "1")

    command = [
        str(VO_BINARY),
        "--start-frame",
        str(warmup_start_frame),
        "--end-frame",
        str(end_frame),
        "--output-root",
        output_root_arg,
    ]
    run_log_path = segment_output / "run.log"
    completed = subprocess.run(
        command,
        cwd=REPO_ROOT,
        check=False,
        env=env,
        text=True,
        capture_output=True,
    )
    write_run_log(run_log_path, completed.stdout, completed.stderr)
    if completed.returncode != 0:
        raise subprocess.CalledProcessError(
            completed.returncode,
            command,
            output=completed.stdout,
            stderr=completed.stderr,
        )

    estimate_path = segment_output / "position" / "position.txt"
    full_estimate_path = segment_output / "position" / "position_full.txt"
    shutil.copy2(estimate_path, full_estimate_path)

    reference_gt = write_gt_slice(gt_lines, eval_start_frame, end_frame, segment_output / "reference" / "gt_tum_segment.txt")
    allowed_timestamps = get_eval_timestamps(gt_lines, eval_start_frame, end_frame)
    slice_estimate_tum(estimate_path, allowed_timestamps)
    metadata = {
        "mode": mode_name,
        "segment_id": segment_id,
        "sequence": row["sequence"],
        "warmup_start_frame": warmup_start_frame,
        "start_frame": start_frame,
        "end_frame": end_frame,
        "evaluation_start_frame": eval_start_frame,
        "is_primary": row["is_primary"] == "1",
        "notes": row["notes"],
        "vo_command": command,
        "run_status": "completed",
        "run_log": str(run_log_path.relative_to(REPO_ROOT)),
        "reference_gt": str(reference_gt.relative_to(REPO_ROOT)),
        "full_estimate_path": str(full_estimate_path.relative_to(REPO_ROOT)),
        "estimate_path": str(estimate_path.relative_to(REPO_ROOT)),
    }
    (segment_output / "segment.json").write_text(json.dumps(metadata, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    if skip_evo:
        return

    evo_ape = shutil.which("evo_ape")
    evo_rpe = shutil.which("evo_rpe")
    evo_traj = shutil.which("evo_traj")
    if not (evo_ape and evo_rpe and evo_traj):
        metadata["evo_status"] = "skipped: evo tools not found in PATH"
        (segment_output / "segment.json").write_text(json.dumps(metadata, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        return

    eval_env = env.copy()
    eval_env.setdefault("MPLBACKEND", "Agg")
    eval_env.setdefault("MPLCONFIGDIR", "/tmp/evo-mpl")

    ape_dir = segment_output / "ape"
    rpe_dir = segment_output / "rpe"
    traj_dir = segment_output / "traj"
    ape_dir.mkdir(exist_ok=True)
    rpe_dir.mkdir(exist_ok=True)
    traj_dir.mkdir(exist_ok=True)

    ape_command = [
        evo_ape,
        "tum",
        str(reference_gt),
        str(estimate_path),
        "-v",
        "-a",
        "--plot_mode",
        "xz",
        "--save_plot",
        str(ape_dir / "evo_ape_plot.pdf"),
        "--no_warnings",
    ]
    rpe_command = [
        evo_rpe,
        "tum",
        str(reference_gt),
        str(estimate_path),
        "-v",
        "-a",
        "--plot_mode",
        "xz",
        "--save_plot",
        str(rpe_dir / "evo_rpe_plot.pdf"),
        "--no_warnings",
    ]
    traj_command = [
        evo_traj,
        "tum",
        str(estimate_path),
        "--ref",
        str(reference_gt),
        "--sync",
        "-a",
        "--plot_mode",
        "xz",
        "--save_plot",
        str(traj_dir / "evo_traj_xz.pdf"),
        "--save_table",
        str(traj_dir / "evo_traj_table.csv"),
        "--no_warnings",
    ]

    evo_results = {
        "ape": run_evo_step("ape", ape_command, ape_dir / "evo_ape_metrics.txt", eval_env),
        "rpe": run_evo_step("rpe", rpe_command, rpe_dir / "evo_rpe_metrics.txt", eval_env),
        "traj": run_evo_step("traj", traj_command, traj_dir / "evo_traj_metrics.txt", eval_env),
    }

    # 近静止段经常触发 Umeyama 退化。此时回退到 origin alignment，保住可比较的误差指标。
    ape_text = read_text_if_exists(ape_dir / "evo_ape_metrics.txt")
    rpe_text = read_text_if_exists(rpe_dir / "evo_rpe_metrics.txt")
    traj_text = read_text_if_exists(traj_dir / "evo_traj_metrics.txt")
    if "Degenerate covariance rank" in ape_text:
        fallback = run_evo_step(
            "ape_fallback_origin",
            remove_flag(ape_command, "-a")[:-1] + ["--align_origin", "--no_warnings"],
            ape_dir / "evo_ape_metrics.txt",
            eval_env,
        )
        fallback["fallback_alignment"] = "origin"
        evo_results["ape"] = fallback
    if "Degenerate covariance rank" in rpe_text:
        fallback = run_evo_step(
            "rpe_fallback_origin",
            remove_flag(rpe_command, "-a")[:-1] + ["--align_origin", "--no_warnings"],
            rpe_dir / "evo_rpe_metrics.txt",
            eval_env,
        )
        fallback["fallback_alignment"] = "origin"
        evo_results["rpe"] = fallback
    if "Degenerate covariance rank" in traj_text:
        fallback = run_evo_step(
            "traj_fallback_origin",
            remove_flag(traj_command, "-a")[:-1] + ["--align_origin", "--no_warnings"],
            traj_dir / "evo_traj_metrics.txt",
            eval_env,
        )
        fallback["fallback_alignment"] = "origin"
        evo_results["traj"] = fallback
    metadata["evo"] = evo_results
    metadata["evo_status"] = (
        "completed"
        if all(result["status"] == "completed" for result in evo_results.values())
        else "partial-failure"
    )
    (segment_output / "segment.json").write_text(json.dumps(metadata, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def record_run_failure(
    mode_dir: Path,
    row: dict[str, str],
    error: subprocess.CalledProcessError,
    output_root: Path,
) -> None:
    mode_name = mode_dir.name
    segment_id = row["segment_id"]
    segment_output = output_root / mode_name / segment_id
    segment_output.mkdir(parents=True, exist_ok=True)
    metadata = {
        "mode": mode_name,
        "segment_id": segment_id,
        "sequence": row["sequence"],
        "warmup_start_frame": int(row["warmup_start_frame"]),
        "start_frame": int(row["start_frame"]),
        "end_frame": int(row["end_frame"]),
        "evaluation_start_frame": int(row["start_frame"]),
        "is_primary": row["is_primary"] == "1",
        "notes": row["notes"],
        "run_status": "failed",
        "failure_stage": "vo",
        "returncode": error.returncode,
        "command": error.cmd,
    }
    run_log_path = segment_output / "run.log"
    write_run_log(run_log_path, error.output, error.stderr)
    metadata["run_log"] = str(run_log_path.relative_to(REPO_ROOT))
    (segment_output / "segment.json").write_text(json.dumps(metadata, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description="Run the VO binary on motion-mode segments.")
    scope = parser.add_mutually_exclusive_group(required=True)
    scope.add_argument("--mode", help="Mode directory name under experiments/motion_modes, for example 01_static.")
    scope.add_argument("--all", action="store_true", help="Run multiple modes.")
    parser.add_argument("--segment-id", help="Only run one segment inside the selected mode.")
    parser.add_argument("--primary-only", action="store_true", help="Only run rows marked is_primary=1.")
    parser.add_argument("--skip-evo", action="store_true", help="Skip evo evaluation and only keep VO outputs.")
    parser.add_argument("--gui", action="store_true", help="Keep GUI enabled for the VO binary.")
    parser.add_argument(
        "--output-root",
        default=str(DEFAULT_OUTPUT_ROOT),
        help="Root directory for motion-mode outputs. Default: output/motion_modes",
    )
    args = parser.parse_args()

    ensure_vo_binary()
    gt_lines = load_gt_lines()
    output_root = Path(args.output_root)
    if not output_root.is_absolute():
        output_root = REPO_ROOT / output_root

    if args.mode:
        mode_dirs = [MODES_ROOT / args.mode]
        if not mode_dirs[0].exists():
            raise SystemExit(f"Unknown mode: {args.mode}")
    else:
        mode_dirs = list_mode_dirs()

    for mode_dir in mode_dirs:
        rows = load_segments(mode_dir)
        selected = select_segments(rows, args.segment_id, args.primary_only)
        if args.segment_id and not selected:
            raise SystemExit(f"Segment {args.segment_id} not found in {mode_dir.name}")
        for row in selected:
            print(f"[run] {mode_dir.name}/{row['segment_id']}  frames={row['start_frame']}..{row['end_frame']}")
            try:
                run_segment(
                    mode_dir,
                    row,
                    gt_lines,
                    skip_evo=args.skip_evo,
                    gui=args.gui,
                    output_root=output_root,
                )
            except subprocess.CalledProcessError as exc:
                record_run_failure(mode_dir, row, exc, output_root=output_root)
                print(
                    f"[warn] {mode_dir.name}/{row['segment_id']} failed during VO run "
                    f"(returncode={exc.returncode}). Continuing with remaining segments.",
                    file=sys.stderr,
                )

    return 0


if __name__ == "__main__":
    sys.exit(main())
