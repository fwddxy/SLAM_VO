#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
import json
import math
import re
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT_ROOT = REPO_ROOT / "output" / "motion_modes"

METRIC_PATTERN = re.compile(r"^\s*([a-z]+)\s+([0-9eE.+-]+)\s*$")


def read_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def count_nonempty_lines(path: Path) -> int:
    if not path.exists():
        return 0
    return sum(1 for line in path.read_text(encoding="utf-8").splitlines() if line.strip())


def parse_evo_metrics(path: Path) -> dict[str, float | str]:
    metrics: dict[str, float | str] = {}
    if not path.exists():
        return metrics
    for line in path.read_text(encoding="utf-8").splitlines():
        match = METRIC_PATTERN.match(line)
        if not match:
            continue
        key, value = match.groups()
        try:
            metrics[key] = float(value)
        except ValueError:
            metrics[key] = value
    return metrics


def parse_run_log(path: Path) -> dict[str, object]:
    result = {
        "skip_pose_count": 0,
        "reinit_failed_count": 0,
        "max_inliers": "",
        "min_inliers": "",
        "final_status": "",
        "final_tracked": "",
        "final_inliers": "",
        "ended_with_skip_pose": False,
    }
    if not path.exists():
        return result

    frame_line_pattern = re.compile(r"Frame\s+(\d+).+tracked=(\d+)(?:\s+inliers=(\d+))?\s+status=([a-z\-]+)")
    tracked_pose_pattern = re.compile(r"Frame\s+(\d+).+tracked=(\d+)\s+inliers=(\d+)\s+pose=\(")
    inliers: list[int] = []

    for line in path.read_text(encoding="utf-8").splitlines():
        if "status=skip-pose" in line:
            result["skip_pose_count"] = int(result["skip_pose_count"]) + 1
        if "status=reinit-failed" in line:
            result["reinit_failed_count"] = int(result["reinit_failed_count"]) + 1

        status_match = frame_line_pattern.search(line)
        if status_match:
            _, tracked, inlier_text, status = status_match.groups()
            result["final_status"] = status
            result["final_tracked"] = int(tracked)
            result["final_inliers"] = int(inlier_text) if inlier_text is not None else ""
            result["ended_with_skip_pose"] = status == "skip-pose"
            if inlier_text is not None:
                inliers.append(int(inlier_text))
            continue

        pose_match = tracked_pose_pattern.search(line)
        if pose_match:
            _, tracked, inlier_text = pose_match.groups()
            result["final_status"] = "pose"
            result["final_tracked"] = int(tracked)
            result["final_inliers"] = int(inlier_text)
            result["ended_with_skip_pose"] = False
            inliers.append(int(inlier_text))

    if inliers:
        result["max_inliers"] = max(inliers)
        result["min_inliers"] = min(inliers)
    return result


def build_rows(output_root: Path) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for metadata_path in sorted(output_root.glob("*/*/segment.json")):
        metadata = read_json(metadata_path)
        segment_dir = metadata_path.parent
        estimate_path_text = metadata.get("estimate_path", "")
        full_estimate_path_text = metadata.get("full_estimate_path", "")
        geometry_estimate_path_text = metadata.get("geometry_estimate_path", "")
        full_geometry_estimate_path_text = metadata.get("full_geometry_estimate_path", "")
        reference_gt_text = metadata.get("reference_gt", "")
        run_log_text = metadata.get("run_log", "")

        estimate_path = REPO_ROOT / estimate_path_text if estimate_path_text else segment_dir / "position" / "position.txt"
        full_estimate_path = (
            REPO_ROOT / full_estimate_path_text
            if full_estimate_path_text
            else segment_dir / "position" / "position_full.txt"
        )
        geometry_estimate_path = (
            REPO_ROOT / geometry_estimate_path_text
            if geometry_estimate_path_text
            else segment_dir / "position" / "geometry_position.txt"
        )
        full_geometry_estimate_path = (
            REPO_ROOT / full_geometry_estimate_path_text
            if full_geometry_estimate_path_text
            else segment_dir / "position" / "geometry_position_full.txt"
        )
        reference_gt = REPO_ROOT / reference_gt_text if reference_gt_text else segment_dir / "reference" / "gt_tum_segment.txt"
        run_log_path = REPO_ROOT / run_log_text if run_log_text else segment_dir / "run.log"

        ape_metrics = parse_evo_metrics(segment_dir / "ape" / "evo_ape_metrics.txt")
        rpe_metrics = parse_evo_metrics(segment_dir / "rpe" / "evo_rpe_metrics.txt")
        run_metrics = parse_run_log(run_log_path)

        row = {
            "mode": metadata["mode"],
            "segment_id": metadata["segment_id"],
            "sequence": metadata["sequence"],
            "is_primary": metadata["is_primary"],
            "warmup_start_frame": metadata["warmup_start_frame"],
            "start_frame": metadata["start_frame"],
            "end_frame": metadata["end_frame"],
            "evaluation_start_frame": metadata["evaluation_start_frame"],
            "evaluated_frames": count_nonempty_lines(reference_gt),
            "estimate_frames": count_nonempty_lines(estimate_path),
            "full_run_frames": count_nonempty_lines(full_estimate_path),
            "geometry_estimate_frames": count_nonempty_lines(geometry_estimate_path),
            "geometry_full_run_frames": count_nonempty_lines(full_geometry_estimate_path),
            "run_status": metadata.get("run_status", "unknown"),
            "evo_status": metadata.get("evo_status", "not-run"),
            "estimate_semantics": metadata.get("estimate_semantics", "final_geometry_spine"),
            "geometry_estimate_semantics": metadata.get("geometry_estimate_semantics", ""),
            "ape_rmse_m": ape_metrics.get("rmse", ""),
            "ape_mean_m": ape_metrics.get("mean", ""),
            "ape_median_m": ape_metrics.get("median", ""),
            "ape_std_m": ape_metrics.get("std", ""),
            "rpe_rmse_m": rpe_metrics.get("rmse", ""),
            "rpe_mean_m": rpe_metrics.get("mean", ""),
            "rpe_median_m": rpe_metrics.get("median", ""),
            "rpe_std_m": rpe_metrics.get("std", ""),
            "skip_pose_count": run_metrics["skip_pose_count"],
            "reinit_failed_count": run_metrics["reinit_failed_count"],
            "max_inliers": run_metrics["max_inliers"],
            "min_inliers": run_metrics["min_inliers"],
            "final_status": run_metrics["final_status"],
            "final_tracked": run_metrics["final_tracked"],
            "final_inliers": run_metrics["final_inliers"],
            "ended_with_skip_pose": run_metrics["ended_with_skip_pose"],
            "notes": metadata["notes"],
        }
        rows.append(row)
    return rows


def as_float(value: object) -> float | None:
    if value == "":
        return None
    if isinstance(value, (int, float)):
        return float(value)
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def compute_mean(values: list[float]) -> float:
    return sum(values) / len(values)


def compute_std(values: list[float]) -> float:
    if len(values) <= 1:
        return 0.0
    mean = compute_mean(values)
    variance = sum((value - mean) ** 2 for value in values) / len(values)
    return math.sqrt(variance)


def build_mode_statistics(rows: list[dict[str, object]]) -> list[dict[str, object]]:
    grouped: dict[str, list[dict[str, object]]] = {}
    for row in rows:
        grouped.setdefault(str(row["mode"]), []).append(row)

    stats_rows: list[dict[str, object]] = []
    for mode, mode_rows in sorted(grouped.items()):
        ape_rmse_values = [value for value in (as_float(row["ape_rmse_m"]) for row in mode_rows) if value is not None]
        rpe_rmse_values = [value for value in (as_float(row["rpe_rmse_m"]) for row in mode_rows) if value is not None]
        ape_mean_values = [value for value in (as_float(row["ape_mean_m"]) for row in mode_rows) if value is not None]
        rpe_mean_values = [value for value in (as_float(row["rpe_mean_m"]) for row in mode_rows) if value is not None]
        skip_counts = [value for value in (as_float(row["skip_pose_count"]) for row in mode_rows) if value is not None]
        reinit_counts = [value for value in (as_float(row["reinit_failed_count"]) for row in mode_rows) if value is not None]
        completed_count = sum(1 for row in mode_rows if row["evo_status"] == "completed")
        ended_with_skip_count = sum(1 for row in mode_rows if row["ended_with_skip_pose"])

        stats_rows.append(
            {
                "mode": mode,
                "segments_total": len(mode_rows),
                "segments_completed": completed_count,
                "ape_rmse_mean_m": compute_mean(ape_rmse_values) if ape_rmse_values else "",
                "ape_rmse_std_m": compute_std(ape_rmse_values) if ape_rmse_values else "",
                "ape_rmse_min_m": min(ape_rmse_values) if ape_rmse_values else "",
                "ape_rmse_max_m": max(ape_rmse_values) if ape_rmse_values else "",
                "rpe_rmse_mean_m": compute_mean(rpe_rmse_values) if rpe_rmse_values else "",
                "rpe_rmse_std_m": compute_std(rpe_rmse_values) if rpe_rmse_values else "",
                "rpe_rmse_min_m": min(rpe_rmse_values) if rpe_rmse_values else "",
                "rpe_rmse_max_m": max(rpe_rmse_values) if rpe_rmse_values else "",
                "ape_mean_mean_m": compute_mean(ape_mean_values) if ape_mean_values else "",
                "rpe_mean_mean_m": compute_mean(rpe_mean_values) if rpe_mean_values else "",
                "skip_pose_mean": compute_mean(skip_counts) if skip_counts else "",
                "skip_pose_max": max(skip_counts) if skip_counts else "",
                "reinit_failed_mean": compute_mean(reinit_counts) if reinit_counts else "",
                "ended_with_skip_segments": ended_with_skip_count,
            }
        )
    return stats_rows


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    fieldnames = list(rows[0].keys())
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def format_number(value: object) -> str:
    if value == "":
        return ""
    if isinstance(value, float):
        return f"{value:.6f}"
    return str(value)


def write_markdown(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "# Motion Mode Summary",
        "",
        "| Mode | Segment | Frames | APE RMSE (m) | RPE RMSE (m) | Skip Pose | Final Status | evo |",
        "| --- | --- | ---: | ---: | ---: | ---: | --- | --- |",
    ]
    for row in rows:
        lines.append(
            "| "
            + " | ".join(
                [
                    str(row["mode"]),
                    str(row["segment_id"]),
                    str(row["evaluated_frames"]),
                    format_number(row["ape_rmse_m"]),
                    format_number(row["rpe_rmse_m"]),
                    str(row["skip_pose_count"]),
                    str(row["final_status"]),
                    str(row["evo_status"]),
                ]
            )
            + " |"
        )
    lines.append("")
    path.write_text("\n".join(lines), encoding="utf-8")


def write_mode_statistics_markdown(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "# Motion Mode Statistics",
        "",
        "| Mode | Segments | APE RMSE Mean (m) | APE RMSE Std (m) | RPE RMSE Mean (m) | RPE RMSE Std (m) | Skip Pose Mean | Ended With Skip |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for row in rows:
        lines.append(
            "| "
            + " | ".join(
                [
                    str(row["mode"]),
                    f"{row['segments_completed']}/{row['segments_total']}",
                    format_number(row["ape_rmse_mean_m"]),
                    format_number(row["ape_rmse_std_m"]),
                    format_number(row["rpe_rmse_mean_m"]),
                    format_number(row["rpe_rmse_std_m"]),
                    format_number(row["skip_pose_mean"]),
                    str(row["ended_with_skip_segments"]),
                ]
            )
            + " |"
        )
    lines.append("")
    path.write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description="Summarize motion-mode outputs.")
    parser.add_argument(
        "--output-root",
        default=str(DEFAULT_OUTPUT_ROOT),
        help="Root directory containing motion-mode outputs. Default: output/motion_modes",
    )
    args = parser.parse_args()

    output_root = Path(args.output_root)
    if not output_root.is_absolute():
        output_root = REPO_ROOT / output_root
    summary_root = output_root / "summary"

    rows = build_rows(output_root)
    primary_rows = [row for row in rows if row["is_primary"]]
    mode_stats_rows = build_mode_statistics(rows)

    write_csv(summary_root / "all_segments.csv", rows)
    write_csv(summary_root / "primary_segments.csv", primary_rows)
    write_csv(summary_root / "mode_statistics.csv", mode_stats_rows)
    write_markdown(summary_root / "primary_segments.md", primary_rows)
    write_markdown(summary_root / "all_segments.md", rows)
    write_mode_statistics_markdown(summary_root / "mode_statistics.md", mode_stats_rows)
    print(f"Wrote {len(rows)} segment rows to {summary_root}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
