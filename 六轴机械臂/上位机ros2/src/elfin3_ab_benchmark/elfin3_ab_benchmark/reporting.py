from __future__ import annotations

import csv
import json
import math
from collections import Counter, defaultdict
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Sequence

from .models import AttemptRecord, BenchmarkConfig


def percentile_nearest_rank(values: Sequence[float], percentile: float) -> float | None:
    finite_values = sorted(float(value) for value in values if math.isfinite(float(value)))
    if not finite_values:
        return None
    if not 0.0 <= percentile <= 100.0:
        raise ValueError("percentile must be within [0, 100]")
    if percentile == 0.0:
        return finite_values[0]
    rank = max(1, math.ceil(percentile / 100.0 * len(finite_values)))
    return finite_values[rank - 1]


def _rate(numerator: int, denominator: int) -> float | None:
    return numerator / denominator if denominator else None


def aggregate_records(records: Iterable[AttemptRecord]) -> Dict[str, Any]:
    record_list = list(records)
    algorithms: Dict[str, Any] = {}
    grouped: Mapping[str, List[AttemptRecord]] = defaultdict(list)
    for record in record_list:
        grouped[record.algorithm].append(record)

    for algorithm in ("move_pose", "move_pose_ptp"):
        rows = grouped.get(algorithm, [])
        accepted_rows = [row for row in rows if row.goal_accepted]
        metric_rows = [row for row in rows if row.planning_metrics_received]
        eligible_rows = [row for row in metric_rows if row.planning_eligible is True]
        planned_rows = [row for row in eligible_rows if row.planning_success is True]
        total_times = [
            row.total_planning_time_ms
            for row in planned_rows
            if row.total_planning_time_ms is not None
        ]
        ompl_times = [
            row.ompl_planning_time_ms
            for row in planned_rows
            if row.ompl_planning_time_ms is not None
        ]
        adjacent_deltas = [
            row.max_adjacent_delta_rad
            for row in planned_rows
            if row.max_adjacent_delta_rad is not None
        ]
        action_successes = sum(1 for row in accepted_rows if row.action_success)
        algorithms[algorithm] = {
            "records": len(rows),
            "goals_accepted": len(accepted_rows),
            "planning_metrics_received": len(metric_rows),
            "planning_eligible": len(eligible_rows),
            "planning_successes": len(planned_rows),
            "planning_success_rate": _rate(len(planned_rows), len(eligible_rows)),
            "action_successes": action_successes,
            "action_success_rate": _rate(action_successes, len(accepted_rows)),
            "p50_total_planning_time_ms": percentile_nearest_rank(total_times, 50.0),
            "p95_total_planning_time_ms": percentile_nearest_rank(total_times, 95.0),
            "p95_ompl_planning_time_ms": percentile_nearest_rank(ompl_times, 95.0),
            "max_adjacent_delta_rad": max(adjacent_deltas) if adjacent_deltas else None,
            "segmented_fallback_successes": sum(
                1 for row in planned_rows if row.ik_method == "segmented"
            ),
            "failure_classifications": dict(Counter(row.classification for row in rows)),
        }

    pairs: Mapping[str, Dict[str, AttemptRecord]] = defaultdict(dict)
    for record in record_list:
        pairs[record.pair_id][record.algorithm] = record
    paired_complete = 0
    ptp_wins = 0
    move_pose_wins = 0
    ties_success = 0
    ties_failure = 0
    for pair in pairs.values():
        if "move_pose" not in pair or "move_pose_ptp" not in pair:
            continue
        move_pose = pair["move_pose"]
        ptp = pair["move_pose_ptp"]
        if move_pose.planning_eligible is not True or ptp.planning_eligible is not True:
            continue
        paired_complete += 1
        move_pose_success = move_pose.planning_success is True
        ptp_success = ptp.planning_success is True
        if ptp_success and not move_pose_success:
            ptp_wins += 1
        elif move_pose_success and not ptp_success:
            move_pose_wins += 1
        elif ptp_success:
            ties_success += 1
        else:
            ties_failure += 1

    return {
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "algorithms": algorithms,
        "paired": {
            "complete_pairs": paired_complete,
            "move_pose_ptp_wins": ptp_wins,
            "move_pose_wins": move_pose_wins,
            "both_succeeded": ties_success,
            "both_failed": ties_failure,
        },
    }


def _format_value(value: Any) -> str:
    if value is None:
        return "N/A"
    if isinstance(value, float):
        return f"{value:.6f}"
    return str(value)


def _format_rate(value: Any) -> str:
    return "N/A" if value is None else f"{100.0 * float(value):.2f}%"


def write_reports(
    output_root: str,
    config: BenchmarkConfig,
    records: Sequence[AttemptRecord],
) -> Path:
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    output_directory = Path(output_root).expanduser().resolve() / f"ab_benchmark_{timestamp}"
    output_directory.mkdir(parents=True, exist_ok=False)

    record_dicts = [record.to_dict() for record in records]
    summary = aggregate_records(records)
    with (output_directory / "records.json").open("w", encoding="utf-8") as stream:
        json.dump(record_dicts, stream, ensure_ascii=False, indent=2, allow_nan=False)
    with (output_directory / "summary.json").open("w", encoding="utf-8") as stream:
        json.dump(summary, stream, ensure_ascii=False, indent=2, allow_nan=False)
    with (output_directory / "config.json").open("w", encoding="utf-8") as stream:
        json.dump(asdict(config), stream, ensure_ascii=False, indent=2, allow_nan=False)

    fieldnames = list(AttemptRecord("", "", "", 0, 0, "", "").to_dict().keys())
    with (output_directory / "records.csv").open(
        "w", encoding="utf-8", newline=""
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for record_dict in record_dicts:
            row = dict(record_dict)
            row["feedback_stages"] = "|".join(row["feedback_stages"])
            writer.writerow(row)

    failed_rows = [row for row in record_dicts if row["classification"] != "SUCCESS"]
    with (output_directory / "failed_cases.csv").open(
        "w", encoding="utf-8", newline=""
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for failed_row in failed_rows:
            row = dict(failed_row)
            row["feedback_stages"] = "|".join(row["feedback_stages"])
            writer.writerow(row)

    with (output_directory / "summary.md").open("w", encoding="utf-8") as stream:
        stream.write("# Elfin3 MovePose vs MovePosePTP A/B Benchmark\n\n")
        stream.write(
            "Result scope: `elfin_base` TCP Y must satisfy "
            f"`y >= {config.effective_y_min_m:.6f} m` "
            f"(`{config.y_safe_min_m:.6f} + {config.y_safety_margin_m:.6f}`).\n\n"
        )
        stream.write(
            "This is a sampled software guard for TCP only; it is not whole-arm "
            "continuous-path proof or functional safety certification.\n\n"
        )
        stream.write(
            "| Algorithm | Eligible | Planning success | Action success | "
            "P95 total planning (ms) | P95 OMPL (ms) | Max adjacent delta (rad) |\n"
        )
        stream.write("|---|---:|---:|---:|---:|---:|---:|\n")
        for algorithm in ("move_pose", "move_pose_ptp"):
            item = summary["algorithms"][algorithm]
            stream.write(
                f"| {algorithm} | {item['planning_eligible']} | "
                f"{_format_rate(item['planning_success_rate'])} | "
                f"{_format_rate(item['action_success_rate'])} | "
                f"{_format_value(item['p95_total_planning_time_ms'])} | "
                f"{_format_value(item['p95_ompl_planning_time_ms'])} | "
                f"{_format_value(item['max_adjacent_delta_rad'])} |\n"
            )
        paired = summary["paired"]
        stream.write("\n## Paired comparison\n\n")
        stream.write(f"- Complete eligible pairs: {paired['complete_pairs']}\n")
        stream.write(f"- MovePosePTP wins: {paired['move_pose_ptp_wins']}\n")
        stream.write(f"- MovePose wins: {paired['move_pose_wins']}\n")
        stream.write(f"- Both succeeded: {paired['both_succeeded']}\n")
        stream.write(f"- Both failed: {paired['both_failed']}\n")

    return output_directory
