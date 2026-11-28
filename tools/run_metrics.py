"""실행 기록(records/run/<기록>/)에서 떨림과 처짐의 지표를 낸다 (docs/decisions/0042의 재는 법).

    uv run tools/run_metrics.py <기록 폴더> [<기록 폴더> ...] [--json <파일>]
    ssh hori1 'python3 - /var/lib/horilab/records/run/<기록> ...' < tools/run_metrics.py   (컨트롤러 파이에서)

기록을 여럿 주면 기록마다 지표를 보이고, 첫 기록을 기준(보통 지금 설정)으로 놓고 관절마다 견준다. 표준 라이브러리만
써서 컨트롤러 파이의 python3로도 돈다 (기록을 PC로 받기 전에 본다).

지표 (관절마다. 한 칸은 서보 값 1, 0.088°):
- 떨림: 관절 하나만 움직인 움직이기에서, 움직이는 동안 1초 창마다 (읽은 위치 - 목표)에서 0.2초 이동 평균을 뺀 값의
  폭. 빠르기마다 가운데 값과 가장 큰 값. 기준: 가장 큰 값이 2칸 안.
- 뒤처짐: 같은 움직이기에서 |읽은 위치 - 목표|의 가운데 값과 가장 큰 값, 그리고 여러 관절을 같이 움직인 것까지 모든
  움직이기에서 가장 큰 값. 기준은 없다 (따라가기 오차 기준을 고를 때 쓴다).
- 처짐: 버티기 단계에서 자리 잡은 뒤(시작에서 1초 뒤부터) 읽은 위치 - 목표의 가운데 값 (부호는 운동학의 관절 각).
  기준: 1칸 안.
- 헌팅: 같은 동안 읽은 위치의 폭. 한두 박자 튀어 읽힌 값은 거른다 (1 %와 99 % 자리 사이의 폭: 처음 전원의 어깨
  버티기에서 J2가 4초에 한 박자만 한 칸 더 읽혔다). 기준: 1칸 안.
- 넘침: 움직임이 끝난 뒤 버티는 동안, 움직이던 쪽으로 목표와 자리 잡은 위치 가운데 먼 쪽을 넘은 가장 큰 양 (두 박자
  넘게 이어진 것. 처짐이 움직이던 쪽이면 처짐은 넘침에 넣지 않는다). 기준: 2칸 안.
- 열과 부하: 온도(처음 1초와 끝 1초의 가운데 값, 1초 가운데 값의 가장 큰 것: 한두 박자씩 튀어 읽히는 값을 거른다),
  |부하|의 평균(관절 하나만 움직일 때, 버틸 때)과 가장 큰 값. 기준은 없다: 첫 기록과 견주어 눈에 띄게 늘었는지 본다.

목표는 기록의 target 칸(실행 루프가 노린 자리, 처짐 보정을 뺀 것)을 한 박자 늦춰 쓴다: 그 박자에 읽은 위치는 지난
박자에 쓴 목표를 따라간 것이다. target 칸이 없는 기록(2026-11-28까지)은 goal 칸(서보에서 읽은 목표, 지난 박자에 쓴
것)을 쓴다: 처짐 보정을 켜지 않았으면 둘이 같다.
"""

from __future__ import annotations

import argparse
import csv
import itertools
import json
import statistics
import sys
import tomllib
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

STEP_DEG = 360 / 4096  # 서보 값 한 칸 [°]
SETTLE_S = 1.0  # 움직임이 끝나고 자리 잡는 시간: 처짐과 헌팅은 그 뒤로 본다
WINDOW_S = 1.0  # 떨림을 재는 창
AVERAGE_HALF = 10  # 떨림에서 빼는 이동 평균의 반 폭 (박자): 21박자, 0.2초
STOP_S = 5.0  # 넘침을 보는 시간 (움직임이 끝난 뒤)
READING_ROWS = 101  # 온도의 가운데 값을 내는 박자 수 (실행 루프의 판단 창과 같다, 약 1초)
MOVED_STEPS = 2  # 움직이기에서 목표가 이만큼(칸) 넘게 바뀐 관절을 움직인 관절로 본다
HOLDING, MOVING = 0, 1  # 기록의 mode 칸
GRIPPER = "gripper"  # 그리퍼는 온도와 부하만 본다

# 통과 기준 [칸] (0042)
LIMITS = {"judder": 2.0, "sag": 1.0, "hunting": 1.0, "overshoot": 2.0}


@dataclass
class Series:
    """서보 하나의 박자마다 값 (서보 값, 0.1 %, °C)."""

    position: list[int]
    reference: list[int]
    load: list[int]
    temperature: list[int]
    direction: int  # 관절 각이 늘 때 서보 값이 늘면 1, 줄면 -1


@dataclass
class Record:
    folder: Path
    meta: dict[str, Any]
    t: list[float]
    labels: list[str]
    modes: list[int]
    servos: dict[str, Series]
    has_target: bool
    script_steps: list[dict[str, Any]]
    script_speed: float  # [%]
    max_speed_deg_s: float


@dataclass
class Segment:
    """같은 단계 이름(`3-move`, `4-hold`)이 이어진 박자들 [start, end)."""

    label: str
    number: int | None  # 순서의 단계 번호 (1부터)
    kind: str
    start: int
    end: int


@dataclass
class JointMetrics:
    """관절 하나의 지표. 값은 칸(서보 값), 처짐은 운동학 부호를 붙인 칸이다."""

    judder: dict[float, list[float]] = field(default_factory=dict)  # 빠르기 [°/s] → 1초 창마다 폭
    lag: dict[float, list[float]] = field(default_factory=dict)  # 빠르기 → 박자마다 |차이|
    lag_all: int = 0  # 모든 움직이기에서 가장 큰 |차이|
    holds: list[tuple[str, float, float]] = field(default_factory=list)  # (단계, 처짐, 헌팅)
    overshoots: list[tuple[str, float]] = field(default_factory=list)  # (단계, 넘침)
    temperature: tuple[float, float, float] | None = None  # 처음, 끝, 가장 높은 것 [°C]
    load_moving: list[int] = field(default_factory=list)  # |부하| [0.1 %]
    load_holding: list[int] = field(default_factory=list)
    load_peak: int = 0


def load_record(folder: Path) -> Record:
    meta: dict[str, Any] = json.loads((folder / "meta.json").read_text(encoding="utf-8"))
    with (folder / "samples.csv").open(encoding="utf-8", newline="") as f:
        reader = csv.reader(f)
        header = next(reader)
        rows = list(reader)
    column = {name: k for k, name in enumerate(header)}
    names: list[str] = meta.get("joints") or [
        name.removesuffix("_position") for name in header if name.endswith("_position")
    ]
    robot = tomllib.loads(meta.get("robot_toml", ""))
    script = tomllib.loads(meta.get("script", ""))
    has_target = all(f"{name}_target" in column for name in names)

    def ints(name: str) -> list[int]:
        k = column[name]
        return [int(row[k]) for row in rows]

    servos: dict[str, Series] = {}
    for name in names:
        if name == GRIPPER:
            direction = robot.get("gripper", {}).get("direction", 1)
        else:
            direction = robot.get("joints", {}).get(name, {}).get("direction", 1)
        if has_target:
            target = ints(f"{name}_target")
            reference = target[:1] + target[:-1]
        else:
            reference = ints(f"{name}_goal")
        servos[name] = Series(
            position=ints(f"{name}_position"),
            reference=reference,
            load=ints(f"{name}_load"),
            temperature=ints(f"{name}_temperature"),
            direction=int(direction),
        )
    return Record(
        folder=folder,
        meta=meta,
        t=[float(row[column["t_s"]]) for row in rows],
        labels=[row[column["step"]] for row in rows],
        modes=ints("mode"),
        servos=servos,
        has_target=has_target,
        script_steps=list(script.get("steps", [])),
        script_speed=float(script.get("speed_percent", 100.0)),
        max_speed_deg_s=float(robot.get("run_loop", {}).get("max_speed_deg_s", 0.0)),
    )


def segments(labels: list[str]) -> list[Segment]:
    found: list[Segment] = []
    start = 0
    for k in range(1, len(labels) + 1):
        if k == len(labels) or labels[k] != labels[start]:
            label = labels[start]
            number_text, _, kind = label.partition("-")
            number = int(number_text) if number_text.isdigit() else None
            found.append(Segment(label, number, kind if number is not None else label, start, k))
            start = k
    return found


def moving_average(values: list[int], half: int) -> list[float]:
    """가운데에 놓인 이동 평균 (끝에서는 있는 값만)."""
    prefix = [0.0]
    for value in values:
        prefix.append(prefix[-1] + value)
    n = len(values)
    out: list[float] = []
    for k in range(n):
        low, high = max(0, k - half), min(n, k + half + 1)
        out.append((prefix[high] - prefix[low]) / (high - low))
    return out


def spread(values: list[int]) -> float:
    """1 %와 99 % 자리 사이의 폭: 한두 박자 튀어 읽힌 값을 거른다."""
    ordered = sorted(values)
    n = len(ordered)
    return float(ordered[round(0.99 * (n - 1))] - ordered[round(0.01 * (n - 1))])


def lasting_peak(values: list[int]) -> int:
    """두 박자 넘게 이어진 값 가운데 가장 큰 것 (한 박자 튄 값을 거른다). 값이 하나면 그 값."""
    if len(values) < 2:
        return max(values)
    return max(min(a, b) for a, b in itertools.pairwise(values))


def windows(t: list[float], first: int, last: int) -> list[tuple[int, int]]:
    """[first, last] 안의 1초 창들 [i, j). 끝을 넘는 마지막 창은 버린다."""
    out: list[tuple[int, int]] = []
    i = first
    while i <= last:
        j = i
        while j <= last and t[j] - t[i] < WINDOW_S - 1e-9:
            j += 1
        if j > last:
            break
        out.append((i, j))
        i = j
    return out


def step_speed(record: Record, segment: Segment) -> float:
    """움직이기 단계의 빠르기 [°/s]: 단계의 speed_percent, 없으면 순서의 것."""
    percent = record.script_speed
    if segment.number is not None and segment.number <= len(record.script_steps):
        percent = float(record.script_steps[segment.number - 1].get("speed_percent", percent))
    return round(percent / 100.0 * record.max_speed_deg_s, 2)


def analyze(record: Record) -> dict[str, JointMetrics]:
    metrics = {name: JointMetrics() for name in record.servos}
    t, modes = record.t, record.modes
    errors = {
        name: [p - r for p, r in zip(s.position, s.reference, strict=True)]
        for name, s in record.servos.items()
    }
    averages = {name: moving_average(e, AVERAGE_HALF) for name, e in errors.items()}
    parts = segments(record.labels)
    for segment in parts:
        rows = range(segment.start, segment.end)
        if segment.kind == "move":
            moving = [k for k in rows if modes[k] == MOVING]
            if not moving:
                continue
            first, last = moving[0], moving[-1]
            moved = [
                name
                for name, s in record.servos.items()
                if name != GRIPPER
                and abs(s.reference[segment.end - 1] - s.reference[segment.start]) >= MOVED_STEPS
            ]
            speed = step_speed(record, segment)
            for name in moved:
                s, e, m = record.servos[name], errors[name], metrics[name]
                m.lag_all = max(m.lag_all, max(abs(e[k]) for k in moving))
                if len(moved) == 1:
                    widths = []
                    for i, j in windows(t, first, last):
                        residual = [e[k] - averages[name][k] for k in range(i, j)]
                        widths.append(max(residual) - min(residual))
                    m.judder.setdefault(speed, []).extend(widths)
                    m.lag.setdefault(speed, []).extend(abs(e[k]) for k in moving)
                    m.load_moving.extend(abs(s.load[k]) for k in moving)
                # 넘침: 움직임이 끝난 뒤 버티는 동안 (다음 움직임이 시작하기 전, STOP_S까지)
                way = 1 if s.reference[last] > s.reference[segment.start] else -1
                stop = []
                k = last + 1
                while k < len(t) and modes[k] == HOLDING and t[k] - t[last] <= STOP_S:
                    stop.append(k)
                    k += 1
                settled = [k for k in stop if t[k] - t[last] >= SETTLE_S]
                if len(settled) * 2 < len(stop) or not settled:
                    continue  # 자리 잡은 뒤를 볼 만큼 버티지 않았다
                rest = statistics.median(e[k] * way for k in settled)
                peak = lasting_peak([e[k] * way for k in stop])
                m.overshoots.append((segment.label, max(0.0, peak - max(rest, 0.0))))
        elif segment.kind == "hold":
            settled = [
                k for k in rows if modes[k] == HOLDING and t[k] - t[segment.start] >= SETTLE_S
            ]
            if not settled or t[settled[-1]] - t[settled[0]] < WINDOW_S - 0.05:
                continue
            for name, s in record.servos.items():
                if name == GRIPPER:
                    continue  # 쥔 짐에 막혀 목표 앞에 선 것은 처짐이 아니다
                e = errors[name]
                # + 0.0: 처짐이 0이면 방향을 곱해도 -0이 아니게
                sag = statistics.median(e[k] for k in settled) * s.direction + 0.0
                hunting = spread([s.position[k] for k in settled])
                metrics[name].holds.append((segment.label, float(sag), hunting))
                metrics[name].load_holding.extend(abs(s.load[k]) for k in settled)
    for name, s in record.servos.items():
        temperature = s.temperature
        if len(temperature) >= READING_ROWS:
            medians = [
                statistics.median(temperature[i : i + READING_ROWS])
                for i in range(0, len(temperature) - READING_ROWS + 1, READING_ROWS)
            ]
            metrics[name].temperature = (
                float(statistics.median(temperature[:READING_ROWS])),
                float(statistics.median(temperature[-READING_ROWS:])),
                float(max(medians)),
            )
        metrics[name].load_peak = max((abs(v) for v in s.load), default=0)
    return metrics


# 보이기


def verdict(value: float, limit: float) -> str:
    return "통과" if value <= limit + 1e-9 else "넘음"


def steps_text(value: float) -> str:
    return f"{value:.1f}칸 ({value * STEP_DEG:.2f}°)"


def speed_text(speed: float) -> str:
    return f"{speed:g}°/s"


def trial_text(meta: dict[str, Any]) -> str:
    """기록의 시험 값 (--trial): 쓰기 전 → 쓴 값. 없으면 지금 설정."""
    trial = meta.get("trial") or {}
    parts: list[str] = []
    written = trial.get("written") or {}
    asked = trial.get("asked") or {}
    if written:
        for joint, values in written.items():
            changes = ", ".join(f"{k} {v['before']}→{v['trial']}" for k, v in values.items())
            parts.append(f"{joint} {changes}")
    else:
        for joint, values in (asked.get("servos") or {}).items():
            parts.append(f"{joint} " + ", ".join(f"{k} {v}" for k, v in values.items()))
    gain = asked.get("sag_gain_per_s")
    if gain is not None:
        parts.append(f"처짐 보정 이득 {gain:g}/s")
    if trial.get("restored") not in (None, True):
        parts.append(f"되돌리지 못했다: {trial['restored']}")
    return " | ".join(parts) if parts else "없음 (지금 설정)"


def servo_values_text(meta: dict[str, Any]) -> str:
    values = meta.get("servo_values") or {}
    parts = []
    for joint, v in values.items():
        if "error" in v:
            parts.append(f"{joint} 못 읽음")
        else:
            parts.append(
                f"{joint} P{v.get('p_gain')} D{v.get('d_gain')} I{v.get('i_gain')} "
                f"가속{v.get('acceleration')}"
            )
    return " | ".join(parts)


def report(record: Record, metrics: dict[str, JointMetrics]) -> list[str]:
    meta = record.meta
    lines = [
        f"== {record.folder.name}: {meta.get('script_name', '?')} (끝: {meta.get('end', '?')})",
        f"   시험 값: {trial_text(meta)}",
    ]
    servo_values = servo_values_text(meta)
    if servo_values:
        lines.append(f"   서보 값(시험 값을 쓰기 전): {servo_values}")
    if not record.has_target:
        lines.append("   목표: goal 칸 (target 칸이 없는 기록)")
    lines.append(
        f"   기준: 떨림 {LIMITS['judder']:g}칸, 처짐 {LIMITS['sag']:g}칸, 헌팅 {LIMITS['hunting']:g}칸, "
        f"넘침 {LIMITS['overshoot']:g}칸 안 (한 칸 {STEP_DEG:.3f}°)"
    )
    for name, m in metrics.items():
        lines.append(name)
        for speed, widths in sorted(m.judder.items()):
            if widths:
                worst = max(widths)
                lines.append(
                    f"  떨림 {speed_text(speed)}: 1초 창 {len(widths)}개, 가운데 "
                    f"{statistics.median(widths):.1f}칸, 가장 큰 {steps_text(worst)} "
                    f"{verdict(worst, LIMITS['judder'])}"
                )
        for speed, lags in sorted(m.lag.items()):
            if lags:
                lines.append(
                    f"  뒤처짐 {speed_text(speed)}: 가운데 {statistics.median(lags) * STEP_DEG:.2f}°, "
                    f"가장 큰 {max(lags) * STEP_DEG:.2f}°"
                )
        if m.lag_all:
            lines.append(f"  뒤처짐 (모든 움직이기): 가장 큰 {m.lag_all * STEP_DEG:.2f}°")
        if m.holds:
            sag = max(m.holds, key=lambda hold: abs(hold[1]))
            hunting = max(hold[2] for hold in m.holds)
            each = ", ".join(f"{label} {value * STEP_DEG:+.2f}°" for label, value, _ in m.holds)
            lines.append(
                f"  처짐: 버티기 {len(m.holds)}번, 가장 큰 {sag[1] * STEP_DEG:+.2f}° "
                f"({abs(sag[1]):.1f}칸) {verdict(abs(sag[1]), LIMITS['sag'])} [{each}]"
            )
            lines.append(f"  헌팅: 가장 큰 {hunting:.0f}칸 {verdict(hunting, LIMITS['hunting'])}")
        if m.overshoots:
            worst = max(value for _, value in m.overshoots)
            lines.append(
                f"  넘침: {len(m.overshoots)}번, 가장 큰 {worst:.1f}칸 "
                f"{verdict(worst, LIMITS['overshoot'])}"
            )
        parts = []
        if m.temperature:
            start, end, peak = m.temperature
            parts.append(f"온도 {start:.0f} → {end:.0f} °C (1초 가운데 값의 가장 큰 것 {peak:.0f})")
        loads = []
        if m.load_moving:
            loads.append(f"움직일 때 {statistics.fmean(m.load_moving) / 10:.1f}%")
        if m.load_holding:
            loads.append(f"버틸 때 {statistics.fmean(m.load_holding) / 10:.1f}%")
        loads.append(f"가장 큰 {m.load_peak / 10:.1f}%")
        parts.append("부하: " + ", ".join(loads))
        lines.append("  " + " | ".join(parts))
    return lines


def summary(m: JointMetrics) -> dict[str, Any]:
    """견주기와 JSON에 쓰는 관절 하나의 값 (칸, 처짐은 °도 같이)."""
    out: dict[str, Any] = {
        "judder_max": {str(s): max(w) for s, w in sorted(m.judder.items()) if w},
        "judder_median": {str(s): statistics.median(w) for s, w in sorted(m.judder.items()) if w},
        "lag_max_deg": {str(s): max(v) * STEP_DEG for s, v in sorted(m.lag.items()) if v},
        "lag_all_max_deg": m.lag_all * STEP_DEG,
        "holds": [{"step": label, "sag": sag, "hunting": hunt} for label, sag, hunt in m.holds],
        "overshoots": [{"step": label, "overshoot": value} for label, value in m.overshoots],
        "load_peak_percent": m.load_peak / 10,
    }
    if m.holds:
        out["sag_max"] = max((abs(sag) for _, sag, _ in m.holds), default=0.0)
        out["hunting_max"] = max(hunt for _, _, hunt in m.holds)
    if m.overshoots:
        out["overshoot_max"] = max(value for _, value in m.overshoots)
    if m.temperature:
        start, end, peak = m.temperature
        out["temperature_c"] = {"start": start, "end": end, "peak": peak}
    if m.load_moving:
        out["load_moving_percent"] = statistics.fmean(m.load_moving) / 10
    if m.load_holding:
        out["load_holding_percent"] = statistics.fmean(m.load_holding) / 10
    return out


def compare(records: list[Record], summaries: list[dict[str, dict[str, Any]]]) -> list[str]:
    """첫 기록을 기준으로 관절마다 지표를 한 줄에 견준다."""
    lines = ["== 견주기 (R1이 기준)"]
    for k, record in enumerate(records, 1):
        lines.append(f"R{k} {record.folder.name}: {trial_text(record.meta)}")
    names: list[str] = []
    for found in summaries:
        names.extend(name for name in found if name not in names)

    def row(name: str, title: str, cell: Callable[[dict[str, Any]], str | None]) -> None:
        cells = []
        for k, found in enumerate(summaries, 1):
            value = cell(found[name]) if name in found else None
            cells.append(f"R{k} {'-' if value is None else value}")
        if any(not c.endswith(" -") for c in cells):
            lines.append(f"{name} {title}: " + " | ".join(cells))

    def judged(key: str, limit: float, digits: int) -> Callable[[dict[str, Any]], str | None]:
        def cell(v: dict[str, Any]) -> str | None:
            return f"{v[key]:.{digits}f}칸 {verdict(v[key], limit)}" if key in v else None

        return cell

    def per_speed(
        key: str, speed: str, text: str, limit: float | None = None
    ) -> Callable[[dict[str, Any]], str | None]:
        def cell(v: dict[str, Any]) -> str | None:
            value = v[key].get(speed)
            if value is None:
                return None
            shown = text.format(value)
            return shown if limit is None else f"{shown} {verdict(value, limit)}"

        return cell

    for name in names:
        speeds = sorted(
            {s for found in summaries if name in found for s in found[name]["judder_max"]},
            key=float,
        )
        for speed in speeds:
            judder = per_speed("judder_max", speed, "{:.1f}칸", LIMITS["judder"])
            row(name, f"떨림 가장 큰 {float(speed):g}°/s", judder)
        for speed in speeds:
            lag = per_speed("lag_max_deg", speed, "{:.2f}°")
            row(name, f"뒤처짐 가장 큰 {float(speed):g}°/s", lag)
        row(
            name,
            "뒤처짐 가장 큰 (모든 움직이기)",
            lambda v: f"{v['lag_all_max_deg']:.2f}°" if v["lag_all_max_deg"] else None,
        )
        row(name, "처짐 가장 큰", judged("sag_max", LIMITS["sag"], 1))
        row(name, "헌팅 가장 큰", judged("hunting_max", LIMITS["hunting"], 0))
        row(name, "넘침 가장 큰", judged("overshoot_max", LIMITS["overshoot"], 1))
        row(
            name,
            "온도 오름",
            lambda v: (
                f"{v['temperature_c']['end'] - v['temperature_c']['start']:+.0f} °C"
                if "temperature_c" in v
                else None
            ),
        )
        row(
            name,
            "부하 움직일 때",
            lambda v: f"{v['load_moving_percent']:.1f}%" if "load_moving_percent" in v else None,
        )
        row(
            name,
            "부하 버틸 때",
            lambda v: f"{v['load_holding_percent']:.1f}%" if "load_holding_percent" in v else None,
        )
        row(name, "부하 가장 큰", lambda v: f"{v['load_peak_percent']:.1f}%")
    return lines


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="실행 기록의 떨림·처짐 지표 (0042)")
    parser.add_argument("records", nargs="+", type=Path, help="records/run/의 기록 폴더")
    parser.add_argument("--json", type=Path, help="지표를 이 파일에 JSON으로 쓴다")
    args = parser.parse_args(argv)
    records = [load_record(folder) for folder in args.records]
    summaries: list[dict[str, dict[str, Any]]] = []
    lines: list[str] = []
    for record in records:
        metrics = analyze(record)
        lines.extend(report(record, metrics))
        summaries.append({name: summary(m) for name, m in metrics.items()})
    if len(records) > 1:
        lines.extend(compare(records, summaries))
    print("\n".join(lines))
    if args.json:
        found = {
            record.folder.name: {
                "script_name": record.meta.get("script_name"),
                "trial": record.meta.get("trial"),
                "servo_values": record.meta.get("servo_values"),
                "joints": summary_of,
            }
            for record, summary_of in zip(records, summaries, strict=True)
        }
        args.json.write_text(json.dumps(found, ensure_ascii=False, indent=2), encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
