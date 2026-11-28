"""tools/run_metrics.py 검사 (docs/decisions/0042의 지표): 값을 아는 가짜 실행 기록을 만들어 지표마다 본다."""

import csv
import json
from pathlib import Path

import pytest

from tools import run_metrics as rm

SCRIPT = """name = "t"
joints = ["j1", "j2"]
speed_percent = 50
[[steps]]
move = { j1 = 20.0 }
speed_percent = 25
[[steps]]
hold_s = 5
[[steps]]
move = { j1 = 0.0, j2 = 10.0 }
[[steps]]
hold_s = 3
"""
ROBOT = """[run_loop]
max_speed_deg_s = 30
[joints.j1]
direction = -1
[joints.j2]
direction = 1
"""
START = 2048
MOVING_ROWS = 450  # 4.5초: 떨림 1초 창 넷
RISE = 255  # j1이 오르는 칸 수 (85칸/s, 약 7.5°/s)
TOP = START + RISE


def _square(k: int) -> int:
    """주기 20박자(5 Hz)의 ±1칸 떨림."""
    return 1 if (k // 10) % 2 == 0 else -1


def _rows() -> list[dict[str, int | float | str]]:
    """박자마다 (단계, mode, j1·j2 목표·위치·부하·온도). 목표는 실행 루프가 그 박자에 노린 자리다."""
    rows: list[dict[str, int | float | str]] = []

    def add(
        step: str, mode: int, target1: int, position1: int, target2: int, position2: int, load1: int
    ) -> None:
        k = len(rows)
        rows.append(
            {
                "step": step,
                "mode": mode,
                "j1_target": target1,
                "j1_position": position1,
                "j1_load": load1,
                "j1_temperature": 76 if k % 50 == 7 else (30 if k < 900 else 32),
                "j2_target": target2,
                "j2_position": position2,
                "j2_load": 0,
                "j2_temperature": 30,
            }
        )

    # 1-move: j1만 오른다. 읽은 위치는 지난 박자의 목표보다 10칸 뒤에서 ±1칸 떤다
    add("1-move", 0, START, START, START, START, 0)
    add("1-move", 0, START, START, START, START, 0)
    previous = START
    for n in range(MOVING_ROWS):
        target = START + round(RISE * (n + 1) / MOVING_ROWS)
        add("1-move", 1, target, previous - 10 + _square(len(rows)), START, START, -100)
        previous = target
    add("1-move", 0, TOP, TOP - 4, START, START, 50)
    # 2-hold 5초: 0.3초 동안 3칸 넘어갔다가(그 사이 한 박자는 7칸으로 튄다) 1칸 모자란 자리에 선다. 일곱 박자마다 한
    # 칸 오가고(헌팅 1칸), 한 박자씩 두 번은 3칸 아래로 튀어 읽힌다
    for n in range(500):
        if n < 30:
            position = TOP + (7 if n == 10 else 3)
        elif n in (200, 300):
            position = TOP - 4
        else:
            position = TOP - 1 + (1 if n % 7 == 0 else 0)
        add("2-hold", 0, TOP, position, START, START, 300 if n == 100 else 50)
    # 3-move: 두 관절을 같이 움직인다 (떨림은 재지 않고, 모든 움직이기의 뒤처짐에만 든다)
    previous1, previous2 = TOP, START
    for n in range(200):
        target1 = TOP - round(RISE * (n + 1) / 200)
        target2 = START + round(114 * (n + 1) / 200)
        add("3-move", 1, target1, previous1 + 20, target2, previous2 - 5, -100)
        previous1, previous2 = target1, target2
    add("3-move", 0, START, START, START + 114, START + 114, 0)
    for _ in range(300):
        add("4-hold", 0, START, START, START + 114, START + 114, 0)
    return rows


def _write(folder: Path, with_target: bool = True, trial: dict | None = None) -> Path:
    folder.mkdir(parents=True)
    rows = _rows()
    names = [
        "goal",
        "position",
        "speed",
        "load",
        "temperature",
        "voltage",
        "current",
        "status",
        "torque",
    ]
    if with_target:
        names.append("target")
    header = ["t_s", "step", "mode", "wake_us", "tick_us"]
    for joint in ("j1", "j2"):
        header += [f"{joint}_{name}" for name in names]
    with (folder / "samples.csv").open("w", encoding="utf-8", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        last = {"j1": START, "j2": START}
        for k, row in enumerate(rows):
            line: list[object] = [f"{(k + 1) * 0.01:.3f}", row["step"], row["mode"], 10, 2000]
            for joint in ("j1", "j2"):
                # 서보에서 읽은 목표는 지난 박자에 쓴 것이다
                values = {
                    "goal": last[joint],
                    "position": row[f"{joint}_position"],
                    "speed": 0,
                    "load": row[f"{joint}_load"],
                    "temperature": row[f"{joint}_temperature"],
                    "voltage": 120,
                    "current": 0,
                    "status": 0,
                    "torque": 1,
                    "target": row[f"{joint}_target"],
                }
                line += [values[name] for name in names]
                last[joint] = int(row[f"{joint}_target"])
            writer.writerow(line)
    meta = {
        "kind": "run",
        "script_name": "시험",
        "script": SCRIPT,
        "robot_toml": ROBOT,
        "joints": ["j1", "j2"],
        "end": "done",
        "trial": trial,
        "servo_values": {
            "j1": {"p_gain": 32, "d_gain": 32, "i_gain": 0, "acceleration": 11},
            "j2": {"p_gain": 32, "d_gain": 32, "i_gain": 0, "acceleration": 11},
        },
    }
    (folder / "meta.json").write_text(json.dumps(meta, ensure_ascii=False), encoding="utf-8")
    return folder


def test_metrics_of_a_known_record(tmp_path: Path) -> None:
    record = rm.load_record(_write(tmp_path / "run"))
    assert record.has_target
    metrics = rm.analyze(record)
    j1, j2 = metrics["j1"], metrics["j2"]
    # 떨림: 빠르기는 단계의 speed_percent(25 %) × 30°/s. ±1칸 네모 떨림에서 0.2초 평균을 빼면 폭이 2 + 2/21칸이다
    assert list(j1.judder) == [7.5]
    widths = j1.judder[7.5]
    assert len(widths) == 4
    assert sorted(widths)[1] == pytest.approx(2 + 2 / 21, abs=1e-6)
    assert j2.judder == {}, "j2는 혼자 움직이지 않았다"
    # 뒤처짐: 10칸 뒤에서 ±1칸
    assert max(j1.lag[7.5]) == 11
    assert j1.lag_all == 20, "같이 움직인 이동에서 20칸"
    assert j2.lag_all == 5
    # 처짐: 1칸 모자라고, j1은 방향이 -1이라 관절 각으로는 +1칸. 헌팅은 튄 두 박자를 거르고 1칸
    label, sag, hunting = j1.holds[0]
    assert (label, sag, hunting) == ("2-hold", 1.0, 1.0)
    assert j2.holds[0][1:] == (0.0, 0.0)
    # 넘침: 3칸 넘어갔다 (한 박자 7칸은 거른다). 두 번째 움직이기는 곧바로 선다
    assert j1.overshoots[0] == ("1-move", 3.0)
    assert ("3-move", 0.0) in j1.overshoots
    # 온도: 튄 76 °C는 거르고 30 → 32. 부하: 움직일 때 10 %, 버틸 때 5 %, 가장 큰 30 %
    assert j1.temperature == (30.0, 32.0, 32.0)
    assert sum(j1.load_moving) / len(j1.load_moving) == 100
    assert j1.load_peak == 300
    text = "\n".join(rm.report(record, metrics))
    for expected in [
        "떨림 7.5°/s: 1초 창 4개, 가운데 2.1칸",
        "넘음",
        "처짐: 버티기 2번, 가장 큰 +0.09° (1.0칸) 통과",
        "헌팅: 가장 큰 1칸 통과",
        "넘침: 2번, 가장 큰 3.0칸 넘음",
        "온도 30 → 32 °C",
        "시험 값: 없음 (지금 설정)",
        "서보 값(시험 값을 쓰기 전): j1 P32 D32 I0 가속11",
    ]:
        assert expected in text, f"{expected}:\n{text}"


def test_old_records_use_the_goal_column(tmp_path: Path) -> None:
    """target 칸이 없는 기록(2026-11-28까지)은 서보에서 읽은 목표로 같은 지표를 낸다 (처짐 보정이 꺼져 있었다)."""
    new = rm.analyze(rm.load_record(_write(tmp_path / "new")))
    old_record = rm.load_record(_write(tmp_path / "old", with_target=False))
    assert not old_record.has_target
    old = rm.analyze(old_record)
    for name in ("j1", "j2"):
        assert rm.summary(old[name]) == rm.summary(new[name]), name
    assert "goal 칸" in "\n".join(rm.report(old_record, old))


def test_runs_are_compared_against_the_first(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    trial = {
        "asked": {"servos": {"j1": {"p_gain": 48}}, "sag_gain_per_s": 2.0},
        "written": {"j1": {"p_gain": {"before": 32, "trial": 48}}},
        "restored": True,
    }
    base = _write(tmp_path / "base")
    tried = _write(tmp_path / "tried", trial=trial)
    out = tmp_path / "metrics.json"
    assert rm.main([str(base), str(tried), "--json", str(out)]) == 0
    text = capsys.readouterr().out
    assert "== 견주기 (R1이 기준)" in text
    assert "R2 tried: j1 p_gain 32→48 | 처짐 보정 이득 2/s" in text
    judder = next(line for line in text.splitlines() if line.startswith("j1 떨림 가장 큰 7.5°/s"))
    assert judder.count("넘음") == 2 and "| R2 " in judder, judder
    assert "j1 처짐 가장 큰: R1 1.0칸 통과 | R2 1.0칸 통과" in text
    assert "j2 처짐 가장 큰: R1 0.0칸 통과" in text
    found = json.loads(out.read_text(encoding="utf-8"))
    assert found["tried"]["trial"]["written"]["j1"]["p_gain"]["trial"] == 48
    assert found["base"]["joints"]["j1"]["overshoot_max"] == 3.0
