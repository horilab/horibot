"""시뮬레이션 서버: 서보 거래를 받고, 동기 읽기마다 한 주기만큼 물리를 진행한다 (0033)."""

from __future__ import annotations

import json
import math
import socket
import threading
from collections.abc import Callable, Iterator
from pathlib import Path
from typing import Any

import pytest

from horibot_sim.server import Server
from horibot_sim.world import RAD_PER_STEP

Ask = Callable[..., dict[str, Any]]
STATE = ["torque_enable", "goal_position", "torque_limit", "present_position", "present_load"]
TILT = 2048 + round(0.6 / RAD_PER_STEP)  # J2를 0.6 rad 기울인 자리

# 시험용 팔: 위로 선 막대 셋, 관절 j2·j3·j4가 모두 y축으로 돈다. j4는 호리봇의 J4처럼 범위(20° ~ 340°)가 0°를 품지
# 않는다 (j4에는 처음 세계에서 서보를 달지 않는다)
MODEL = """<mujoco model="test">
  <compiler angle="radian"/>
  <worldbody>
    <body name="link1" pos="0 0 0.1">
      <joint name="j2" type="hinge" axis="0 1 0" range="-2 2" limited="true"/>
      <geom type="capsule" fromto="0 0 0 0 0 0.22" size="0.012"/>
      <body name="link2" pos="0 0 0.22">
        <joint name="j3" type="hinge" axis="0 1 0" range="-2 2" limited="true"/>
        <geom type="capsule" fromto="0 0 0 0 0 0.18" size="0.012"/>
        <body name="link3" pos="0 0 0.18">
          <joint name="j4" type="hinge" axis="0 1 0" range="0.349066 5.934119" limited="true"/>
          <geom type="capsule" fromto="0 0 0 0 0 0.1" size="0.012"/>
        </body>
      </body>
    </body>
  </worldbody>
</mujoco>
"""
SERVO_SPECS = "[STS3250]\nstall_torque_kgcm = 50\n"


@pytest.fixture
def ask(tmp_path: Path) -> Iterator[Ask]:
    model, specs = tmp_path / "arm.xml", tmp_path / "servos.toml"
    model.write_text(MODEL, encoding="utf-8")
    specs.write_text(SERVO_SPECS, encoding="utf-8")
    server = Server(("127.0.0.1", 0), model, specs)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    connection = socket.create_connection(("127.0.0.1", server.server_address[1]), timeout=30)
    stream = connection.makefile("rw", encoding="utf-8")

    def request(**body: Any) -> dict[str, Any]:
        stream.write(json.dumps(body) + "\n")
        stream.flush()
        return json.loads(stream.readline())

    hello = request(
        op="hello",
        period=0.01,
        servos=[
            {"id": 2, "joint": "j2", "model": 2825, "direction": 1},
            {"id": 3, "joint": "j3", "model": 2825, "direction": 1},
        ],
    )
    assert hello["ok"], hello
    yield request
    # 스트림까지 닫아야 연결이 끊겨, 서버가 다음 줄을 기다리지 않고 끝난다
    stream.close()
    connection.close()
    server.shutdown()
    server.server_close()


def run(ask: Ask, beats: int) -> dict[str, Any]:
    """박자만큼 동기 읽기를 보내고 마지막 답을 돌려준다."""
    reply: dict[str, Any] = {}
    for _ in range(beats):
        reply = ask(op="sync_read", ids=[2, 3], registers=STATE)
    return reply


def enable(ask: Ask, servo_id: int, goal: int) -> None:
    """시작 절차: 목표를 쓰고 토크를 켠다."""
    assert ask(op="write", id=servo_id, register="goal_position", value=goal)["ok"]
    assert ask(op="write", id=servo_id, register="torque_enable", value=1)["ok"]


def test_servos_answer_like_after_power_on(ask: Ask) -> None:
    assert ask(op="ping", id=2) == {"ok": True, "model": 2825}
    assert ask(op="ping", id=9)["error"] == "no_reply"
    values = ask(op="read", id=2, registers=STATE)["values"]
    # 전원 직후: 토크 꺼짐, 목표 0, 토크 한도 최대. 영점 자세(팔이 곧게 섬)는 2048로 읽힌다
    assert values == {
        "torque_enable": 0,
        "goal_position": 0,
        "torque_limit": 1000,
        "present_position": 2048,
        "present_load": 0,
    }


def test_each_sync_read_advances_one_period(ask: Ask) -> None:
    reply: dict[str, Any] = {}
    for beat in range(1, 11):
        reply = ask(op="sync_read", ids=[2, 3, 9], registers=STATE)
        assert math.isclose(reply["time"], beat * 0.01, abs_tol=1e-9)
    assert reply["replies"]["9"] == {"error": "no_reply"}


def test_position_servo_holds_a_tilted_arm_with_load(ask: Ask) -> None:
    enable(ask, 3, 2048)
    enable(ask, 2, TILT)
    replies = run(ask, 300)["replies"]
    j2 = replies["2"]["values"]
    assert abs(j2["present_position"] - TILT) <= 10, j2  # 짐에 조금 처진 채 버틴다
    assert abs(j2["present_load"]) > 10, j2  # 팔의 무게를 버티는 힘


def test_arm_falls_when_torque_is_off_or_too_weak(ask: Ask) -> None:
    enable(ask, 3, 2048)
    enable(ask, 2, TILT)
    run(ask, 300)
    ask(op="write", id=2, register="torque_limit", value=5)  # 0.5 %로는 못 버틴다
    weak = run(ask, 100)["replies"]["2"]["values"]["present_position"]
    assert abs(weak - TILT) > 50
    ask(op="write", id=2, register="torque_enable", value=0)
    before = ask(op="state")["angles"]["j2"]
    after = (run(ask, 100), ask(op="state")["angles"]["j2"])[1]
    assert abs(after - before) > 0.05 or abs(after) > 1.0  # 힘이 없으면 계속 떨어진다


def test_power_cycle_resets_ram(ask: Ask) -> None:
    enable(ask, 2, TILT)
    ask(op="write_trial", id=2, values=[["position_offset", 100]])
    ask(op="power_cycle", id=2)
    values = ask(op="read", id=2, registers=["torque_enable", "goal_position", "position_offset"])
    assert values["values"] == {"torque_enable": 0, "goal_position": 0, "position_offset": 0}


def test_servo_reads_2048_at_the_range_middle_and_turns_its_own_way(ask: Ask) -> None:
    # 실물처럼 영점을 재서 범위 가운데가 2048로 읽히고, 로봇 설정의 방향으로 달려 있다
    servo = {"id": 4, "joint": "j4", "model": 2825, "direction": -1}
    assert ask(op="hello", period=0.01, servos=[servo])["ok"]
    middle = (0.349066 + 5.934119) / 2
    assert ask(op="state")["angles"]["j4"] == pytest.approx(middle)  # 범위 가운데에서 시작한다
    assert (
        ask(op="read", id=4, registers=["present_position"])["values"]["present_position"] == 2048
    )
    # 힘 한도에 걸리지 않을 만큼만 옮긴다 (시험 팔의 j4 막대는 50 g이라, 힘이 한도에 걸리면 물리 계산이 떤다)
    enable(ask, 4, 2048 + 20)
    for _ in range(100):
        ask(op="sync_read", ids=[4], registers=STATE)
    # 방향이 -1이라 서보 값이 늘면 관절 각이 준다
    assert ask(op="state")["angles"]["j4"] == pytest.approx(middle - 20 * RAD_PER_STEP, abs=0.003)
    position = ask(op="read", id=4, registers=["present_position"])["values"]["present_position"]
    assert abs(position - (2048 + 20)) <= 2
    # 방향은 1이나 -1이다
    wrong = ask(op="hello", period=0.01, servos=[{**servo, "direction": 0}])
    assert not wrong["ok"]


def test_servo_keeps_the_position_offset_burned_at_zeroing(ask: Ask) -> None:
    # 영점을 잰 서보는 위치 오프셋이 구워져 있다 (로봇 설정의 값). 읽는 위치는 그대로 범위 가운데가 2048이다
    servo = {"id": 4, "joint": "j4", "model": 2825, "direction": -1, "position_offset": -1865}
    assert ask(op="hello", period=0.01, servos=[servo])["ok"]
    values = ask(op="read", id=4, registers=["position_offset", "present_position"])["values"]
    assert values == {"position_offset": -1865, "present_position": 2048}
    enable(ask, 4, 2048 + 20)
    for _ in range(100):
        ask(op="sync_read", ids=[4], registers=STATE)
    middle = (0.349066 + 5.934119) / 2
    assert ask(op="state")["angles"]["j4"] == pytest.approx(middle - 20 * RAD_PER_STEP, abs=0.003)
    # 전원을 껐다 켜도 남는다
    ask(op="power_cycle", id=4)
    offset = ask(op="read", id=4, registers=["position_offset"])["values"]["position_offset"]
    assert offset == -1865
    # 담을 수 없는 값이면 세계를 만들지 않는다
    assert not ask(op="hello", period=0.01, servos=[{**servo, "position_offset": 2048}])["ok"]


def test_world_starts_at_the_given_pose(ask: Ask) -> None:
    # 시험 순서의 시작 자세(예: 쉬는 자세)에서 시작한다. 적지 않은 관절은 범위 가운데다 (0034)
    servos = [{"id": 4, "joint": "j4", "model": 2825, "direction": -1}]
    assert ask(op="hello", period=0.01, servos=servos, start={"j4": 5.6, "j2": 0.3})["ok"]
    angles = ask(op="state")["angles"]
    assert angles["j4"] == pytest.approx(5.6)
    middle = (0.349066 + 5.934119) / 2
    position = ask(op="read", id=4, registers=["present_position"])["values"]["present_position"]
    assert position == 2048 - round((5.6 - middle) / RAD_PER_STEP)  # 방향 -1
    # 범위 밖이거나 모르는 관절이면 세계를 만들지 않는다
    assert not ask(op="hello", period=0.01, servos=servos, start={"j4": 6.1})["ok"]
    assert not ask(op="hello", period=0.01, servos=servos, start={"j9": 0.0})["ok"]
