"""손목 D405 받침과 손목 몸통의 돋움: 뽑을 수 있는 형상인지, 겹치지 않는지, J6를 돌릴 때 닿지 않는지."""

from __future__ import annotations

import pytest

from horilab_design.parts import camera_mount as cm
from horilab_design.parts import joint as j
from horilab_design.parts import wrist
from horilab_design.printcheck import (
    LAYER,
    air_walls,
    mid_layer_levels,
    overhangs,
    printed_mass_g,
    wall_settings,
)
from horilab_design.studies import ur_arm


@pytest.fixture(scope="module")
def parts() -> list[j.Part]:
    return cm.assembly("STS3215")


@pytest.mark.slow
def test_mount_is_a_single_valid_solid_printed_on_its_side() -> None:
    printed = cm.printed()
    assert set(cm.LABELS) == set(printed)  # 출력 안내의 한글 이름
    shape, up, _color, _preset = printed["d405_mount"]
    assert shape.is_valid
    assert len(shape.solids()) == 1
    # 옆으로 눕혀 뽑으면 걸치는 면은 가로 구멍 넷의 윗변뿐이다
    over = overhangs(shape, up)
    assert over.total < 60.0, (over.total, over.flat)


@pytest.mark.slow
def test_camera_and_mount_do_not_overlap_the_wrist(parts: list[j.Part]) -> None:
    solids = [p for p in parts if "핀" not in p.name and "손잡이" not in p.name]
    assert j.overlaps(solids) == {}
    cam = next(p for p in parts if p.name == "D405")
    body = next(p for p in parts if p.name == "손목 몸통")
    assert cam.shape.distance_to(body.shape) == pytest.approx(cm.DRUM_GAP, abs=0.05)
    # 받침의 발은 돋움 면에 앉는다 (틈 없이 닿는다)
    mount = next(p for p in parts if p.name == "D405 받침")
    assert mount.shape.distance_to(body.shape) < 0.01


@pytest.mark.slow
def test_plug_fits_between_the_camera_and_the_drum(parts: list[j.Part]) -> None:
    # 커넥터가 있는 옆면이 원통 쪽을 본다: ㄱ자 플러그가 카메라와 원통 사이에 들고 PLUG_GAP이 남는다
    plug = next(p for p in parts if p.name == "D405 플러그")
    cam = next(p for p in parts if p.name == "D405")
    assert plug.shape.distance_to(cam.shape) < 0.01
    near = cm.plug_clearance(parts)
    assert min(near.values()) >= cm.PLUG_GAP, near
    assert near["손목 몸통"] < cm.PLUG_GAP + 1.0  # 카메라를 더 밖으로 내지 않았다
    # 선은 플러그의 J5 쪽(-x) 끝에서 나와 바깥(J5 축에서 멀어지는 쪽)으로 한 번 굽어, 카메라 바닥면 밑의 잡는 자리로 간다
    run = cm.wire_run()
    assert run[1][0] < run[0][0] and run[0][0] == pytest.approx(cm.to_servo(cm.wire_exit())[0])
    assert run[-1][2] == pytest.approx(-cm.HOLD_R) and run[-1][0] < -cm.CAM_H / 2 - cm.WIRE_D / 2
    assert run[-1][0] - wrist.X_FOOT == pytest.approx(cm.HOLD_H) and cm.HOLD_H > 5.0
    # 층으로 넘기는 길(안 될 때의 길)은 받침의 발 밑으로 돌아 뒷덮개 가운데에서 끝난다
    layer = cm.wire_run_layer()
    assert all(p[0] <= -cm.MOUNT_W / 2 - cm.WIRE_D / 2 - 1.0 for p in layer[2:])
    assert layer[-1][2] == 0.0 and layer[-1][1] == pytest.approx(j.BACK_OUT - cm.WIRE_D / 2)


@pytest.mark.slow
def test_turning_parts_clear_the_camera_by_2mm(parts: list[j.Part]) -> None:
    # J6를 멈춤 각까지 돌려도 플랜지 원판, 멈춤 기둥의 나사와 너트, 도구 어댑터가 카메라와 받침에서 2 mm 넘게 뜬다
    gaps = cm.sweep_gaps(parts)
    assert min(gaps.values()) >= 2.0, gaps


def test_screws_engage_and_do_not_bottom() -> None:
    c = cm.screw_checks()
    # 카메라 뒷면 구멍은 나사가 4.0까지만 들어간다
    assert 2.5 <= c["카메라 나사가 카메라에 들어간 길이"] <= cm.CAM_HOLE_DEPTH - 0.5
    assert c["받침 나사가 너트(2.4)에 물린 길이"] == pytest.approx(j.NUT_T)
    assert c["받침 나사 끝과 나사 구멍 바닥 사이"] >= 0.5
    # 돋움의 나사 구멍은 원통 벽에 닿지 않는다
    assert c["돋움 나사 구멍 바닥과 원통 옆면 사이"] >= 1.0


def test_plate_keeps_the_camera_within_half_a_pixel() -> None:
    # 카메라 무게에 판이 휘어 기우는 각이, 탄성 계수가 낮은 쪽(2.0 GPa)이어도 화소 반 칸보다 작다
    assert cm.tilt_under_weight(2.0) < cm.PIXEL_DEG / 2


def test_bosses_stay_clear_of_the_cover_rim_and_the_lock_screw() -> None:
    # 돋움은 뒷덮개 테(원통 뒤 끝을 2.5 겹친다)보다 앞에 있다
    assert wrist.MOUNT_Y[0] > j.BACK_END + j.COVER_SKIRT + 0.5
    # 서보를 잠그는 벽 나사(x = 0, 머리 반지름 2.85)와 옆으로 1 mm 넘게 떨어져 있다
    assert wrist.MOUNT_X - wrist.MOUNT_HW > 2.85 + 1.0
    # 너트 홈이 돋움 안에 있다: 양옆 벽 1.5 넘게, 바닥 1.0 넘게
    w, _t, _roof = wrist.MOUNT_NUT
    assert wrist.MOUNT_HW - w / 2 >= 1.5
    assert wrist.MOUNT_Y[1] - (wrist.MOUNT_SCREW_Y + w / 2 / 0.866) >= 1.0
    # 받침의 발 나사 머리가 발과 판 사이를 메운 자리에 걸리지 않는다
    shape = cm.mount()
    for x, y in wrist.mount_xy():
        head_edge = (x, y + 2.85 + 0.2, -(wrist.MOUNT_Z + cm.FOOT_T) - 0.5)
        assert not shape.solids()[0].is_inside(head_edge)


@pytest.mark.slow
def test_arm_model_uses_the_mount_masses() -> None:
    shape, up, _color, preset = cm.printed()["d405_mount"]
    total = printed_mass_g(shape, up, preset) + sum(p.mass_g for p in cm.hardware())
    assert total == pytest.approx(ur_arm.D405_PLATE_MASS * 1000, abs=0.5)


@pytest.mark.slow
def test_clip_holds_the_wire_just_below_the_plug(parts: list[j.Part]) -> None:
    start, full, end = cm.clip_span()
    plug_end = cm.CAM_H / 2 - cm.PLUG_TOP - cm.PLUG_LEN
    # 클립은 플러그 몸통 밑에서 시작해 고무 끝을 지나 솟고, 고무 끝에서 15 mm 안에서 끝난다
    assert start > plug_end > full > end
    assert plug_end - end <= 15.0
    assert cm.clip_grip()["갈래가 선의 옆에 닿는 길이 [mm]"] >= 6.0
    # 끝 면은 뽑을 때 층 경계에 온다 (+x 쪽 옆면이 바닥이다)
    height = cm.MOUNT_W / 2 - end
    assert abs(height / LAYER - round(height / LAYER)) < 1e-6
    # 두 갈래 사이가 선 지름보다 좁고(문다), 입구는 그보다 더 좁다 (선이 저절로 빠지지 않는다)
    outline = cm.clip_outline()
    grip = min(abs(q) for q, p in outline if abs(p - cm.CONN_BACK) < 0.75 and abs(q) > 1.0)
    mouth = min(abs(q) for q, p in outline if p > cm.CONN_BACK + 0.8)
    assert cm.WIRE_D / 2 - 0.25 <= grip < cm.WIRE_D / 2
    assert mouth < grip
    # 끼울 때 갈래 뿌리의 변형률이 2% 안이다 (PLA가 금 가지 않는다)
    assert cm.clip_grip()["그때 갈래 뿌리의 변형률 [%]"] < 2.0
    # 클립은 카메라, 플러그, 손목 몸통과 떨어져 있다
    around = cm.clip_clearance(parts)
    assert min(around.values()) >= cm.PLUG_GAP, around
    # 받침은 +x 쪽 옆면을 바닥에 두고 뽑아 클립이 맨 위에 온다. 막는 공중 벽 선과 층 가운데의 수평면이 없다
    shape, up, _color, preset = cm.printed()["d405_mount"]
    assert up == (-1.0, 0.0, 0.0)
    assert [a for a in air_walls(shape, up, wall_settings(preset)) if a.blocking] == []
    assert mid_layer_levels(shape, up) == []


def test_plug_sits_where_it_was_measured_and_the_clip_is_left_off() -> None:
    # 플러그 몸통의 위끝은 카메라 윗면에서 11.74 mm다 (잰 값). 고무 끝이 카메라 바닥면보다 2.0 mm 위, 받침의 J5 쪽
    # 면보다 2.0 mm 밖이다
    assert cm.PLUG_TOP == pytest.approx(11.74)
    plug_end = cm.CAM_H / 2 - cm.PLUG_TOP - cm.PLUG_LEN
    assert plug_end == pytest.approx(-cm.CAM_H / 2 + 1.96, abs=0.01)
    assert plug_end < -cm.MOUNT_W / 2 - 2.0
    # 그래서 플러그를 따라 그린 클립은 판에서 솟지 못한다 (판 밖에서 시작한다). 받침에는 클립이 없다
    assert cm.clip_span()[0] < -cm.MOUNT_W / 2 and not cm.clip_rooted()
    shape = cm.printed()["d405_mount"][0]
    assert shape.bounding_box().min.X == pytest.approx(-cm.MOUNT_W / 2, abs=0.01)
    assert any("클립" in text for text in cm.PROVISIONAL)


def test_clip_follows_the_plug(monkeypatch: pytest.MonkeyPatch) -> None:
    # 플러그가 붙는 자리(PLUG_TOP)를 바꾸면 클립이 그만큼 따라간다. 사진으로 어림하던 자리(7 mm)에서는 판에서 솟는다
    monkeypatch.setattr(cm, "PLUG_TOP", 7.0)
    assert cm.clip_rooted()
    base = cm.clip_span()
    monkeypatch.setattr(cm, "PLUG_TOP", cm.PLUG_TOP + 2.0)
    moved = cm.clip_span()
    assert moved[0] == pytest.approx(base[0] - 2.0) and moved[1] == pytest.approx(base[1] - 2.0)
    assert moved[2] == pytest.approx(base[2] - 2.0, abs=LAYER)
