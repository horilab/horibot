"""값 파일(TOML)을 읽어 검사하고, 로봇 기술(SI)로 바꾼다 (docs/decisions/0006).

- params/arm.toml: 팔의 설계 값 (명목 치수, 관절 범위, 관절마다 서보)
- params/servos.toml: 서보 모델의 명목 사양
- data/measured.toml: 실측 값. 설계 값을 덮어쓰지 않고 따로 둔다

파일의 단위는 키 이름에 붙은 mm, deg, g, kgcm이고, 로봇 기술로 바꿀 때 SI로 바꾼다.
키 이름이 틀리면(단위를 빠뜨리는 등) 읽을 때 바로 실패한다.
"""

from __future__ import annotations

import tomllib
from pathlib import Path
from typing import Any, Literal

import numpy as np
from pydantic import BaseModel, ConfigDict, Field, TypeAdapter, model_validator

from horilab_design.robot import Joint, Robot, Vec3, rotation

REPO_ROOT = Path(__file__).resolve().parents[4]
ARM_FILE = REPO_ROOT / "params" / "arm.toml"
SERVOS_FILE = REPO_ROOT / "params" / "servos.toml"
MEASURED_FILE = REPO_ROOT / "data" / "measured.toml"

KGCM = 0.0980665  # 1 kg·cm [N·m]
JOINT_NAMES = ("j1", "j2", "j3", "j4", "j5", "j6")
X: Vec3 = (1.0, 0.0, 0.0)
Y: Vec3 = (0.0, 1.0, 0.0)
Z: Vec3 = (0.0, 0.0, 1.0)

ServoModel = Literal["STS3215", "STS3250", "ST-3120"]


class _Values(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)


class ServoSpec(_Values):
    rated_torque_kgcm: float = Field(gt=0)
    stall_torque_kgcm: float = Field(gt=0)
    no_load_speed_s_per_60deg: float = Field(gt=0)
    stall_current_a: float = Field(gt=0)
    mass_g: float = Field(gt=0)
    body_length_mm: float = Field(gt=0)  # 출력축에 수직인 단면의 긴 변
    body_width_mm: float = Field(gt=0)
    shaft_to_end_mm: float = Field(gt=0)  # 출력축에서 길이 방향으로 가까운 끝까지
    horn_diameter_mm: float = Field(gt=0)
    horn_bolt_circle_mm: float = Field(gt=0)
    horn_bolts: int = Field(gt=0)  # 혼의 M3 구멍 수

    @property
    def rated_torque(self) -> float:
        """정격 토크 [N·m]. 토크 여유는 최대 토크가 아니라 이 값으로 본다 (docs/hardware.md)."""
        return self.rated_torque_kgcm * KGCM

    @property
    def max_speed(self) -> float:
        """무부하 속도 [rad/s]."""
        return float(np.pi / 3 / self.no_load_speed_s_per_60deg)


class ArmGeometry(_Values):
    shoulder_height_mm: float = Field(gt=0)
    upper_arm_mm: float = Field(gt=0)
    forearm_mm: float = Field(gt=0)
    j4_to_j5_mm: float = Field(ge=0)
    j5_to_j6_mm: float = Field(ge=0)
    j6_to_flange_mm: float = Field(ge=0)


class JointParams(_Values):
    servo: ServoModel
    range_deg: tuple[float, float]

    @model_validator(mode="after")
    def _one_turn(self) -> JointParams:
        lower, upper = self.range_deg
        if not lower < upper:
            raise ValueError(f"관절 범위는 아래 < 위여야 한다: {self.range_deg}")
        if upper - lower > 360:
            raise ValueError(f"서보 위치 모드는 한 바퀴(360°) 안이다: {self.range_deg}")
        return self


class ArmParams(_Values):
    arm: ArmGeometry
    joints: dict[str, JointParams]

    @model_validator(mode="after")
    def _all_joints(self) -> ArmParams:
        if sorted(self.joints) != list(JOINT_NAMES):
            raise ValueError(f"관절은 {JOINT_NAMES}여야 한다: {sorted(self.joints)}")
        return self


Filament = Literal["pla_basic_white", "pla_basic_black"]


class PrintFit(_Values):
    """공차 시험편으로 고른 설계 치수 [mm]. 필라멘트마다 하나다 (docs/manufacturing.md의 "공차")."""

    m3_hole_mm: float | None = None  # 세로 구멍: M3 볼트가 걸리지 않고 지나가는 가장 작은 지름
    m3_hole_horizontal_mm: float | None = None  # 가로 구멍(둥근 구멍)의 지름
    m3_tap_hole_mm: float | None = None  # M3 볼트를 플라스틱에 바로 조여 넣는 구멍
    m3_nut_pocket_mm: float | None = None  # 위에서 눌러 끼우는 육각 홈의 맞변 거리
    m3_nut_slot_width_mm: float | None = None  # 너트가 누워 옆에서 밀려 들어가는 홈의 폭
    m3_nut_slot_height_mm: float | None = None  # 그 홈의 높이 (볼트 방향, 세로)
    # 너트가 서는 홈 (볼트가 가로). 뽑히는 방향이 달라 누운 너트 값을 쓰지 않는다
    m3_nut_upright_width_mm: float | None = (
        None  # 옆에서 밀어 넣는 홈의 세로 폭 (천장이 허공에 걸친다)
    )
    m3_nut_upright_thickness_mm: float | None = None  # 그 홈의 두께 (볼트 방향, 가로)
    m3_nut_drop_width_mm: float | None = None  # 윗면에서 떨어뜨려 넣는 홈의 폭 (가로)
    m3_nut_drop_thickness_mm: float | None = None  # 그 홈의 두께 (볼트 방향, 가로)
    # 서보 몸체 자리와 혼 자리: 도면 치수(params/servos.toml)에 더하는 여유. 길이·폭·지름에 그대로 더한다
    seat_clearance_mm: dict[ServoModel, float] = {}
    horn_clearance_mm: dict[ServoModel, float] = {}
    # 3차 시험편: 영점 핀(MSH3-20), 6806 자리와 축 (설계 지름)
    pin_hole_mm: float | None = (
        None  # 세로 구멍: 핀이 손으로 부드럽게 들어가고 빠지며 흔들어도 놀지 않는 가장 작은 지름
    )
    pin_hole_horizontal_mm: float | None = None  # 가로 구멍(둥근 구멍)의 지름
    bearing_6806_bore_mm: float | None = None  # 바깥 링을 손으로 밀어 넣을 수 있고 놀지 않는 구멍
    bearing_6806_shaft_mm: float | None = None  # 안쪽 링을 손으로 끼울 수 있고 놀지 않는 축
    bearing_6808_bore_mm: float | None = None  # 6808 바깥 링 자리 (6806과 같은 규칙)
    bearing_6808_shaft_mm: float | None = None  # 6808 안쪽 링 축 (6806과 같은 규칙)
    pin_press_mm: float | None = None  # 세로 구멍: 도구 핀을 눌러 박아 손으로 안 빠지는 지름
    # 큰 세로 구멍: 위팔 관 끝 고리(설계 지름 34.8)를 세로로 뽑았을 때의 실제 안지름 (받침 기둥 위끝 고리로 잰다)
    tube_bore_mm: float | None = None


class ServoBody(_Values):
    """캘리퍼스로 잰 서보 몸통 치수 [mm] (docs/hardware.md의 "기구 치수")."""

    far_thickness_mm: float = Field(gt=0)  # 라벨 쪽 끝 두께: 앞면 ~ 뒷면의 볼록한 판
    horn_to_back_mm: float = Field(gt=0)  # 출력 혼 윗면 ~ 출력축 둘레의 낮은 뒷면 (두께 29인 면)
    horn_to_wire_mm: float | None = Field(default=None, gt=0)  # 출력 혼 윗면 ~ 뒷면에 꽂아 눕힌 선
    horn_to_boss_mm: float | None = Field(default=None, gt=0)  # 출력 혼 윗면 ~ 뒷면 보스 끝
    # ST-3120 (혼 테의 가운데가 솟아 있고, 뒷면 보스에 베어링이 남고, 커넥터가 옆면에 있다)
    horn_boss_to_back_mm: float | None = Field(default=None, gt=0)  # 혼 가운데 돋움 윗면 ~ 뒷면
    # 혼 가운데 나사 머리 ~ 뒷면 보스 끝
    horn_screw_to_back_boss_mm: float | None = Field(default=None, gt=0)
    back_boss_mm: float | None = Field(default=None, gt=0)  # 뒷면 ~ 보스 끝 (남은 베어링의 면)
    back_bearing_diameter_mm: float | None = Field(default=None, gt=0)  # 보스에 남은 베어링
    # 옆면 커넥터에 꽂은 플러그: 혼 쪽 끝 면 ~ 플러그의 먼 쪽 면, 뒷면 ~ 플러그의 앞쪽 면
    side_plug_far_mm: float | None = Field(default=None, gt=0)
    side_plug_front_mm: float | None = Field(default=None, gt=0)
    # 혼 가운데 돋움과 나사 머리의 지름, 옆면 ~ 꽂은 플러그의 바깥 면, 옆면 ~ 꺾어 몸체를 따라 눕힌 선의 가장 바깥
    horn_boss_diameter_mm: float | None = Field(default=None, gt=0)
    horn_screw_head_diameter_mm: float | None = Field(default=None, gt=0)
    side_plug_out_mm: float | None = Field(default=None, gt=0)
    side_wire_out_mm: float | None = Field(default=None, gt=0)


class ServoDepth(_Values):
    """깊이 막대로 잰 STS 서보의 단차 [mm]. 값이 오르내려서 (작은 값, 큰 값)이다."""

    horn_above_label_mm: tuple[float, float]  # 출력 혼 윗면 ~ 앞면 라벨 판
    horn_screw_head_mm: tuple[float, float]  # 혼 가운데 나사 머리가 혼 윗면 위로 나온 높이
    horn_screw_head_diameter_mm: float = Field(gt=0)

    @model_validator(mode="after")
    def _ordered(self) -> ServoDepth:
        for low, high in (self.horn_above_label_mm, self.horn_screw_head_mm):
            if not 0 < low <= high:
                raise ValueError(f"(작은 값, 큰 값)이어야 한다: {(low, high)}")
        return self


class ServoWire(_Values):
    """서보 선 한 줄 [mm]: 세 가닥을 나란히 놓은 폭과 두께."""

    width_mm: float = Field(gt=0)
    thickness_mm: float = Field(gt=0)


class BoughtServoWire(_Values):
    """따로 산 서보 선(Waveshare, 5264 3핀)의 선 한 줄. 길이는 상품 이름의 값이다."""

    mass_300_g: float = Field(gt=0)  # 300 mm 선 하나 (양쪽 플러그 포함)
    mass_900_g: float = Field(gt=0)  # 900 mm 선 하나 (양쪽 플러그 포함)
    width_mm: float | None = Field(default=None, gt=0)  # 세 가닥을 나란히 놓은 폭


class Cable(_Values):
    """둥근 선 [mm]."""

    diameter_mm: float = Field(gt=0)


class ThinCable(_Values):
    """재는 중인 선 [mm]: 길이부터 쟀고 굵기는 재면 채운다."""

    length_mm: float = Field(gt=0)  # 플러그 끝에서 끝까지
    diameter_mm: float | None = Field(default=None, gt=0)
    mass_g: float | None = Field(default=None, gt=0)  # 선 전체 (양쪽 플러그 포함)


class AnglePlug(_Values):
    """카메라에 꽂은 ㄱ자 플러그 [mm]."""

    protrusion_mm: float = Field(gt=0)  # 꽂은 채 카메라 옆면 ~ 플러그 가장 바깥
    body_length_mm: float = Field(gt=0)  # 선 방향: 꺾인 몸통 끝 ~ 고무 끝
    body_out_mm: float = Field(gt=0)  # 꽂는 방향(옆면에서 바깥쪽)의 몸통 두께
    body_across_mm: float = Field(gt=0)  # 위 두 방향과 직각인 몸통 폭
    top_mm: float | None = Field(
        default=None, gt=0
    )  # 카메라 윗면 ~ 몸통 위끝 (몸통이 옆면에 붙는 자리)


class BackScrew(_Values):
    """STS3215 뒷면 고정 나사 (예전 SO-101에서 뺀 십자 둥근머리 자가탭) [mm]."""

    total_length_mm: float = Field(gt=0)  # 머리 윗면 ~ 끝


class ServoPlug(_Values):
    """서보 선 플러그(5264 3핀) 몸통 치수 [mm]."""

    length_mm: float = Field(gt=0)  # 꽂는 방향
    width_mm: float = Field(gt=0)
    thickness_mm: float = Field(gt=0)


class PiCase(_Values):
    """적층 아크릴 케이스(파이 덮개에 쓴다)의 판, 기둥, 나사, 팬 [mm]."""

    plate_mm: tuple[float, float]  # 판 바깥 (긴 변, 짧은 변)
    plate_thickness_mm: float = Field(gt=0)
    hole_pitch_mm: tuple[float, float]  # 귀퉁이 구멍 가운데 사이 (긴 변 쪽, 짧은 변 쪽)
    hole_diameter_mm: float = Field(gt=0)
    post_body_mm: float = Field(gt=0)  # 황동 육각 기둥의 몸통 (한 끝 M3 수나사, 다른 끝 암나사)
    post_stud_mm: float = Field(gt=0)  # 수나사가 몸통에서 나온 길이
    post_thread_mm: float = Field(gt=0)  # 수나사 바깥 지름
    fan_mm: tuple[float, float]  # 팬 한 변, 두께
    fan_hole_pitch_mm: float = Field(gt=0)  # 팬 나사 구멍 가운데 사이
    fan_hole_diameter_mm: float = Field(gt=0)
    fan_opening_mm: float = Field(gt=0)  # 판의 팬 자리 둥근 구멍 지름
    pi_screw_mm: tuple[float, float]  # 파이를 다는 M2.5 접시머리 나사 (나사산 바깥 지름, 전체 길이)
    pi_spacer_mm: tuple[float, float]  # 파이 밑 나일론 링 (높이, 바깥 지름)


class Desk(_Values):
    """로봇을 놓을 책상 [mm]."""

    back_gap_mm: float = Field(
        gt=0
    )  # 상판 뒤 가장자리 ~ 벽 (상판 윗면 높이). 책상을 옮겨 바꿀 수 있다


class BasePlug(_Values):
    """베이스에서 꽂는 플러그 [mm]. 사진으로 어림한 값은 measured.toml의 주석에 적는다."""

    width_mm: float = Field(gt=0)  # 몸통 폭 (단자의 넓은 쪽과 나란히)
    thickness_mm: float = Field(gt=0)  # 몸통 두께
    out_mm: float = Field(gt=0)  # 끝까지 꽂았을 때 단자 면 ~ 딱딱한 끝 (고무 꼬리까지)


class Measured(_Values):
    servo_mass_g: dict[ServoModel, float] = {}  # 서보 몸통 (혼 빼고)
    servo_horn_mass_g: dict[ServoModel, float] = {}  # 혼 두 개 (출력 혼과 뒷면 혼, 나사 포함)
    printed_mass_g: dict[
        str, float
    ] = {}  # 출력물 무게 (STL 파일 이름 → g). 무게 어림을 맞춰 보는 데 쓴다
    servo_body: dict[ServoModel, ServoBody] = {}
    servo_depth: ServoDepth | None = None
    servo_wire: ServoWire | None = None
    waveshare_servo_wire: BoughtServoWire | None = None  # 손목 구간과 긴 구간에 쓰는 선 (0031)
    d405_cable: Cable | None = None  # D405 기본 선 (USB 3, 1 m)
    d405_usb2_cable: ThinCable | None = None  # D405에 쓸 얇은 USB 2 선 (0031)
    d405_usb2_plug: AnglePlug | None = None  # 그 선의 카메라 쪽 ㄱ자 플러그
    servo_plug: ServoPlug | None = None
    servo_back_screw: BackScrew | None = None
    pi_case: PiCase | None = None  # 적층 아크릴 케이스 (파이 덮개)
    desk: Desk | None = None
    base_plug: dict[str, BasePlug] = {}  # 베이스에서 꽂는 플러그 (pi_power, lan, d405_usb_a)
    pi_adapter_cable: ThinCable | None = None  # 파이와 서보 어댑터를 잇는 USB A–C 선
    bearing_mass_g: dict[str, float] = {}  # 베어링 하나의 무게 (호칭 번호마다, 예: "6806")
    # 조립한 무게 (조립 이름 → g). 팔 모델의 무게와 맞춰 본다
    assembly_mass_g: dict[str, float] = {}
    fit: dict[Filament, PrintFit] = {}


def read_toml(path: Path) -> dict[str, Any]:
    with path.open("rb") as file:
        return tomllib.load(file)


def load_arm(path: Path = ARM_FILE) -> ArmParams:
    return ArmParams.model_validate(read_toml(path))


def load_servos(path: Path = SERVOS_FILE) -> dict[ServoModel, ServoSpec]:
    return TypeAdapter(dict[ServoModel, ServoSpec]).validate_python(read_toml(path))


def load_measured(path: Path = MEASURED_FILE) -> Measured:
    return Measured.model_validate(read_toml(path))


def servo_mass(model: ServoModel, servos: dict[ServoModel, ServoSpec], measured: Measured) -> float:
    """서보 무게 [kg]. 저울로 잰 값이 있으면 그 값을, 없으면 명목값을 쓴다 (0006의 "질량")."""
    grams = measured.servo_mass_g.get(model, servos[model].mass_g)
    return grams * 1e-3


def arm_robot(params: ArmParams, servos: dict[ServoModel, ServoSpec]) -> Robot:
    """UR형 팔의 로봇 기술 (SI). 관절의 낼 수 있는 토크와 속도는 서보의 정격 토크와 무부하 속도다.

    플랜지 z(도구 방향)는 J6 축(y)을 본다. 도구 축 둘레로는 -90° 더 돌려서, 기본 작업 자세
    (도구가 아래, 박스 돌림 0°)에서 J6가 범위 가운데에 오게 한다 (0013).
    """
    g = params.arm

    def joint(name: str, origin_mm: Vec3, axis: Vec3) -> Joint:
        values = params.joints[name]
        spec = servos[values.servo]
        lower, upper = np.radians(values.range_deg)
        origin = (origin_mm[0] * 1e-3, origin_mm[1] * 1e-3, origin_mm[2] * 1e-3)
        return Joint(
            name, origin, axis, float(lower), float(upper), spec.rated_torque, spec.max_speed
        )

    joints = (
        joint("j1", (0.0, 0.0, 0.0), Z),
        joint("j2", (0.0, 0.0, g.shoulder_height_mm), Y),
        joint("j3", (0.0, 0.0, g.upper_arm_mm), Y),
        joint("j4", (0.0, 0.0, g.forearm_mm), Y),
        joint("j5", (0.0, g.j4_to_j5_mm, 0.0), Z),
        joint("j6", (0.0, 0.0, g.j5_to_j6_mm), Y),
    )
    flange = np.eye(4)
    flange[:3, :3] = rotation(X, -np.pi / 2) @ rotation(Z, -np.pi / 2)
    flange[:3, 3] = (0.0, g.j6_to_flange_mm * 1e-3, 0.0)
    return Robot("horibot", joints, flange)


def load_robot() -> Robot:
    """레포의 값 파일로 만든 지금 설계의 로봇 기술."""
    return arm_robot(load_arm(), load_servos())
