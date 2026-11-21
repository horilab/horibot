"""파이 자리: 날개 위의 기둥에 얹는 파이 판(작은 판)과, 그 위아래에 놓이는 것들의 자리 (docs/decisions/0038-base-layout.md).

책상에 무는 판(parts/base.py의 plate)의 날개가 곧 파이 받침이다. 날개의 기둥 넷 위에 파이 판을 M3 × 8 넷으로 달고,
파이 판에 파이(기판만)와 덮개 기둥 넷을 단다. 덮개는 가진 적층 아크릴 케이스의 팬 구멍 판과 팬이다. 파이 판 밑에는
클램프의 위 턱이 들고, 컨트롤러 파이 밑에는 서보 어댑터가 눕는다. 파이와 덮개에 맞춘 것은 모두 이 작은 판에 있어서,
가정한 값이 틀리면 이 판만 다시 뽑는다.

- 두 파이는 같은 방향이다 (USB·랜이 뒤, 전원이 로봇 왼쪽을 보는 긴 변). 그래서 파이 판 둘이 같은 부품이다.
- 컨트롤러 파이(hori1)가 오른쪽, 카메라 파이(hori2)가 왼쪽이다. D405 선의 J1·J2 고리는 팔이 J1을 따라 ±160° 도는
  가운데 방위(로봇 왼쪽 앞)에서 베이스에 묶어야 가장 짧고, 거기서 왼쪽 파이의 USB 단자까지가 가깝다
  (studies/wiring.py의 D405 선). 서보 선은 어느 쪽이든 닿는다.
- 파이: M2.5 × 12 접시머리 나사(케이스의 것)가 기판, 흰 나일론 링(높이 3.17), 파이 판의 돋움을 지나 판 밑의
  M2.5 너트에 물린다. 가정: 케이스에 M2.5 너트가 여덟 개 있다. 구멍은 맞춤 없이 지나가는 구멍이다.
- 덮개 기둥(육각 황동, 몸통 22.93, 한 끝 M3 수나사 6.2): 수나사가 파이 판의 긴 구멍을 지나 판 밑의 M3 너트에 물린다.
  긴 구멍은 x 쪽으로 ±1 mm 길다 (판이 기판 가운데에 맞는지 모른다: 가정).
- 조립 순서: 날개에 서보 어댑터(컨트롤러 쪽)와 선 → 손에 든 파이 판에 덮개 기둥 넷(밑에서 너트)과 파이(밑에서 너트)
  → 파이 판을 기둥에 얹고 M3 × 8 넷 → 젠더와 플러그 → 팬을 단 아크릴 판. 클램프는 파이 판을 떼지 않고 뒤에서 끼우고 뺀다.

좌표(mm)는 parts/base.py와 같다 (J1의 서보 좌표: x 앞, y 위, z 오른쪽 = 로봇 좌표의 -y).

    uv run python -m horilab_design.parts.pi_mount
"""

from __future__ import annotations

from build123d import Compound, Pos

from horilab_design.parts import base as b
from horilab_design.parts import joint as j
from horilab_design.parts.shapes import box, cyl_y, far_apart, solid
from horilab_design.printcheck import air_wall_lines, mid_layer_levels, overhangs, printed_mass_g

_M = j._MEASURED
_CASE = _M.pi_case
assert _CASE is not None
_PLUG = _M.base_plug

# ---- 누가 어느 쪽인가 (z의 부호: +1이 로봇 오른쪽) ----
CONTROLLER, CAMERA = 1, -1

# ---- 파이 판 ----
PI_HOLES = (
    (39.0, 24.5),
    (39.0, -24.5),
    (-19.0, 24.5),
    (-19.0, -24.5),
)  # 기판 가운데에서 (x, 옆). 공식 도면
PI_HOLE_D = 3.0  # M2.5가 지나가는 구멍 (맞춤 없이 넉넉히)
PI_BOSS = (3.1, 1.6)  # 나일론 링이 앉는 돋움의 반지름과 높이: 젠더와 판 나사 머리 사이를 띄운다
POST_SLOT = (3.6, 1.0)  # 덮개 기둥 수나사의 긴 구멍: 폭, x 쪽으로 더 긴 반길이
WINDOW = ((-12.0, 32.0), 17.0)  # 판 가운데의 창: x 범위, 옆 반폭 (바람이 통하고 선이 지난다)
# 찍찍이 타이 구멍 (가정: 타이 폭 12, 두께 1.3. 아직 사지 않았다): 긴 변마다 하나, 앞 변에 둘
TIE = (14.0, 3.0)
TIE_EDGE = 4.0  # 구멍과 판 가장자리 사이의 살
TIE_FRONT_V = (7.5, 20.5)  # 앞 변 구멍의 옆 범위 (SD 카드와 파이 돋움 사이)

# ---- 파이와 덮개 (자리만 그린다) ----
BOARD_T = 1.5  # 기판 두께 (가정: 도면에 없다)
RING_H = _CASE.pi_spacer_mm[0]  # 흰 나일론 링의 높이 3.17 (잰 값)
POST_BODY = _CASE.post_body_mm  # 덮개 기둥의 몸통 22.93 (잰 값)
POST_STUD = _CASE.post_stud_mm  # 수나사 길이 6.2 (잰 값)
COVER = (*_CASE.plate_mm, _CASE.plate_thickness_mm)  # 아크릴 판 97.84 × 72.52 × 2.86 (잰 값)
FAN = _CASE.fan_mm  # 30 × 30 × 7 (상품 이름의 값)
SPACER_R = _CASE.pi_spacer_mm[1] / 2  # 나일론 링의 바깥 반지름 (잰 값)
PI_SCREW = _CASE.pi_screw_mm[1]  # 파이를 다는 M2.5 접시머리 나사의 전체 길이 11.93 (잰 값)
# 기판 위 단자: (앞 끝에서 x0, x1, 전원 쪽 긴 변에서 y0, y1, 높이). 공식 도면에서 읽었다 (hardware.md)
PORTS = {
    "랜": (67.0, 88.0, 37.75, 53.75, 13.5),
    "USB 2.0": (70.0, 88.0, 20.0, 34.0, 16.0),
    "USB 3.0": (70.0, 88.0, 2.0, 16.0, 16.0),
    "USB-C 전원": (7.6, 14.8, -b.USBC_OUT, 6.3, 3.2),
    "GPIO": (7.0, 58.0, 50.0, 55.0, 8.5),
}
DONGLE_OUT = 30.0  # 게임패드 동글이 단자 밖으로 나온 길이 (가정: 재지 않았다)
UM2_OUT = 30.0  # 파이와 어댑터를 잇는 USB A–C 선의 플러그가 나온 길이 (가정: 재지 않았다)

LABELS = {"pi_tray": "파이 판"}
PROVISIONAL = (
    "아크릴 판의 팬 구멍과 파이 고정 구멍의 자리 (판이 기판 가운데에 맞고 팬이 판 가운데에 있다고 봤다)",
    "케이스의 M2.5 너트 (여덟 개가 있다고 봤다)",
    "기판 두께 BOARD_T, 게임패드 동글 DONGLE_OUT, USB A–C 플러그 UM2_OUT",
    "찍찍이 타이의 폭과 두께 (TIE)",
)

Y_TRAY = b.Y_DESK + b.TRAY_UNDER  # 파이 판 밑면
Y_TOP = Y_TRAY + b.TRAY_T  # 파이 판 윗면
Y_BOARD = Y_TOP + PI_BOSS[1] + RING_H  # 기판 밑면
Y_COVER = Y_TOP + POST_BODY  # 아크릴 판 밑면


def post_uv() -> list[tuple[float, float]]:
    """덮개 기둥 넷 (기판 가운데에서 x, 옆)."""
    hx, hv = b.COVER_PITCH[0] / 2, b.COVER_PITCH[1] / 2
    return [(sx * hx, sv * hv) for sx in (1, -1) for sv in (1, -1)]


def screw_uv() -> list[tuple[float, float]]:
    """파이 판을 날개의 기둥에 다는 나사 넷 (기판 가운데에서 x, 옆)."""
    return [(sx * b.POST_DX, sv * b.POST_DZ) for sx in (1, -1) for sv in (1, -1)]


def tie_slots() -> list[tuple[float, float, float, float]]:
    """타이 구멍 넷: (x0, x1, 옆 0, 옆 1). 긴 변 둘의 가운데와 앞 변의 둘."""
    long_, wide = TIE
    v1 = b.TRAY_HZ - TIE_EDGE
    u1 = b.TRAY_HX - TIE_EDGE
    a, c = TIE_FRONT_V
    return [
        (-long_ / 2, long_ / 2, v1 - wide, v1),
        (-long_ / 2, long_ / 2, -v1, -v1 + wide),
        (u1 - wide, u1, a, c),
        (u1 - wide, u1, -c, -a),
    ]


def tray_local() -> Compound:
    """파이 판 (기판 가운데가 x = 0, z = 0인 자리로 그린 것). 밑면을 바닥에 두고 뽑는다.

    전원 쪽 긴 변(옆 +)이 -z다. 두 파이가 같은 방향이라 이 판 하나를 양쪽에 쓴다.
    """
    hx, hz = b.TRAY_HX, b.TRAY_HZ
    t = box(-hx, hx, Y_TRAY, Y_TOP, -hz, hz)
    (u0, u1), wv = WINDOW
    t = solid(t - box(u0, u1, Y_TRAY - 1, Y_TOP + 1, -wv, wv))
    r, h = PI_BOSS
    for u, v in PI_HOLES:
        t = solid(t + cyl_y(r, Y_TOP - 0.01, Y_TOP + h, u, -v))
        t = solid(t - cyl_y(PI_HOLE_D / 2, Y_TRAY - 1, Y_TOP + h + 1, u, -v))
    w, d = POST_SLOT
    for u, v in post_uv():
        slot = cyl_y(w / 2, Y_TRAY - 1, Y_TOP + 1, u - d, -v) + cyl_y(
            w / 2, Y_TRAY - 1, Y_TOP + 1, u + d, -v
        )
        slot += box(u - d, u + d, Y_TRAY - 1, Y_TOP + 1, -v - w / 2, -v + w / 2)
        t = solid(t - solid(slot))
    for u, v in screw_uv():
        t = solid(t - cyl_y(j.K_M3_FREE / 2, Y_TRAY - 1, Y_TOP + 1, u, -v))
    for a0, a1, v0, v1 in tie_slots():
        t = solid(t - box(a0, a1, Y_TRAY - 1, Y_TOP + 1, -v1, -v0))
    if not t.is_valid or len(t.solids()) != 1:
        raise RuntimeError("파이 판 형상이 깨졌다")
    return t


def at(side: int) -> Pos:
    """기판 가운데가 원점인 자리를 side 쪽 파이의 자리로 옮긴다 (side: +1 오른쪽, -1 왼쪽)."""
    return Pos(b.PI_CX, 0.0, side * b.PI_CZ)


def tray(side: int) -> Compound:
    return solid(at(side) * tray_local())


def _rbox(x0: float, x1: float, v0: float, v1: float, y0: float, y1: float) -> Compound:
    """기판 가운데에서 (x, 옆) 범위와 높이로 만든 상자 (옆 +가 -z)."""
    return box(x0, x1, y0, y1, -v1, -v0)


def pi_parts(side: int) -> list[j.Part]:
    """파이 한 대와 덮개, 꽂힌 플러그 (자리만 그린 상자와 원기둥). side 쪽 자리다.

    기판에서 전원 쪽 긴 변이 옆 +(로봇 왼쪽), USB·랜이 뒤다.
    """
    hx, hv = b.PI_BOARD[0] / 2, b.PI_BOARD[1] / 2
    top = Y_BOARD + BOARD_T
    who = "컨트롤러 파이" if side == CONTROLLER else "카메라 파이"
    out: list[tuple[str, Compound, str]] = [
        ("기판", _rbox(-hx, hx, -hv, hv, Y_BOARD, top), "servo")
    ]

    def port(key: str) -> tuple[float, float, float, float, float]:
        x0, x1, y0, y1, h = PORTS[key]
        return hx - x1, hx - x0, hv - y1, hv - y0, h

    for key in PORTS:
        x0, x1, v0, v1, h = port(key)
        out.append((key, _rbox(x0, x1, v0, v1, top, top + h), "metal"))
    # ㄱ자 젠더 (ML-CCSG3, 도면 값): 전원 단자에 꽂혀 옆으로 나오고, 소켓이 뒤를 본다. 전원 플러그가 거기서 뒤로 간다
    x0, x1, _v0, v1, h = port("USB-C 전원")
    xc, yc = (x0 + x1) / 2, top + h / 2
    g_len, g_deep, g_t = b.GENDER_BODY
    g0, g1 = v1, v1 + b.GENDER_OUT
    g_front = xc + b.GENDER_END  # 닫힌 끝 (앞쪽)
    out.append(
        ("젠더", _rbox(g_front - g_len, g_front, g0, g1, yc - g_t / 2, yc + g_t / 2), "plug")
    )
    pw = _PLUG["pi_power"]
    pv = g1 - g_deep / 2  # 소켓은 몸통 폭의 가운데다
    px1 = g_front - g_len
    out.append(
        (
            "전원 플러그",
            _rbox(
                px1 - pw.out_mm,
                px1,
                pv - pw.width_mm / 2,
                pv + pw.width_mm / 2,
                yc - pw.thickness_mm / 2,
                yc + pw.thickness_mm / 2,
            ),
            "plug",
        )
    )
    x0, _x1, v0, v1, h = port("랜")
    lan = _PLUG["lan"]
    vm = (v0 + v1) / 2
    out.append(
        (
            "랜 플러그",
            _rbox(
                x0 - lan.out_mm,
                x0,
                vm - lan.width_mm / 2,
                vm + lan.width_mm / 2,
                top + 1,
                top + 1 + lan.thickness_mm,
            ),
            "plug",
        )
    )
    usb = _PLUG["d405_usb_a"]
    if side == CAMERA:  # D405 선의 USB-A
        x0, _x1, v0, v1, h = port("USB 3.0")
        vm = (v0 + v1) / 2
        out.append(
            (
                "D405 USB 플러그",
                _rbox(
                    x0 - usb.out_mm,
                    x0,
                    vm - usb.width_mm / 2,
                    vm + usb.width_mm / 2,
                    top + 8.5,
                    top + 8.5 + usb.thickness_mm,
                ),
                "plug",
            )
        )
    else:  # 게임패드 동글과 서보 어댑터로 가는 USB 선 (둘 다 USB 2.0 단자: 가정한 크기)
        x0, _x1, v0, v1, h = port("USB 2.0")
        vm = (v0 + v1) / 2
        out.append(
            (
                "게임패드 동글",
                _rbox(x0 - DONGLE_OUT, x0, vm - 7.0, vm + 7.0, top + 8.5, top + 8.5 + 6.5),
                "plug",
            )
        )
        out.append(
            (
                "어댑터 USB 플러그",
                _rbox(x0 - UM2_OUT, x0, vm - 7.0, vm + 7.0, top + 1.0, top + 1.0 + 6.5),
                "plug",
            )
        )
    for k, (u, v) in enumerate(PI_HOLES, start=1):
        ring = cyl_y(SPACER_R, Y_TOP + PI_BOSS[1], Y_BOARD, u, -v)
        out.append((f"나일론 링 {k}", ring, "body"))
    for k, (u, v) in enumerate(post_uv(), start=1):
        out.append((f"덮개 기둥 {k}", cyl_y(3.0, Y_TOP, Y_COVER, u, -v), "metal"))
        stud = cyl_y(1.5, Y_TOP - POST_STUD, Y_TOP, u, -v) + j.hex_y(
            u, -v, Y_TRAY - j.NUT_T, Y_TRAY, 5.4
        )
        out.append((f"덮개 기둥 수나사와 너트 {k}", solid(stud), "metal"))
    cl, cw, ct = COVER
    out.append(("덮개 판", _rbox(-cl / 2, cl / 2, -cw / 2, cw / 2, Y_COVER, Y_COVER + ct), "cap"))
    fs, ft = FAN
    out.append(("팬", _rbox(-fs / 2, fs / 2, -fs / 2, fs / 2, Y_COVER - ft, Y_COVER), "servo"))
    for k, (u, v) in enumerate(screw_uv(), start=1):
        s = cyl_y(1.5, Y_TOP - 8.0, Y_TOP, u, -v) + cyl_y(2.85, Y_TOP, Y_TOP + 1.65, u, -v)
        out.append((f"파이 판 나사 M3x8 {k}", solid(s), "metal"))
    move = at(side)
    return [j.Part(f"{who}/{n}", solid(move * s), color=c) for n, s, c in out]


def adapter_parts(side: int = CONTROLLER) -> list[j.Part]:
    """서보 어댑터와 꽂힌 플러그 (자리만. 높이와 단자 자리는 가정이다): 컨트롤러 파이 밑.

    기판 위로 ADAPTER_TALL까지를 한 덩어리로 그린다 (가장 높은 부품이 어디에 있는지 모른다).
    """
    ax, az = b.ADAPTER[0] / 2, b.ADAPTER[1] / 2
    y0 = b.Y_DESK + b.WING_T + b.ADAPTER_SEAT
    cx, cz = b.ADAPTER_CX, side * b.ADAPTER_CZ
    body = box(cx - ax, cx + ax, y0, y0 + 1.6 + b.ADAPTER_TALL, cz - az, cz + az)
    inner, outer = sorted((cz - side * az, cz + side * az), key=abs)
    # 서보 플러그 둘(몸통 쪽 긴 변)과 USB-C 플러그(바깥 쪽 긴 변). 변의 어디에 있는지 몰라 변 전체로 그린다
    servo = box(cx - ax, cx + ax, y0 + 1.6, y0 + 1.6 + 6.0, *sorted((inner, inner - side * 10.0)))
    usb = box(cx - ax, cx + ax, y0 + 1.6, y0 + 1.6 + 7.0, *sorted((outer, outer + side * UM2_OUT)))
    return [
        j.Part("서보 어댑터", body, color="servo"),
        j.Part("서보 어댑터/서보 플러그 자리", servo, color="plug"),
        j.Part("서보 어댑터/USB와 전원선 자리", usb, color="plug"),
    ]


def parts() -> list[j.Part]:
    """파이 판 둘과 파이 두 대, 덮개, 플러그, 서보 어댑터 (베이스에 서 있는 것들)."""
    out = [
        j.Part("파이 판 (오른쪽)", tray(1), color="frame"),
        j.Part("파이 판 (왼쪽)", tray(-1), color="frame"),
    ]
    return out + pi_parts(1) + pi_parts(-1) + adapter_parts()


def clearances(limit: float = 6.0) -> dict[tuple[str, str], float]:
    """파이 쪽 것들과 베이스 몸통, 책상에 무는 판, 클램프 사이의 틈 [mm] (limit 아래만. 파이 판과 기둥은 닿는다)."""
    mine = parts()
    fixed = [
        j.Part("베이스 몸통", b.core()),
        j.Part("책상에 무는 판", b.plate()),
        *[p for p in b.desk_parts() if p.name.startswith("클램프")],
    ]
    out: dict[tuple[str, str], float] = {}
    for a in mine:
        for other in fixed:
            if far_apart(a.shape, other.shape, limit):
                continue
            d = a.shape.distance_to(other.shape)
            if d < limit:
                out[(a.name, other.name)] = d
    return out


def stack() -> dict[str, float]:
    """높이 쌓임 (책상에서) [mm]와 여유."""
    desk = b.Y_DESK
    top = Y_BOARD + BOARD_T
    jaw_top = b.WING_T + b.JAW_T
    return {
        "클램프 위 턱의 윗면": jaw_top,
        "서보 어댑터에서 가장 높은 곳 (가정)": b.WING_T + b.ADAPTER_SEAT + 1.6 + b.ADAPTER_TALL,
        "파이 판 밑면": Y_TRAY - desk,
        "파이 기판 밑면": Y_BOARD - desk,
        "USB 단자 윗면": top + 16.0 - desk,
        "팬 밑면": Y_COVER - FAN[1] - desk,
        "아크릴 판 밑면": Y_COVER - desk,
        "아크릴 판 윗면": Y_COVER + COVER[2] - desk,
        "띠 밑 (선의 고리가 생기는 높이)": b.FRONT - b.BAND_H - desk,
        "파이 나사 M2.5 × 12가 판 밑으로 나온 길이 (너트 2.0)": PI_SCREW
        - (BOARD_T + RING_H + PI_BOSS[1] + b.TRAY_T),
        "덮개 기둥 수나사가 판 밑으로 나온 길이 (너트 2.4)": POST_STUD - b.TRAY_T,
        "젠더 밑과 파이 판 나사 머리 사이": (top + 1.6 - b.GENDER_BODY[2] / 2) - (Y_TOP + 1.65),
    }


def printed() -> dict[str, tuple[Compound, tuple[float, float, float], str, str, int]]:
    return {"pi_tray": (tray_local(), (0.0, 1.0, 0.0), "black", "strength", 2)}


def report() -> None:
    print(f"가정한 값: {'; '.join(PROVISIONAL)}")
    for name, (shape, up, color, preset, count) in printed().items():
        bb = shape.bounding_box()
        over = overhangs(shape, up)
        print(
            f"  {name} × {count} {color} {bb.size.X:.1f} × {bb.size.Y:.1f} × {bb.size.Z:.1f} mm  "
            f"{printed_mass_g(shape, up, preset):.1f} g  걸침 {over.total:.0f} mm²"
        )
        print(f"  층 가운데에 걸친 수평면: {mid_layer_levels(shape, up) or '없음'}")
    print("\n".join(air_wall_lines({k: v[:4] for k, v in printed().items()})))
    print("높이 (책상에서)")
    for name, value in stack().items():
        print(f"  {name}: {value:.2f}")
    print("파이 쪽 것들과 몸통·판·클램프 사이 틈 [mm] (6 아래)")
    for (a, other), d in sorted(clearances().items(), key=lambda kv: kv[1]):
        print(f"  {a} ↔ {other}: {d:.2f}")


if __name__ == "__main__":
    report()
