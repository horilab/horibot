"""작업 안내의 캘리퍼스 그림: 재는 두 점에 캘리퍼스를 댄 모습이다 (어림 모양, 150 mm짜리).

재는 자리는 글로만 쓰면 자주 헷갈려서, 어느 턱을 어디에 대는지를 3D로 보인다. 모양은 실물과 같지 않아도 되고
턱, 자, 움직이는 턱에 붙은 몸통, 깊이 막대를 알아볼 만큼만 그린다.
"""

from __future__ import annotations

from dataclasses import dataclass

from build123d import Compound, Location, Plane, Vector

from horibot_design.parts.shapes import Point, box, solid

JAW_W = 8.0  # 바깥 턱의 폭 (재는 방향)
JAW_L = 30.0  # 바깥 턱이 자에서 나온 길이 (그림. 실물은 40 mm쯤)
IN_W = 0.9  # 안쪽 턱의 폭 (재는 방향). 지름 2.5 mm 구멍에도 들어가게 가늘게 그린다
IN_L = 15.0  # 안쪽 턱이 자에서 나온 길이
IN_T = 1.6  # 안쪽 턱의 두께
BEAM_H = 16.0  # 자의 폭
THICK = 3.0  # 바깥 턱의 두께
BEAM_T = 3.5  # 자의 두께
HEAD = 6.0  # 고정 턱 바깥으로 나온 자
SLIDER = 28.0  # 움직이는 턱에 붙은 몸통(화면이 있는 것)의 길이
TAIL = 25.0  # 몸통 뒤로 나온 자 (그림이라 짧게 그린다. 실물은 더 길다)
DEPTH_BEAM = 80.0  # 깊이를 잴 때 그리는 자의 길이 (읽은 값 + 턱과 몸통보다 길어야 한다)
ROD = 1.5  # 깊이 막대의 굵기


@dataclass(frozen=True)
class Caliper:
    """캘리퍼스 하나 (장면 좌표, mm).

    a와 b는 재는 두 점이다. kind가 outer면 바깥 턱으로 a와 b를 바깥에서 물고, inner면 작은 안쪽 턱을 벌려 a와 b에
    안에서 대고, depth면 자 끝을 a에 대고 깊이 막대를 b까지 뺀다. side는 a-b 선에서 자가 놓이는 쪽이다 (depth는
    자의 폭 방향). reach는 재는 점까지 닿는 턱의 길이(outer는 바깥 턱, inner는 안쪽 턱), tip은 턱 끝이 재는 점을
    지나 더 나간 길이, beam은 depth의 자 길이다 (한 장면의 깊이 캘리퍼스 둘이 겹치지 않게 바꾼다).
    """

    a: Point
    b: Point
    side: Point
    kind: str = "outer"
    reach: float | None = None
    tip: float = 2.0
    beam: float = DEPTH_BEAM


def _plane(c: Caliper) -> tuple[Plane, float]:
    """a가 원점이고 x가 a → b, y가 side 쪽인 평면과 a-b 길이."""
    a, b = Vector(*c.a), Vector(*c.b)
    length = (b - a).length
    u = (b - a).normalized()
    s = Vector(*c.side)
    s = (s - u * s.dot(u)).normalized()
    return Plane(origin=a, x_dir=u, z_dir=u.cross(s)), length


def _reach(c: Caliper) -> float:
    if c.reach is not None:
        return c.reach
    return IN_L if c.kind == "inner" else JAW_L


def _fuse(parts: list[Compound]) -> Compound:
    out = parts[0]
    for p in parts[1:]:
        out = solid(out + p)
    return out


def _beam_and_slider(length: float, at: float) -> list[Compound]:
    """자(at에서 side 쪽으로 BEAM_H)와 움직이는 턱에 붙은 몸통."""
    bt = BEAM_T / 2
    top = at + BEAM_H
    return [
        box(-JAW_W - HEAD, length + JAW_W + SLIDER + TAIL, at, top, -bt, bt),
        box(length - 3.0, length + JAW_W + SLIDER, at - 3.0, top + 3.0, -bt - 2.0, bt + 2.0),
    ]


def _outer(length: float, reach: float, tip: float) -> Compound:
    t, it = THICK / 2, IN_T / 2
    top = reach + BEAM_H
    w_in = min(
        2.5, length / 2 - 0.1
    )  # 자 반대쪽의 안쪽 턱 (그림만). 맞붙은 둘이 겹치지 않게 좁힌다
    parts = [
        box(-JAW_W, 0.0, -tip, reach, -t, t),
        box(length, length + JAW_W, -tip, reach, -t, t),
        *_beam_and_slider(length, reach),
    ]
    if w_in > 0.2:
        parts += [
            box(0.0, w_in, top, top + 14.0, -it, it),
            box(length - w_in, length, top, top + 14.0, -it, it),
        ]
    return _fuse(parts)


def _inner(length: float, reach: float, tip: float) -> Compound:
    t, it = THICK / 2, IN_T / 2
    top = reach + BEAM_H
    return _fuse(
        [
            box(0.0, IN_W, -tip, reach, -it, it),
            box(length - IN_W, length, -tip, reach, -it, it),
            *_beam_and_slider(length, reach),
            box(-JAW_W, 0.0, top, top + JAW_L, -t, t),  # 자 반대쪽의 바깥 턱 (그림만)
            box(length, length + JAW_W, top, top + JAW_L, -t, t),
        ]
    )


def _depth(length: float, beam: float) -> Compound:
    """자 끝(x = 0)을 a에 대고 막대를 b까지 뺀 모습. 턱과 몸통은 자의 먼 끝에 있다 (몸통은 읽은 값만큼 옮겨 있다)."""
    hb, bt, t, r = BEAM_H / 2, BEAM_T / 2, THICK / 2, ROD / 2
    far = -beam
    jaw = far + JAW_W + length  # 움직이는 턱
    return _fuse(
        [
            box(far, 0.0, -hb, hb, -bt, bt),
            box(0.0, length, -r, r, -r, r),
            box(far, far + JAW_W, -hb - JAW_L, -hb, -t, t),
            box(jaw, jaw + JAW_W, -hb - JAW_L, -hb, -t, t),
            box(jaw - 3.0, jaw + JAW_W + SLIDER, -hb - 3.0, hb + 3.0, -bt - 2.0, bt + 2.0),
        ]
    )


def caliper_shape(c: Caliper) -> Compound:
    """재는 모습의 캘리퍼스 형상 (장면 좌표)."""
    plane, length = _plane(c)
    if c.kind == "outer":
        shape = _outer(length, _reach(c), c.tip)
    elif c.kind == "inner":
        shape = _inner(length, _reach(c), c.tip)
    elif c.kind == "depth":
        shape = _depth(length, c.beam)
    else:
        raise ValueError(f"캘리퍼스 종류가 틀렸다: {c.kind}")
    return solid(shape.moved(Location(plane)))
