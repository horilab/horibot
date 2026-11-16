"""부품 목록: 안내의 "이번에 쓰는 것"과 준비물에 나오는 것 (산 부품, 나사와 너트, 공구).

출력 부품은 안내마다 설계 코드의 뽑는 부품 표(parts/*.PRINT_SPEC)에서 만든다 (printed_items). 나사 그림은
화면이 draw의 치수로 그림끼리 같은 비율로 그린다. 나사의 머리는 둥근머리 렌치볼트(ISO 7380)다 (bom.md).
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from typing import Any

from horilab_design.guide.model import Item, View

KINDS = {"printed": "출력한 부품", "bought": "산 부품", "hardware": "나사와 너트", "tool": "공구"}
COLOR_KO = {"white": "흰색", "black": "검정"}

# 출력 부품의 작은 그림 방향 (부품을 잘 알아보는 쪽)
THUMBS = {
    "wrist_body": View((0, 1, 0), (0.7, 0, 1), 24, 0),
    "out_disc": View((0, 1, 0), (1, 0, 0.3), 38, 0),
    "hub_plate": View((0, 1, 0), (0, 0, 1), 50, 0),
    "retainer": View((0, -1, 0), (1, 0, 0), 28, 34),
    "carrier": View((0, -1, 0), (-1, 0, 0.4), 34, 0),
    "wrist_cover": View((0, -1, 0), (1, 0, 0), 45, 0),
}


def _screw(length: int) -> Item:
    return Item(
        f"m3x{length}",
        f"M3 × {length}",
        "hardware",
        "둥근머리 렌치볼트",
        draw={"t": "screw", "L": length},
    )


ITEMS: dict[str, Item] = {
    item.key: item
    for item in (
        Item(
            "bearing_6806",
            "베어링 6806",
            "bought",
            "바깥 지름 42 · 안지름 30 · 두께 7 mm",
            thumb=View((0, 1, 0), (0, 0, 1), 40, 0),
        ),
        Item(
            "servo_sts3215",
            "서보 STS3215",
            "bought",
            "혼이 달린 것",
            thumb=View((0, 1, 0), (0.4, 0, 1), 22, 0),
        ),
        Item("servo_cable", "서보 선 20 cm", "bought", "서보에 딸린 것", icon="wire"),
        Item(
            "d405_cable",
            "D405 선",
            "bought",
            "얇은 USB 2 선 (카메라 쪽 플러그가 ㄱ자). 감아 돌려 볼 때만 쓴다",
            icon="wire",
        ),
        Item(
            "pin_msh3_20",
            "영점 핀 MSH3-20",
            "bought",
            "강철 핀, 지름 3 · 길이 20 mm",
            draw={"t": "pin", "L": 20, "d": 3},
        ),
        _screw(8),
        _screw(10),
        _screw(12),
        _screw(20),
        Item("nut_m3", "M3 너트", "hardware", "육각", draw={"t": "nut"}),
        Item(
            "selftap_so101",
            "서보 뒷면 나사",
            "hardware",
            "옛 SO-101의 작은 십자 자가탭. 길이만 잰 값(7.16 mm), 모양은 어림",
            draw={"t": "tap", "L": 7.16},
        ),
        Item(
            "washer_m3",
            "와셔",
            "hardware",
            "가진 M3 평와셔 (8 × 0.5)",
            draw={"t": "washer", "D": 8, "d": 3.2},
        ),
        Item("hexkey", "육각렌치", "tool", "M3 둥근머리용 (보통 2 mm)", icon="hex"),
        Item("driver", "작은 십자 드라이버", "tool", "서보 뒷면 나사", icon="driver"),
        Item("tweezers", "핀셋", "tool", "실 걷기, 너트 잡기", icon="tweezers"),
        Item("knife", "작은 칼이나 니퍼", "tool", "처진 실 걷기", icon="knife"),
        Item("tape", "테이프", "tool", "선을 잡을 때", icon="tape"),
    )
}


def printed_items(labels: Mapping[str, str], spec: Mapping[str, Sequence[Any]]) -> dict[str, Item]:
    """뽑는 부품 표(키 → (형상 함수, 위 방향, 색, 프리셋))로 출력 부품 목록을 만든다. 키는 STL 파일 이름이다.
    형상은 만들지 않는다."""
    out: dict[str, Item] = {}
    for key, row in spec.items():
        color = str(row[2])
        out[key] = Item(
            key,
            labels[key],
            "printed",
            COLOR_KO.get(color, color),
            file=f"{key}.stl",
            color=color,
            thumb=THUMBS.get(key),
        )
    return out
