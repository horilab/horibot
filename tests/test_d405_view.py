"""D405 화면 보기의 색 바꾸기와 저장 (tools/d405_view.py)."""

import csv
import json
from pathlib import Path

import cv2
import numpy as np

from horibot_camera.message import Frame, Header
from tools import d405_view


def _frame(captured: float | None) -> Frame:
    header = Header(3, 30, 31, 4, 2, 0.0001, captured, 1_764_300_000.5, 1_764_300_000.51)
    depth = np.array([[0, 1000, 2500, 5000], [6000, 40000, 0, 1]], dtype=np.uint16)
    color = np.full((2, 4, 2), 128, dtype=np.uint8)
    return Frame(header, depth, color)


def test_depth_colors_near_red_far_blue_and_empty_black() -> None:
    frame = _frame(1.0)
    image = d405_view.colorize_depth(frame.depth, frame.header.depth_scale)
    assert image.shape == (2, 4, 3) and image.dtype == np.uint8
    assert (image[0, 0] == 0).all() and (image[1, 2] == 0).all()  # 값이 없는 곳
    near_b, _, near_r = (int(v) for v in image[1, 3])  # 0.1 mm: 가장 가깝다
    far_b, _, far_r = (int(v) for v in image[1, 1])  # 4 m: 끝(0.5 m)을 넘는다
    assert near_r > near_b and far_b > far_r
    assert int(image[1, 1].max()) > 150  # 먼 곳도 값이 없는 곳(검정)과 헷갈리지 않게 밝다
    assert (
        image[1, 1] == d405_view.colorize_depth(np.array([[5000]], np.uint16), 0.0001)[0, 0]
    ).all()


def test_ssh_names_resolve_through_the_ssh_config(monkeypatch) -> None:
    def no_dns(host: str) -> str:
        raise OSError("No such host is known")

    class Done:
        stdout = "user hori2\nhostname 192.168.0.102\nport 22\n"

    monkeypatch.setattr(d405_view.socket, "gethostbyname", no_dns)
    monkeypatch.setattr(d405_view.subprocess, "run", lambda *a, **k: Done())
    assert d405_view.resolve("hori2") == "192.168.0.102"
    monkeypatch.setattr(d405_view.socket, "gethostbyname", lambda host: host)
    assert d405_view.resolve("127.0.0.1") == "127.0.0.1"


def test_gray_yuyv_becomes_gray() -> None:
    bgr = d405_view.to_bgr(_frame(1.0).color_yuyv)
    assert bgr.shape == (2, 4, 3)
    assert np.abs(bgr.astype(int) - 128).max() <= 2


def test_snapshots_write_images_rows_and_meta(tmp_path: Path) -> None:
    snapshots = d405_view.Snapshots(tmp_path / "view")
    with_time = _frame(1_764_300_000.4567)
    snapshots.save(with_time, d405_view.to_bgr(with_time.color_yuyv))
    without_time = _frame(None)  # 메타데이터가 없으면 파이가 받은 때로 이름을 짓는다
    snapshots.save(without_time, d405_view.to_bgr(without_time.color_yuyv))
    snapshots.close({"saved": snapshots.saved})
    folder = tmp_path / "view"
    assert d405_view.snapshot_stem(with_time) == "1764300000457"
    assert d405_view.snapshot_stem(without_time) == "1764300000500"
    depth = cv2.imread(str(folder / "1764300000457_depth.png"), cv2.IMREAD_UNCHANGED)
    assert depth is not None
    assert depth.dtype == np.uint16 and np.array_equal(depth, with_time.depth)
    assert (folder / "1764300000500_color.jpg").exists()
    with (folder / "frames.csv").open(encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    assert [r["depth"] for r in rows] == ["1764300000457_depth.png", "1764300000500_depth.png"]
    assert json.loads((folder / "meta.json").read_text(encoding="utf-8")) == {"saved": 2}
