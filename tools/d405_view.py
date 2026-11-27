"""D405 화면 보기 (docs/decisions/0039-camera-stream.md, 0031의 D405 선).

카메라 파이가 보내는 영상을 받아 창에 컬러와 뎁스를 나란히 띄우고, 정한 간격(1초)마다 한 장씩 기록 폴더에
저장한다. 처음 전원 날 관절을 돌릴 때 D405 고리가 화면에 들어오는지 본다. 카메라 파이에서 보내는 프로그램
(pi/camera/send.py)이 먼저 돌고 있어야 한다.

uv run tools/d405_view.py [--host hori2] [--port 5555] [--every 1.0] [--seconds 0] [--no-window]

- 창: 왼쪽 컬러, 오른쪽 뎁스(가까울수록 붉다, 0 ~ 0.5 m. 값이 없는 곳은 검정). 위 줄은 초당 장 수, 찍은 때부터
  받기까지의 늦음, 빠진 장(파이 → PC, 카메라 → 파이의 뎁스·컬러), 저장한 장 수다. q나 Esc, 창 닫기로 끝내고,
  s는 지금 장을 하나 더 저장한다.
- 저장: records/camera/<날짜>_<시각>_d405-view-<host>/에 <찍은 때 ms>_color.jpg와 _depth.png(16비트, 한 칸이
  depth_scale m), frames.csv(장마다 찍은 때와 장 번호), meta.json(받은 장, 빠진 장, 시계 오차 한도).
  찍은 때는 파이 시계의 유닉스 시각이라 컨트롤러 기록(events.csv)과 바로 견준다 (두 파이 모두 시각 서버에 맞춘다).
"""

from __future__ import annotations

import argparse
import csv
import json
import socket
import subprocess
import sys
import time
from dataclasses import asdict
from datetime import datetime
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from horibot_camera.message import Frame
from horibot_camera.receiver import PORT, Receiver

ROOT = Path(__file__).resolve().parents[1]
WINDOW = "D405"
FAR_M = 0.5  # 뎁스 색을 매기는 끝 (D405가 잘 재는 거리 7 ~ 50 cm)
FAR_LEVEL = 40  # 먼 곳의 색표 자리 (0 ~ 255, TURBO에서 밝은 파랑)
WAIT_NOTE_S = 5.0
FRAME_COLUMNS = ["captured", "received", "depth_no", "color_no", "seq", "color", "depth"]


def resolve(host: str) -> str:
    """ssh에 주는 이름(hori2)도 받는다: 이름이 풀리지 않으면 ssh 설정(ssh -G)의 hostname을 쓴다.
    파이 이름은 PC의 ssh 설정에만 있어서 zmq가 바로 풀지 못한다 (2026-11-28)."""
    try:
        socket.gethostbyname(host)
        return host
    except OSError:
        pass
    try:
        out = subprocess.run(
            ["ssh", "-G", host], capture_output=True, text=True, timeout=10, check=True
        ).stdout
    except (OSError, subprocess.SubprocessError):
        return host
    for line in out.splitlines():
        key, _, value = line.partition(" ")
        if key == "hostname" and value:
            return value
    return host


def to_bgr(color_yuyv: np.ndarray) -> np.ndarray:
    return cv2.cvtColor(color_yuyv, cv2.COLOR_YUV2BGR_YUYV)


def colorize_depth(depth: np.ndarray, scale: float, far_m: float = FAR_M) -> np.ndarray:
    """뎁스를 색으로: 가까울수록 붉고(0 m) 멀수록 푸르다(far_m 넘으면 같은 색). 값이 없는 곳(0)은 검정."""
    meters = depth.astype(np.float32) * scale
    near = np.clip(1.0 - meters / far_m, 0.0, 1.0)
    # 색표의 맨 아래(거의 검정)는 쓰지 않는다: 먼 곳이 값이 없는 곳(검정)과 헷갈린다
    level = (FAR_LEVEL + (255 - FAR_LEVEL) * near).astype(np.uint8)
    image = cv2.applyColorMap(level, cv2.COLORMAP_TURBO)
    image[depth == 0] = 0
    return image


def snapshot_stem(frame: Frame) -> str:
    """저장 파일 이름의 앞: 찍은 때 [ms] (메타데이터가 없으면 파이가 받은 때)."""
    when = frame.header.captured if frame.header.captured is not None else frame.header.received
    return f"{round(when * 1000)}"


class Snapshots:
    """기록 폴더에 장을 저장한다."""

    def __init__(self, folder: Path) -> None:
        self.folder = folder
        folder.mkdir(parents=True, exist_ok=True)
        self._csv = (folder / "frames.csv").open("w", newline="", encoding="utf-8")
        self._rows = csv.writer(self._csv)
        self._rows.writerow(FRAME_COLUMNS)
        self.saved = 0

    def save(self, frame: Frame, color_bgr: np.ndarray) -> None:
        stem = snapshot_stem(frame)
        color_file, depth_file = f"{stem}_color.jpg", f"{stem}_depth.png"
        cv2.imwrite(str(self.folder / color_file), color_bgr, [cv2.IMWRITE_JPEG_QUALITY, 90])
        cv2.imwrite(str(self.folder / depth_file), frame.depth)
        h = frame.header
        self._rows.writerow(
            [h.captured, h.received, h.depth_no, h.color_no, h.seq, color_file, depth_file]
        )
        self._csv.flush()
        self.saved += 1

    def close(self, meta: dict[str, Any]) -> None:
        self._csv.close()
        (self.folder / "meta.json").write_text(
            json.dumps(meta, ensure_ascii=False, indent=1), encoding="utf-8"
        )


def overlay(image: np.ndarray, text: str) -> None:
    cv2.putText(image, text, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(
        image, text, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA
    )


def window_closed() -> bool:
    try:
        return cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1
    except cv2.error:
        return True


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="D405 화면 보기")
    parser.add_argument("--host", default="hori2", help="카메라 파이 (ssh 이름이나 주소)")
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument(
        "--every", type=float, default=1.0, help="저장 간격 [s], 0이면 저장하지 않는다"
    )
    parser.add_argument(
        "--seconds", type=float, default=0.0, help="이만큼 받고 끝낸다, 0이면 끝내기 전까지"
    )
    parser.add_argument("--no-window", action="store_true", help="창 없이 받고 저장만 한다")
    args = parser.parse_args(argv)

    started = datetime.now()
    folder = ROOT / "records" / "camera" / f"{started:%Y-%m-%d_%H%M%S}_d405-view-{args.host}"
    snapshots = Snapshots(folder) if args.every > 0 else None
    print(
        f"받는다: {args.host}:{args.port}" + (f", 저장: {folder}" if snapshots else ""), flush=True
    )

    first: Frame | None = None
    rate_at, rate_frames, fps = time.monotonic(), 0, 0.0
    last_frame = save_at = note_at = time.monotonic()
    end_at = time.monotonic() + args.seconds if args.seconds > 0 else None
    latency_ms: float | None = None
    shown = False  # 창에 한 번이라도 띄웠는지 (그 전에는 창이 닫혔는지 묻지 않는다)
    address = resolve(args.host)
    if address != args.host:
        print(f"{args.host}의 주소: {address} (ssh 설정)", flush=True)
    with Receiver(address, args.port) as receiver:
        if not args.no_window:
            cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        while end_at is None or time.monotonic() < end_at:
            got = receiver.receive(0.2)
            now = time.monotonic()
            if got is None:
                if receiver.end is not None and now - note_at > WAIT_NOTE_S:
                    print(f"보내는 쪽이 멈췄다: {receiver.end.summary}", flush=True)
                    note_at = now
                elif now - last_frame > WAIT_NOTE_S and now - note_at > WAIT_NOTE_S:
                    print(
                        "영상이 오지 않는다: 카메라 파이에서 보내는 프로그램이 도는지 본다",
                        flush=True,
                    )
                    note_at = now
                if not args.no_window:
                    key = cv2.waitKey(1) & 0xFF
                    if key in (ord("q"), 27) or (shown and window_closed()):
                        break
                continue
            frame = got.frame
            first = first or frame
            last_frame = now
            rate_frames += 1
            if now - rate_at >= 1.0:
                fps, rate_at, rate_frames = rate_frames / (now - rate_at), now, 0
            pi_now = receiver.pi_time(got.pc_time)
            if pi_now is not None and frame.header.captured is not None:
                latency_ms = (pi_now - frame.header.captured) * 1000
            color = to_bgr(frame.color_yuyv)
            save_now = snapshots is not None and now - save_at >= args.every
            key = -1
            if not args.no_window:
                counts = receiver.tracker.counts
                view = np.hstack([color, colorize_depth(frame.depth, frame.header.depth_scale)])
                overlay(
                    view,
                    f"{fps:4.1f} fps  latency {latency_ms or 0:5.0f} ms  "
                    f"drop pc {counts.sent_gaps} d {counts.depth_gaps} c {counts.color_gaps}  "
                    f"saved {snapshots.saved if snapshots else 0}",
                )
                cv2.imshow(WINDOW, view)
                shown = True
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27) or window_closed():
                    break
            if snapshots is not None and (save_now or key == ord("s")):
                snapshots.save(frame, color)
                save_at = now if save_now else save_at
        counts = asdict(receiver.tracker.counts)
        bound = receiver.clock_bound(time.perf_counter())
        end = receiver.end.summary if receiver.end is not None else None
    if not args.no_window:
        cv2.destroyAllWindows()
    meta = {
        "host": args.host,
        "port": args.port,
        "started": started.isoformat(timespec="seconds"),
        "ended": datetime.now().isoformat(timespec="seconds"),
        "width": first.header.width if first else None,
        "height": first.header.height if first else None,
        "depth_scale": first.header.depth_scale if first else None,
        "save_every_s": args.every,
        "counts": counts,
        "saved": snapshots.saved if snapshots else 0,
        "last_latency_ms": None if latency_ms is None else round(latency_ms, 1),
        "clock_bound_ms": None if bound is None else round(bound * 1000, 3),
        "sender_end": end,
    }
    if snapshots is not None:
        snapshots.close(meta)
    print(f"끝: 받은 장 {counts['frames']}, 빠진 장 {counts}, 저장 {meta['saved']}장", flush=True)
    return 0 if counts["frames"] > 0 else 1


if __name__ == "__main__":
    sys.exit(main())
