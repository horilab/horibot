"""카메라 파이 프로그램: D405 영상을 카메라가 USB로 내는 형식 그대로 압축 없이 PC로 보낸다 (docs/decisions/0039).

uv run send.py [--width 640] [--height 480] [--fps 30] [--port 5555]
- 포트: 영상 PUB. 메시지 형식은 horibot_camera.message다.
- 포트 + 1: 시계 재기 REP. 무엇을 받든 파이의 time.time()을 돌려준다.
- 장이 몇 초 동안 오지 않으면 카메라를 다시 연다: 꽂고 처음 켤 때 영상이 오지 않을 때가 있다 (0031).
- 10초마다 보낸 장과 빠진 장을 한 줄 쓴다. Ctrl+C나 SIGTERM이면 끝 메시지를 보내고 끝난다.
"""

from __future__ import annotations

import argparse
import signal
import threading
import time
from pathlib import Path
from typing import Any

import pyrealsense2 as rs
import zmq

from horibot_camera.message import Header, encode, end_message, exposure_middle

QUEUE = 30  # 보내는 쪽 큐 (장). 넘으면 버리므로 메모리와 늦음이 끝없이 늘지 않는다
FIRST_FRAME_MS = 5000  # 카메라를 연 뒤 첫 장을 기다리는 시간
STALL_S = 5.0  # 이만큼 장이 오지 않으면 카메라를 다시 연다
OPEN_TRIES = 4
REPORT_S = 10.0
META = rs.frame_metadata_value


def meta(frame: Any, key: Any) -> float | None:
    return frame.get_frame_metadata(key) if frame.supports_frame_metadata(key) else None


def clock_server(ctx: zmq.Context, port: int, stop: threading.Event) -> None:
    rep = ctx.socket(zmq.REP)
    rep.bind(f"tcp://*:{port}")
    try:
        while not stop.is_set():
            if rep.poll(200):
                rep.recv()
                rep.send(repr(time.time()).encode())
    finally:
        rep.close(0)


def open_camera(width: int, height: int, fps: int) -> tuple[Any, float, dict[str, str]]:
    """카메라를 열어 첫 장이 올 때까지 기다린다. 안 오면 닫고 다시 연다."""
    for attempt in range(1, OPEN_TRIES + 1):
        config = rs.config()
        config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        config.enable_stream(rs.stream.color, width, height, rs.format.yuyv, fps)
        pipe = rs.pipeline()
        profile = pipe.start(config)
        ok, _ = pipe.try_wait_for_frames(FIRST_FRAME_MS)
        if ok:
            device = profile.get_device()
            scale = device.first_depth_sensor().get_depth_scale()
            info = {
                "camera": device.get_info(rs.camera_info.name),
                "firmware": device.get_info(rs.camera_info.firmware_version),
                "usb": device.get_info(rs.camera_info.usb_type_descriptor),
                "tries": str(attempt),
            }
            return pipe, scale, info
        pipe.stop()
        print(
            f"카메라를 열었지만 {FIRST_FRAME_MS} ms 안에 장이 오지 않았다 ({attempt}번째)",
            flush=True,
        )
    raise RuntimeError(f"카메라를 {OPEN_TRIES}번 열었지만 영상이 오지 않았다")


def temperature() -> float | None:
    try:
        return int(Path("/sys/class/thermal/thermal_zone0/temp").read_text()) / 1000
    except (OSError, ValueError):
        return None


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--port", type=int, default=5555)
    args = parser.parse_args()

    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())

    ctx = zmq.Context()
    pub = ctx.socket(zmq.PUB)
    pub.setsockopt(zmq.SNDHWM, QUEUE)
    pub.bind(f"tcp://*:{args.port}")
    clock = threading.Thread(target=clock_server, args=(ctx, args.port + 1, stop), daemon=True)
    clock.start()

    pipe, scale, info = open_camera(args.width, args.height, args.fps)
    print(f"보낸다: {args.width}×{args.height} {args.fps}장, 포트 {args.port}, {info}", flush=True)

    seq = reopened = 0
    gaps = {"depth": 0, "color": 0, "depth_repeats": 0}
    last: tuple[int, int] | None = None
    last_frame = report_at = time.monotonic()
    sent_at_report = 0
    try:
        while not stop.is_set():
            ok, frames = pipe.try_wait_for_frames(1000)
            now = time.monotonic()
            if not ok:
                if now - last_frame > STALL_S:
                    print(f"{STALL_S:.0f}초 동안 장이 오지 않아 카메라를 다시 연다", flush=True)
                    pipe.stop()
                    pipe, scale, info = open_camera(args.width, args.height, args.fps)
                    reopened += 1
                    last = None
                    last_frame = time.monotonic()
                continue
            received = time.time()
            depth, color = frames.get_depth_frame(), frames.get_color_frame()
            if not depth or not color:
                continue
            last_frame = now
            numbers = (depth.get_frame_number(), color.get_frame_number())
            if last is not None:
                gaps["depth"] += max(0, numbers[0] - last[0] - 1)
                gaps["color"] += max(0, numbers[1] - last[1] - 1)
                gaps["depth_repeats"] += numbers[0] == last[0]
            last = numbers
            header = Header(
                seq=seq,
                depth_no=numbers[0],
                color_no=numbers[1],
                width=args.width,
                height=args.height,
                depth_scale=scale,
                captured=exposure_middle(
                    depth.get_timestamp(),
                    meta(depth, META.frame_timestamp),
                    meta(depth, META.sensor_timestamp),
                ),
                received=received,
                sent=time.time(),
            )
            pub.send_multipart(encode(header, depth.get_data(), color.get_data()))
            seq += 1
            if now - report_at >= REPORT_S:
                fps = (seq - sent_at_report) / (now - report_at)
                print(
                    f"보낸 장 {seq} ({fps:.1f}장/초), 카메라에서 빠진 장 뎁스 {gaps['depth']}·컬러 "
                    f"{gaps['color']}, 같은 뎁스 장 {gaps['depth_repeats']}, 다시 연 수 {reopened}, "
                    f"온도 {temperature()} °C",
                    flush=True,
                )
                report_at, sent_at_report = now, seq
    finally:
        summary = {"sent": seq, "gaps": gaps, "reopened": reopened, "info": info}
        for _ in range(3):
            pub.send_multipart(end_message(summary))
            time.sleep(0.1)
        print(f"끝: {summary}", flush=True)
        stop.set()
        pipe.stop()
        clock.join(1)
        pub.close(linger=1000)
        ctx.term()


if __name__ == "__main__":
    main()
