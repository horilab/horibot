"""PC 쪽 받기 (docs/decisions/0039-camera-stream.md): 카메라 파이가 보내는 영상(zmq PUB)을 받고, 왕복으로 시계를
맞추고, 빠진 장을 센다.

- 영상은 포트, 시계 재기는 포트 + 1이다 (파이의 보내는 프로그램 pi/camera/send.py).
- 같은 뎁스 장이 다시 오면 세고 건너뛴다 (카메라 → 파이에서 뎁스 장 하나가 빠지면 librealsense가 앞 뎁스 장을 새
  컬러 장과 묶어 한 번 더 낸다, 0039).
- 보내는 쪽이 다시 켜지면(보낸 번호가 0으로 돌아감) 이어 받는다. zmq가 다시 붙는다.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass

import zmq

from horibot_camera.clock import ClockFit
from horibot_camera.message import End, Frame, Header, decode

PORT = 5555
QUEUE = 30  # 받는 쪽 큐 (장). 넘으면 zmq가 버려 늦음이 끝없이 늘지 않는다
PING_S = 0.25  # 시계를 묻는 간격


@dataclass
class Counts:
    frames: int = 0  # 받아 넘긴 장
    sent_gaps: int = 0  # 파이 → PC에서 빠진 장 (보낸 번호가 건넌 수)
    depth_gaps: int = 0  # 카메라 → 파이에서 빠진 뎁스 장
    color_gaps: int = 0
    depth_repeats: int = 0  # 같은 뎁스 장이 다시 와서 건너뛴 수
    restarts: int = 0  # 보내는 쪽이 다시 켜진 수


class FrameTracker:
    """장 번호로 빠진 장과 되풀이를 센다."""

    def __init__(self) -> None:
        self.counts = Counts()
        self._last: Header | None = None

    def accept(self, header: Header) -> bool:
        """넘길 장이면 True. 같은 뎁스 장이 다시 오면 세고 False."""
        last = self._last
        if last is not None and header.seq <= last.seq:
            self.counts.restarts += 1
            last = None
        if last is not None:
            self.counts.sent_gaps += header.seq - last.seq - 1
            self.counts.color_gaps += max(0, header.color_no - last.color_no - 1)
            if header.depth_no == last.depth_no:
                self.counts.depth_repeats += 1
                self._last = header
                return False
            self.counts.depth_gaps += max(0, header.depth_no - last.depth_no - 1)
        self._last = header
        self.counts.frames += 1
        return True


@dataclass(frozen=True)
class Received:
    frame: Frame
    pc_time: float  # 다 받은 때 (time.perf_counter)


class Receiver:
    """카메라 파이에 붙어 장을 받는다. with 문으로 쓰거나 close()를 부른다."""

    def __init__(
        self, host: str, port: int = PORT, *, clock_port: int | None = None, queue: int = QUEUE
    ) -> None:
        self.host = host
        self.port = port
        self.tracker = FrameTracker()
        self.clock = ClockFit()
        self.end: End | None = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._ctx = zmq.Context()
        self._sub = self._ctx.socket(zmq.SUB)
        self._sub.setsockopt(zmq.RCVHWM, queue)
        self._sub.setsockopt(zmq.SUBSCRIBE, b"")
        self._sub.setsockopt(zmq.LINGER, 0)
        self._sub.connect(f"tcp://{host}:{port}")
        address = f"tcp://{host}:{clock_port if clock_port is not None else port + 1}"
        self._pinger = threading.Thread(target=self._ping, args=(address,), daemon=True)
        self._pinger.start()

    def _ping(self, address: str) -> None:
        req = self._ctx.socket(zmq.REQ)
        req.setsockopt(zmq.REQ_RELAXED, 1)
        req.setsockopt(zmq.REQ_CORRELATE, 1)
        req.setsockopt(zmq.LINGER, 0)
        req.connect(address)
        try:
            while not self._stop.is_set():
                t1 = time.perf_counter()
                req.send(b"t")
                if req.poll(1000):
                    t2 = float(req.recv())
                    t3 = time.perf_counter()
                    with self._lock:
                        self.clock.add(t1, t2, t3)
                self._stop.wait(PING_S)
        finally:
            req.close(0)

    def receive(self, timeout_s: float) -> Received | None:
        """다음 장을 기다린다. 시간이 지나거나 끝 메시지를 받으면 None (끝 메시지는 self.end에 남는다)."""
        deadline = time.monotonic() + timeout_s
        while True:
            left_ms = max(0, int((deadline - time.monotonic()) * 1000))
            if not self._sub.poll(left_ms):
                return None
            parts = self._sub.recv_multipart(copy=False)
            pc_time = time.perf_counter()
            message = decode([part.buffer for part in parts])
            if isinstance(message, End):
                self.end = message
                return None
            self.end = None
            if self.tracker.accept(message.header):
                return Received(message, pc_time)

    def pi_time(self, pc_time: float) -> float | None:
        """PC 시각(time.perf_counter)을 파이 시각으로. 시계를 한 번도 못 쟀으면 None."""
        with self._lock:
            return self.clock.to_pi(pc_time)

    def clock_bound(self, pc_time: float) -> float | None:
        with self._lock:
            return self.clock.bound_at(pc_time)

    def close(self) -> None:
        self._stop.set()
        self._pinger.join(2)
        self._sub.close(0)
        self._ctx.term()

    def __enter__(self) -> Receiver:
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()
