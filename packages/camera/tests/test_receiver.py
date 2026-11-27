"""PC 쪽 받기: 빠진 장 세기와, 가짜 보내는 쪽으로 PC 안에서 보내고 받기 (docs/decisions/0039-camera-stream.md)."""

import threading
import time

import numpy as np
import zmq

from horibot_camera.message import Header, encode, end_message
from horibot_camera.receiver import FrameTracker, Receiver


def _header(seq: int, depth_no: int, color_no: int, width: int = 4, height: int = 2) -> Header:
    now = time.time()
    return Header(seq, depth_no, color_no, width, height, 0.0001, now - 0.05, now - 0.01, now)


def test_tracker_counts_gaps_repeats_and_restarts() -> None:
    tracker = FrameTracker()
    assert tracker.accept(_header(0, 10, 10))
    assert tracker.accept(_header(1, 11, 11))
    assert tracker.accept(_header(3, 12, 12))  # 파이 → PC에서 한 장 빠짐
    assert tracker.accept(_header(4, 14, 14))  # 카메라 → 파이에서 뎁스·컬러 한 장씩 빠짐
    assert not tracker.accept(_header(5, 14, 15))  # 같은 뎁스 장이 새 컬러와 다시 옴: 건너뛴다
    assert tracker.accept(_header(6, 15, 16))
    assert tracker.accept(_header(0, 2, 2))  # 보내는 쪽이 다시 켜짐
    counts = tracker.counts
    assert counts.frames == 6
    assert counts.sent_gaps == 1
    assert counts.depth_gaps == 1
    assert counts.color_gaps == 1
    assert counts.depth_repeats == 1
    assert counts.restarts == 1


def _clock_server(rep: zmq.Socket, stop: threading.Event) -> None:
    while not stop.is_set():
        if rep.poll(50):
            rep.recv()
            rep.send(repr(time.time()).encode())


def test_frames_and_clock_come_through_on_this_pc() -> None:
    ctx = zmq.Context()
    pub = ctx.socket(zmq.PUB)
    rep = ctx.socket(zmq.REP)
    pub_port = pub.bind_to_random_port("tcp://127.0.0.1")
    rep_port = rep.bind_to_random_port("tcp://127.0.0.1")
    stop = threading.Event()
    server = threading.Thread(target=_clock_server, args=(rep, stop), daemon=True)
    server.start()
    depth = np.arange(8, dtype="<u2").reshape(2, 4)
    color = np.arange(16, dtype=np.uint8).reshape(2, 4, 2)
    receiver = Receiver("127.0.0.1", pub_port, clock_port=rep_port)
    try:
        got = []
        seq = 0
        deadline = time.monotonic() + 10
        # 받는 쪽이 붙기 전에 보낸 장은 버려지므로(PUB/SUB) 받을 때까지 계속 보낸다
        while len(got) < 5 and time.monotonic() < deadline:
            pub.send_multipart(
                encode(_header(seq, 100 + seq, 100 + seq), depth.tobytes(), color.tobytes())
            )
            seq += 1
            received = receiver.receive(0.02)
            if received is not None:
                got.append(received)
        assert len(got) == 5
        for received in got:
            assert np.array_equal(received.frame.depth, depth)
            assert np.array_equal(received.frame.color_yuyv, color)
        # 같은 PC라 "파이 시계"는 time.time(), PC 시계는 time.perf_counter다
        deadline = time.monotonic() + 5
        while receiver.pi_time(time.perf_counter()) is None and time.monotonic() < deadline:
            time.sleep(0.05)
        pc = time.perf_counter()
        pi = receiver.pi_time(pc)
        bound = receiver.clock_bound(pc)
        assert pi is not None and bound is not None
        assert abs(pi - time.time()) < bound + 0.005
        # 보내는 쪽이 멈추면 끝 메시지가 남는다
        deadline = time.monotonic() + 5
        while receiver.end is None and time.monotonic() < deadline:
            pub.send_multipart(end_message({"sent": seq}))
            receiver.receive(0.05)
        assert receiver.end is not None and receiver.end.summary == {"sent": seq}
    finally:
        receiver.close()
        stop.set()
        server.join(1)
        pub.close(0)
        rep.close(0)
        ctx.term()
