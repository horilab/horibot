"""만든 사이트를 브라우저로 열어 점검한다 (0036의 브라우저 시험).

작은 로컬 서버로 build/guide/를 띄우고, PC에 깔린 크롬(없으면 엣지)을 헤드리스로 열어 #selftest를 돌린다.
페이지가 모든 안내의 모든 단계를 넘겨 보고 결과를 <pre id="selftest-report">에 적으면, 그 글을 읽어 돌려준다.
새 패키지나 PC 전체 설치를 쓰지 않는다. three.js는 인터넷(cdnjs)에서 받는다.
"""

from __future__ import annotations

import contextlib
import functools
import html
import json
import os
import re
import shutil
import subprocess
import tempfile
import threading
from collections.abc import Iterator
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

_CANDIDATES = (
    r"C:\Program Files\Google\Chrome\Application\chrome.exe",
    r"C:\Program Files (x86)\Google\Chrome\Application\chrome.exe",
    r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe",
    r"C:\Program Files\Microsoft\Edge\Application\msedge.exe",
)
_REPORT = re.compile(r'<pre id="selftest-report" hidden(?:="")?>([^<]*)</pre>')


def find_browser() -> str | None:
    """크롬이나 엣지의 실행 파일. 없으면 None."""
    for path in _CANDIDATES:
        if os.path.exists(path):
            return path
    for name in ("google-chrome", "chromium", "chromium-browser", "chrome", "msedge"):
        found = shutil.which(name)
        if found:
            return found
    return None


class _Quiet(SimpleHTTPRequestHandler):
    # 아티팩트는 페이지를 UTF-8로 감싸 보낸다. 로컬 서버도 글자 판을 밝혀야 한국어가 깨지지 않는다
    extensions_map = {  # noqa: RUF012 (부모와 같은 꼴)
        **SimpleHTTPRequestHandler.extensions_map,
        ".html": "text/html; charset=utf-8",
        ".json": "application/json; charset=utf-8",
    }

    def log_message(self, format: str, *args: Any) -> None:
        pass


class _Server(ThreadingHTTPServer):
    # 페이지는 형상 파일 수십 개를 한꺼번에 받는다. 연결 대기 자리가 기본값(5)이면 검사로 CPU가 바쁠 때
    # 서버가 늦게 받아 연결이 거절되고 페이지에는 "Failed to fetch"로 나온다
    request_queue_size = 128


@contextlib.contextmanager
def serve(folder: Path) -> Iterator[str]:
    """folder를 127.0.0.1의 빈 포트로 띄우고 주소를 준다."""
    handler = functools.partial(_Quiet, directory=str(folder))
    server = _Server(("127.0.0.1", 0), handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        yield f"http://127.0.0.1:{server.server_address[1]}/"
    finally:
        server.shutdown()
        server.server_close()


def run(folder: Path, browser: str | None = None, timeout: float = 240.0) -> dict[str, Any]:
    """folder(만든 사이트)를 점검하고 페이지가 적은 보고를 돌려준다. 브라우저가 없으면 RuntimeError."""
    exe = browser or find_browser()
    if exe is None:
        raise RuntimeError("크롬이나 엣지를 찾지 못했다")
    with tempfile.TemporaryDirectory(prefix="guide-selftest-") as profile, serve(folder) as url:
        args = [
            exe,
            "--headless=new",
            "--use-angle=swiftshader",
            "--enable-unsafe-swiftshader",
            "--no-first-run",
            "--no-default-browser-check",
            "--disable-extensions",
            f"--user-data-dir={profile}",
            "--window-size=1600,900",
            "--virtual-time-budget=120000",
            "--dump-dom",
            url + "index.html#selftest",
        ]
        out = subprocess.run(
            args,
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=timeout,
        )
    found = _REPORT.findall(out.stdout or "")
    if not found:
        tail = (out.stderr or "")[-1500:]
        raise RuntimeError(f"점검 보고가 페이지에 없다 (브라우저 끝난 값 {out.returncode})\n{tail}")
    return json.loads(html.unescape(found[-1]))


def problems(report: dict[str, Any]) -> list[str]:
    """보고에서 고칠 것만 모은다."""
    out = [str(e) for e in report.get("errors", [])]
    for g in report.get("guides", []):
        out += [f"{g['id']}: {p}" for p in g.get("problems", [])]
    return out
