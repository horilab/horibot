"""tools/deploy.py와 pi/controller/ 검사 (docs/decisions/0026-pi-runtime-deploy-records.md의 2, 4, 5).

파이에 닿는 일(ssh, scp)은 실물이 있는 PC에서 hori1에 올려 본다. 여기서는 판 이름과 지울 판 고르기, 업로드
유닛, 드라이브 토큰 꺼내기, 처음 설정 스크립트와 컨트롤러 자가 점검이 같은 값을 쓰는지를 본다.
"""

import base64
import datetime as dt
import json
import re
import shutil
import subprocess
from pathlib import Path

import pytest

from tools import deploy, sync

ROOT = Path(__file__).resolve().parents[1]
SETUP = ROOT / "pi" / "controller" / "setup.sh"
SYSTEM_RS = ROOT / "crates" / "controller" / "src" / "system.rs"


def test_release_names_sort_by_time_and_mark_uncommitted_changes() -> None:
    now = dt.datetime(2026, 11, 26, 21, 5, 9)
    name = deploy.release_name(now, "e500b0c35dbcf4b15322638148aeb404ed77066b", False)
    assert name == "20261126-210509-e500b0c"
    assert deploy.RELEASE_NAME.match(name)
    dirty = deploy.release_name(now, "e500b0c35dbcf4b", True)
    assert dirty.endswith("-dirty") and deploy.RELEASE_NAME.match(dirty)


def test_pruning_keeps_recent_releases_and_the_current_one() -> None:
    names = [f"202611{day:02d}-120000-abcdef0" for day in range(20, 28)]
    current = names[0]  # 되돌려 둔 옛 판
    old = deploy.releases_to_prune([*names, "손으로 둔 폴더"], current, keep=3)
    assert old == names[1:5]  # 최근 셋(25, 26, 27일)과 지금 판(20일)을 남긴다
    assert deploy.releases_to_prune(names[:2], names[1], keep=5) == []


def test_rollback_goes_to_the_release_just_before_the_current_one() -> None:
    names = ["20261125-100000-aaaaaaa", "20261126-100000-bbbbbbb", "20261127-100000-ccccccc"]
    assert deploy.previous_release(names, names[2]) == names[1]
    assert deploy.previous_release(names, names[0]) is None
    assert deploy.previous_release(names, None) == names[2]
    assert deploy.previous_release([], None) is None


def test_upload_unit_takes_its_rclone_flags_from_the_sync_tool() -> None:
    template = (ROOT / "pi" / "controller" / "horilab-upload.service.in").read_text(
        encoding="utf-8"
    )
    unit = deploy.upload_unit(template, sync.pi_upload_args())
    exec_line = next(line for line in unit.splitlines() if line.startswith("ExecStart="))
    assert "@RCLONE_ARGS@" not in unit
    assert f"copy {sync.PI_RECORDS} {sync.REMOTE_ROOT}/records --update" in exec_line
    assert f"--min-age {sync.PI_UPLOAD_MIN_AGE}" in exec_line
    assert f"--bwlimit {sync.PI_UPLOAD_BWLIMIT}" in exec_line
    assert f"--config {deploy.UPLOAD_CONFIG}" in exec_line
    # 업로드 사용자는 서보 장치와 판 폴더를 쓰지 못한다 (setup.sh가 그룹을 뺀다)
    assert "User=horilab-upload" in unit
    assert f"ReadWritePaths={deploy.UPLOAD_HOME}" in unit
    with pytest.raises(SystemExit):
        deploy.upload_unit("[Service]\n", ["copy"])


def _authorize_output(blob: str) -> str:
    return (
        "If your browser doesn't open automatically go to the following link: http://127.0.0.1:53682/auth\n"
        "Log in and authorize rclone for access\nWaiting for code...\nGot code\n"
        f"{deploy.PASTE_START}\n{blob}\n{deploy.PASTE_END}\n"
    )


def test_drive_token_is_read_from_old_and_new_authorize_output() -> None:
    token = {
        "access_token": "a",
        "token_type": "Bearer",
        "refresh_token": "r",
        "expiry": "2026-11-26",
    }
    raw = json.dumps(token)
    assert json.loads(deploy.parse_authorize(_authorize_output(raw))) == token
    wrapped = base64.b64encode(json.dumps({"token": raw}).encode()).decode().rstrip("=")
    assert json.loads(deploy.parse_authorize(_authorize_output(wrapped))) == token
    with pytest.raises(SystemExit):
        deploy.parse_authorize("Waiting for code...\n")
    with pytest.raises(SystemExit):
        deploy.parse_authorize(_authorize_output('{"something": 1}'))


def test_pi_config_names_the_same_remote_as_the_sync_tool() -> None:
    text = deploy.pi_config("id", "secret", '{"access_token":"a"}')
    assert text.startswith(f"[{sync.REMOTE}]\ntype = drive\n")
    assert 'token = {"access_token":"a"}' in text


def _rust_const(name: str) -> str:
    match = re.search(
        rf"pub const {name}: [^=]+= (.+?);", SYSTEM_RS.read_text(encoding="utf-8"), re.S
    )
    assert match, name
    return match.group(1)


def test_setup_and_the_controller_self_check_agree() -> None:
    script = SETUP.read_text(encoding="utf-8")
    boot = re.search(r'^BOOT_ARGS="(.+)"', script, re.M)
    assert boot
    rust_args = re.findall(r'"([^"]+)"', _rust_const("BOOT_ARGS"))
    assert boot.group(1).split() == rust_args
    # 실행 루프의 우선순위가 한도 안이다
    rtprio = re.search(r"rtprio (\d+)", script)
    assert rtprio and int(rtprio.group(1)) >= int(_rust_const("LOOP_PRIORITY"))
    assert "memlock unlimited" in script
    # 배포 명령과 같은 자리를 쓴다
    for path in (deploy.UPLOAD_HOME, "/opt/horilab/releases", deploy.RECORDS):
        assert path in script, path
    assert deploy.RECORDS == "/var/lib/horilab/records"


@pytest.mark.skipif(shutil.which("sh") is None, reason="sh가 없다")
def test_setup_script_parses() -> None:
    sh = shutil.which("sh")
    assert sh
    assert subprocess.run([sh, "-n", str(SETUP)], check=False).returncode == 0


def test_pi_files_use_lf_line_endings() -> None:
    for path in (ROOT / "pi" / "controller").iterdir():
        assert b"\r\n" not in path.read_bytes(), path.name
