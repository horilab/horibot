"""컨트롤러 파이의 처음 설정과 배포 (docs/decisions/0026-pi-runtime-deploy-records.md의 4, 5).

    uv run tools/deploy.py setup <파이> [--yes]     처음 설정: 지금 상태를 보고, --yes면 빠진 것을 넣는다
    uv run tools/deploy.py login <파이>             업로드 사용자의 드라이브 로그인 (브라우저로, 파이마다 한 번)
    uv run tools/deploy.py deploy <파이> [--yes]    컨트롤러를 빌드해 새 판으로 올린다
    uv run tools/deploy.py rollback <파이> [--yes]  바로 앞 판으로 되돌린다
    uv run tools/deploy.py status <파이>            지금 판, 남은 판, 자가 점검, 기록 올리기

- <파이>는 ssh에 주는 이름이다 (예: hori1, 사용자@192.168.0.101). SSH 키로 들어가고 sudo는 비밀번호 없이 된다.
- 처음 설정은 pi/controller/setup.sh를 파이에서 root로 돌린다. 업로드 유닛의 rclone 인자는 tools/sync.py에서
  가져온다. 파이용 rclone은 PC가 공식 배포를 받아 해시를 맞춰 올린다.
- 판마다 /opt/horilab/releases/<날짜시각>-<커밋>/에 실행 파일과 로봇 설정(data/robot.toml), 부딪힘 검사의 형상
  (data/collision/, 0041), 시험 순서(data/scripts/)를 둔다. 임시 폴더에 올려 SHA-256을 맞춘 뒤 판 폴더로 옮기고,
  current 링크는 한 번에 바꾼다.
  새 판이 자가 점검(selfcheck)을 답하지 않으면 앞 판으로 되돌린다. 최근 KEEP개 판만 남긴다.
- 컨트롤러가 돌고 있으면 바꾸지 않는다. --yes가 없으면 할 일만 보여 준다.
- 파이에서 돌릴 때: cd /opt/horilab/current && ./horibot-controller <명령> --records /var/lib/horilab/records ...
"""

from __future__ import annotations

import argparse
import base64
import datetime as dt
import hashlib
import json
import re
import shlex
import shutil
import subprocess
import sys
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:  # uv run tools/deploy.py로 돌 때 tools 패키지를 찾게 한다
    sys.path.insert(0, str(ROOT))

from tools import pi_build, sync  # noqa: E402

PI_SETUP = ROOT / "pi" / "controller"
STAGE = ROOT / "build" / "deploy"
BINARY = "horibot-controller"
PROCESS = BINARY[:15]  # 리눅스가 보이는 프로세스 이름은 15자에서 잘린다
OPT = "/opt/horilab"
RELEASES = f"{OPT}/releases"
CURRENT = f"{OPT}/current"
RECORDS = sync.PI_RECORDS
UPLOAD_HOME = "/var/lib/horilab-upload"
UPLOAD_CONFIG = f"{UPLOAD_HOME}/rclone.conf"
KEEP = 5  # 남길 판 수
RELEASE_NAME = re.compile(r"^\d{8}-\d{6}-[0-9a-f]{7,}(-dirty)?$")
# rclone authorize가 토큰을 감싸는 줄
PASTE_START = "Paste the following into your remote machine --->"
PASTE_END = "<---End paste"


@dataclass(frozen=True)
class Remote:
    """ssh로 닿는 파이."""

    host: str

    def run(
        self, command: str, *, input_text: str | None = None, check: bool = True
    ) -> subprocess.CompletedProcess[str]:
        """파이에서 sh 명령을 돌리고 출력을 받는다."""
        result = subprocess.run(
            ["ssh", "-o", "BatchMode=yes", self.host, command],
            input=input_text,
            capture_output=True,
            text=True,
            encoding="utf-8",
            check=False,
        )
        if check and result.returncode != 0:
            raise SystemExit(
                f"{self.host}에서 실패했다 (끝 코드 {result.returncode}): {command}\n"
                f"{result.stdout}{result.stderr}"
            )
        return result

    def copy(self, sources: Sequence[Path], dest: str) -> None:
        """PC의 파일이나 폴더를 파이의 dest로 올린다."""
        command = [
            "scp",
            "-q",
            "-r",
            "-o",
            "BatchMode=yes",
            *map(str, sources),
            f"{self.host}:{dest}",
        ]
        if subprocess.run(command, check=False).returncode != 0:
            raise SystemExit(f"{self.host}로 올리지 못했다: {dest}")


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def git(*args: str) -> str:
    return subprocess.run(
        ["git", *args], cwd=ROOT, check=True, capture_output=True, text=True
    ).stdout.strip()


def release_name(now: dt.datetime, commit: str, dirty: bool) -> str:
    """판 이름: <날짜시각>-<커밋 7자리>, 커밋하지 않은 변경이 있으면 -dirty."""
    return f"{now:%Y%m%d-%H%M%S}-{commit[:7]}" + ("-dirty" if dirty else "")


def releases_to_prune(names: Sequence[str], current: str | None, keep: int = KEEP) -> list[str]:
    """지울 판: 이름 순(시각 순)으로 최근 keep개와 지금 판을 남긴다. 판 이름 꼴이 아닌 것은 건드리지 않는다."""
    ordered = sorted(n for n in names if RELEASE_NAME.match(n))
    old = ordered[:-keep] if keep > 0 else ordered
    return [n for n in old if n != current]


def previous_release(names: Sequence[str], current: str | None) -> str | None:
    """되돌릴 판: 지금 판 바로 앞(시각 순)의 판."""
    ordered = sorted(n for n in names if RELEASE_NAME.match(n))
    if current not in ordered:
        return ordered[-1] if ordered else None
    i = ordered.index(current)
    return ordered[i - 1] if i > 0 else None


def upload_unit(template: str, args: Sequence[str]) -> str:
    """업로드 유닛: 틀의 @RCLONE_ARGS@를 tools/sync.py의 인자로 채운다."""
    if "@RCLONE_ARGS@" not in template:
        raise SystemExit("업로드 유닛 틀에 @RCLONE_ARGS@가 없다")
    return template.replace("@RCLONE_ARGS@", " ".join(shlex.quote(a) for a in args))


def parse_authorize(output: str) -> str:
    """rclone authorize의 출력에서 토큰(JSON 글)을 꺼낸다. 옛 판은 토큰 JSON을, 새 판은 그것을 감싼 base64를
    붙여 넣을 줄 사이에 준다."""
    match = re.search(re.escape(PASTE_START) + r"\s*(.*?)\s*" + re.escape(PASTE_END), output, re.S)
    if match is None:
        raise SystemExit("rclone authorize의 출력에서 토큰을 찾지 못했다")
    blob = match.group(1).strip()
    data: object
    try:
        data = json.loads(blob)
    except ValueError:
        try:
            data = json.loads(base64.b64decode(blob + "=" * (-len(blob) % 4)))
        except (ValueError, TypeError):
            raise SystemExit("rclone authorize의 토큰을 읽지 못했다") from None
    if isinstance(data, dict) and isinstance(data.get("token"), str):
        data = json.loads(data["token"])
    if isinstance(data, dict) and ("access_token" in data or "refresh_token" in data):
        return json.dumps(data, separators=(",", ":"))
    raise SystemExit("rclone authorize의 토큰 꼴이 다르다")


def pi_config(client_id: str, client_secret: str, token: str) -> str:
    """파이의 업로드 사용자가 쓰는 rclone 설정 (PC의 연결과 같은 앱, 파이마다 따로 받은 토큰)."""
    return (
        f"[{sync.REMOTE}]\ntype = drive\nscope = drive\n"
        f"client_id = {client_id}\nclient_secret = {client_secret}\ntoken = {token}\n"
    )


# --- 처음 설정 -------------------------------------------------------------------------------------


def setup(remote: Remote, yes: bool) -> int:
    rclone = sync.ensure_rclone(sync.PI_PLATFORM)
    template = (PI_SETUP / "horilab-upload.service.in").read_text(encoding="utf-8")
    stage = STAGE / "setup"
    if stage.exists():
        shutil.rmtree(stage)
    stage.mkdir(parents=True)
    for name in ("setup.sh", "horilab-cpu.service", "horilab-upload.timer"):
        shutil.copy2(PI_SETUP / name, stage / name)
    unit = upload_unit(template, sync.pi_upload_args())
    (stage / "horilab-upload.service").write_text(unit, encoding="utf-8", newline="\n")
    shutil.copy2(rclone, stage / "rclone")
    user = remote.run("id -un").stdout.strip()
    folder = remote.run("mktemp -d /tmp/horilab-setup.XXXXXX").stdout.strip()
    try:
        remote.copy(sorted(stage.iterdir()), folder)
        mode = "apply" if yes else "check"
        result = remote.run(
            f"sudo -n sh {shlex.quote(folder)}/setup.sh {mode} {shlex.quote(user)}", check=False
        )
        print(result.stdout, end="")
        print(result.stderr, end="", file=sys.stderr)
    finally:
        remote.run(f"rm -rf {shlex.quote(folder)}", check=False)
    if not yes:
        print("--yes를 붙이면 '할 것'을 넣는다 (드라이브 로그인과 다시 켜기는 따로 한다)")
    elif result.returncode == 0:
        print("그룹은 다시 로그인해야, 부팅 인자는 다시 켜야 걸린다. 다 하면 setup으로 다시 본다")
    return result.returncode


# --- 드라이브 로그인 ---------------------------------------------------------------------------------


def authorize(rclone: sync.Rclone, client_id: str, client_secret: str) -> str:
    """PC의 브라우저로 구글 로그인을 하고 토큰을 받는다. 안내 줄은 보이고 토큰은 화면에 내지 않는다."""
    command = rclone.command("authorize", "drive", client_id, client_secret)
    lines: list[str] = []
    with subprocess.Popen(command, stdout=subprocess.PIPE, text=True, encoding="utf-8") as proc:
        assert proc.stdout is not None
        hiding = False
        for line in proc.stdout:
            lines.append(line)
            if PASTE_START in line:
                hiding = True
            if not hiding:
                print(line, end="")
            if PASTE_END in line:
                hiding = False
    if proc.returncode != 0:
        raise SystemExit(f"구글 로그인을 하지 못했다 (끝 코드 {proc.returncode})")
    return parse_authorize("".join(lines))


def login(remote: Remote) -> int:
    rclone = sync.Rclone(sync.ensure_rclone(), sync.CONFIG, sync.CACHE)
    settings = sync.remote_settings(rclone)
    if not settings or not settings.get("client_id") or not settings.get("client_secret"):
        raise SystemExit("이 PC에서 먼저 로그인한다: uv run tools/sync.py login --client <앱 파일>")
    print(
        "브라우저가 열리면, 데이터를 둘 구글 계정으로 로그인해 허용한다 (이 파이의 업로드 사용자 몫)."
    )
    token = authorize(rclone, settings["client_id"], settings["client_secret"])
    config = pi_config(settings["client_id"], settings["client_secret"], token)
    write = "sudo -n sh -c " + shlex.quote(
        f"umask 077 && cat > {UPLOAD_CONFIG}.new && "
        f"chown horilab-upload:horilab-upload {UPLOAD_CONFIG}.new && "
        f"mv {UPLOAD_CONFIG}.new {UPLOAD_CONFIG}"
    )
    remote.run(write, input_text=config)
    folder = sync.REMOTE_ROOT
    probe = remote.run(
        f"sudo -n -u horilab-upload /usr/local/bin/rclone --config {UPLOAD_CONFIG} "
        f"--cache-dir {UPLOAD_HOME}/cache lsjson --stat {shlex.quote(folder)}",
        check=False,
    )
    if probe.returncode == 0:
        print(f"드라이브의 {folder}에 닿았다. 기록 올리기를 한 번 돌린다")
        remote.run("sudo -n systemctl start --no-block horilab-upload.service", check=False)
        return 0
    print(f"드라이브에 닿지 못했다 (끝 코드 {probe.returncode}):\n{probe.stderr}")
    return 1


# --- 배포 ------------------------------------------------------------------------------------------


def build_release(name: str, commit: str, dirty: bool) -> Path:
    """판 폴더를 PC에 만든다: 실행 파일, 로봇 설정, 부딪힘 검사의 형상, 시험 순서, SHA256SUMS, RELEASE.json."""
    if pi_build.main(["-p", BINARY, "--bin", BINARY]) != 0:
        raise SystemExit("파이용 빌드가 실패했다")
    binary = ROOT / "target" / pi_build.MUSL / "release" / BINARY
    stage = STAGE / name
    if stage.exists():
        shutil.rmtree(stage)
    (stage / "data").mkdir(parents=True)
    shutil.copy2(binary, stage / BINARY)
    shutil.copy2(ROOT / "data" / "robot.toml", stage / "data" / "robot.toml")
    # run은 움직이기 전에 이 형상으로 부딪힘을 검사한다 (없으면 시작하지 않는다)
    shutil.copytree(ROOT / "data" / "collision", stage / "data" / "collision")
    scripts = ROOT / "data" / "scripts"
    if scripts.is_dir():
        shutil.copytree(scripts, stage / "data" / "scripts")
    files = sorted(p for p in stage.rglob("*") if p.is_file())
    sums = "".join(f"{sha256(p)}  {p.relative_to(stage).as_posix()}\n" for p in files)
    (stage / "SHA256SUMS").write_text(sums, encoding="utf-8", newline="\n")
    info = {
        "name": name,
        "commit": commit,
        "uncommitted_changes": dirty,
        "built": dt.datetime.now().astimezone().isoformat(timespec="seconds"),
        "files": [p.relative_to(stage).as_posix() for p in files],
    }
    (stage / "RELEASE.json").write_text(
        json.dumps(info, ensure_ascii=False, indent=2) + "\n", encoding="utf-8", newline="\n"
    )
    return stage


def remote_state(remote: Remote) -> tuple[str | None, list[str]]:
    """파이의 지금 판과 판 목록."""
    out = remote.run(
        f"readlink {CURRENT} || true; echo ---; ls -1 {RELEASES} 2>/dev/null || true"
    ).stdout
    head, _, tail = out.partition("---")
    current = Path(head.strip()).name if head.strip() else None
    return current, [n for n in tail.split() if n]


def ensure_idle(remote: Remote) -> None:
    running = remote.run(f"pgrep -x {PROCESS} || true").stdout.strip()
    if running:
        raise SystemExit(f"컨트롤러가 돌고 있다 (프로세스 {running.split()}). 쉬는 상태에서 바꾼다")


def switch(remote: Remote, name: str) -> None:
    """current 링크를 한 번에 바꾼다 (새 링크를 만들어 이름을 바꾼다)."""
    remote.run(
        f"ln -sfn releases/{shlex.quote(name)} {CURRENT}.new && mv -T {CURRENT}.new {CURRENT}"
    )


def self_check(remote: Remote, commit: str | None) -> bool:
    """지금 판의 자가 점검: 판(커밋)을 답하고 로봇 설정을 읽어야 통과다."""
    result = remote.run(
        f"cd {CURRENT} && ./{BINARY} selfcheck --settings data/robot.toml", check=False
    )
    print(result.stdout, end="")
    if result.returncode != 0:
        print(result.stderr, end="")
        return False
    first = result.stdout.splitlines()[0] if result.stdout else ""
    return commit is None or commit[:7] in first


def deploy(remote: Remote, yes: bool) -> int:
    commit = git("rev-parse", "HEAD")
    dirty = bool(git("--no-optional-locks", "status", "--porcelain"))
    name = release_name(dt.datetime.now(), commit, dirty)
    current, names = remote_state(remote)
    print(f"{remote.host}: 지금 판 {current or '없음'}, 남은 판 {len(names)}개")
    print(f"새 판 {name}" + (" (커밋하지 않은 변경이 들어간다)" if dirty else ""))
    if not yes:
        print("--yes를 붙이면 빌드해 올리고 current를 바꾼다")
        return 0
    ensure_idle(remote)
    stage = build_release(name, commit, dirty)
    incoming = f"{RELEASES}/.incoming-{name}"
    remote.run(f"rm -rf {incoming}")
    remote.copy([stage], incoming)
    remote.run(
        f"cd {incoming} && chmod 755 {BINARY} && sha256sum -c --quiet SHA256SUMS && "
        f"mv {incoming} {RELEASES}/{name}"
    )
    ensure_idle(remote)
    switch(remote, name)
    if not self_check(remote, commit):
        if current:
            switch(remote, current)
            print(f"새 판이 자가 점검을 통과하지 못해 {current}로 되돌렸다")
        else:
            print("새 판이 자가 점검을 통과하지 못했다 (되돌릴 판이 없다)")
        return 1
    for old in releases_to_prune([*names, name], name):
        remote.run(f"rm -rf {RELEASES}/{shlex.quote(old)}")
        print(f"옛 판을 지웠다: {old}")
    print(f"{remote.host}: 지금 판 {name}")
    return 0


def rollback(remote: Remote, yes: bool) -> int:
    current, names = remote_state(remote)
    target = previous_release(names, current)
    print(f"{remote.host}: 지금 판 {current or '없음'}, 되돌릴 판 {target or '없음'}")
    if target is None:
        return 1
    if not yes:
        print("--yes를 붙이면 되돌린다")
        return 0
    ensure_idle(remote)
    switch(remote, target)
    return 0 if self_check(remote, None) else 1


def status(remote: Remote) -> int:
    current, names = remote_state(remote)
    print(f"{remote.host}: 지금 판 {current or '없음'}")
    for n in sorted(names):
        print(f"  {'*' if n == current else ' '} {n}")
    if current:
        self_check(remote, None)
    out = remote.run(
        "systemctl is-active horilab-upload.timer || true; "
        "systemctl show horilab-upload.service -p Result -p ExecMainExitTimestamp --value || true; "
        f"df -h --output=avail {RECORDS} 2>/dev/null | tail -n 1 || true; "
        f"if sudo -n test -f {UPLOAD_CONFIG}; then echo 드라이브 로그인함; else echo 드라이브 로그인 전; fi",
        check=False,
    ).stdout.split("\n")
    lines = [line.strip() for line in out if line.strip()]
    print("기록 올리기: " + " | ".join(lines))
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="uv run tools/deploy.py",
        description="컨트롤러 파이의 처음 설정과 배포 (0026의 4, 5).",
    )
    commands = parser.add_subparsers(dest="command", required=True)
    for name, text, takes_yes in (
        ("setup", "처음 설정: 지금 상태를 보고, --yes면 빠진 것을 넣는다", True),
        ("login", "업로드 사용자의 드라이브 로그인 (브라우저, 파이마다 한 번)", False),
        ("deploy", "컨트롤러를 빌드해 새 판으로 올린다", True),
        ("rollback", "바로 앞 판으로 되돌린다", True),
        ("status", "지금 판, 남은 판, 자가 점검, 기록 올리기", False),
    ):
        command = commands.add_parser(name, help=text)
        command.add_argument("host", metavar="파이", help="ssh에 주는 이름 (예: hori1)")
        if takes_yes:
            command.add_argument("--yes", action="store_true", help="보여 준 대로 바꾼다")
    args = parser.parse_args(argv)
    remote = Remote(args.host)
    if args.command == "setup":
        return setup(remote, args.yes)
    if args.command == "login":
        return login(remote)
    if args.command == "deploy":
        return deploy(remote, args.yes)
    if args.command == "rollback":
        return rollback(remote, args.yes)
    return status(remote)


if __name__ == "__main__":
    sys.exit(main())
