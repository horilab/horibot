"""작업 안내 사이트 만들기.

uv run python -m horilab_design.guide build [--out 폴더]   # 검사하고 build/guide/에 쓴다
uv run python -m horilab_design.guide check                 # 검사만
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

from horilab_design.guide.build import OUT, resolve, write_site
from horilab_design.values import REPO_ROOT


def main() -> None:
    parser = argparse.ArgumentParser(description="작업 안내 사이트를 검사하고 만든다 (0036).")
    sub = parser.add_subparsers(dest="cmd", required=True)
    build = sub.add_parser("build", help="검사하고 사이트 파일을 쓴다")
    build.add_argument("--out", type=Path, default=OUT, help="쓸 폴더 (기본 build/guide)")
    sub.add_parser("check", help="안내 정의를 장면과 부품 목록에 맞춰 검사만 한다")
    test = sub.add_parser(
        "selftest", help="만든 사이트를 헤드리스 브라우저로 열어 모든 단계를 넘겨 본다"
    )
    test.add_argument("--out", type=Path, default=OUT, help="만든 사이트 폴더 (기본 build/guide)")
    args = parser.parse_args()
    start = time.perf_counter()
    if args.cmd == "selftest":
        from horilab_design.guide import selftest

        report = selftest.run(args.out)
        steps = sum(int(g["steps"]) for g in report["guides"])
        found = selftest.problems(report)
        three = "three.js 있음" if report["three"] else "three.js 없음 (3D는 보지 못함)"
        print(f"안내 {len(report['guides'])}개, 단계 {steps}개를 넘겨 봤다 ({three})")
        for p in found:
            print(f"  고칠 것: {p}")
        if found:
            raise SystemExit(1)
    elif args.cmd == "check":
        from horilab_design.guide.guides import ALL

        for g in ALL:
            r = resolve(g)
            work = sum(1 for s in r.steps if s["kind"] == "work")
            print(f"{g.id}: 장면 부품 {len(r.parts)}개, 작업 단계 {work}개")
            for w in r.warnings:
                print(f"  알림: {w}")
    else:
        s = write_site(args.out)
        try:
            shown = s.out.relative_to(REPO_ROOT).as_posix()
        except ValueError:
            shown = str(s.out)
        print(
            f"{shown}: 안내 {len(s.guides)}개, 형상 {s.meshes}개, 파일 {s.files}개, "
            f"{s.bytes / 1024:.0f} KiB"
        )
        for w in s.warnings:
            print(f"  알림: {w}")
    print(f"{time.perf_counter() - start:.1f}초")


if __name__ == "__main__":
    main()
