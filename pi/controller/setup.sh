#!/bin/sh
# 컨트롤러 파이의 처음 설정 (docs/decisions/0026-pi-runtime-deploy-records.md의 2, 4, 5, docs/setup.md의 컨트롤러 파이).
# PC의 배포 명령이 이 폴더의 파일을 파이의 임시 폴더에 올려 root로 돌린다: uv run tools/deploy.py setup <파이>
#
#     sh setup.sh check <로그인 사용자>   바꾸지 않고 지금 상태만 본다 (할 것이 있으면 끝 코드 1)
#     sh setup.sh apply <로그인 사용자>   빠진 것을 넣는다. 여러 번 돌려도 같다
#
# 같은 폴더에 배포 명령이 만든 horilab-upload.service(rclone 인자를 tools/sync.py에서 채운 것)와 rclone(공식
# 배포의 linux-arm64 실행 파일, 해시를 맞춘 것)이 있어야 한다.
#
# - 로그인 사용자를 horilab 그룹에 넣는다. 이 그룹은 실행 루프의 SCHED_FIFO와 메모리 잠금 한도, 판 폴더와 기록
#   폴더의 쓰기, 기록 올리기를 바로 돌리는 sudo 규칙을 받는다. 서보 어댑터(dialout)와 게임패드(input)도 연다.
# - 업로드 사용자(horilab-upload)는 서보 장치를 쓰지 못하고(dialout에 넣지 않는다) 판 폴더도 쓰지 못한다
#   (horilab에 넣지 않는다). 드라이브 토큰은 이 사용자만 읽는다.
# - 부팅 인자와 그룹은 다시 켜거나 다시 로그인해야 걸린다.
set -eu

MODE=${1:-}
LOGIN=${2:-}
HERE=$(cd "$(dirname "$0")" && pwd)

GROUP=horilab
UPLOADER=horilab-upload
UPLOAD_HOME=/var/lib/horilab-upload
LIMITS=/etc/security/limits.d/90-horilab.conf
SUDOERS=/etc/sudoers.d/horilab
JOURNALD_DIR=/etc/systemd/journald.conf.d
UNITS=/etc/systemd/system
RCLONE=/usr/local/bin/rclone
# 실행 루프를 코어 3에 두고 인터럽트를 0~2번 코어로 (0026의 2). 컨트롤러의 자가 점검(system.rs의 BOOT_ARGS)과 같다
BOOT_ARGS="isolcpus=3 irqaffinity=0-2"

case "$MODE" in
  check | apply) ;;
  *)
    sed -n '5,6p' "$0"
    exit 2
    ;;
esac
[ "$(id -u)" -eq 0 ] || {
  echo "root로 돌린다: sudo sh $0 $MODE <로그인 사용자>" >&2
  exit 2
}
[ -n "$LOGIN" ] && [ "$LOGIN" != root ] && id "$LOGIN" >/dev/null 2>&1 || {
  echo "로그인 사용자가 없다: '$LOGIN'" >&2
  exit 2
}

TODO=0
RELOAD=0
REBOOT=0
ok() { printf '통과 %s: %s\n' "$1" "$2"; }
need() {
  TODO=$((TODO + 1))
  printf '할 것 %s: %s\n' "$1" "$2"
}
did() { printf '바꿈 %s: %s\n' "$1" "$2"; }

in_group() { id -nG "$1" 2>/dev/null | tr ' ' '\n' | grep -qx "$2"; }

# 파일 하나를 정한 내용(표준 입력)으로 둔다. 바꿨으면 0, 그대로거나 check면 1.
want_file() {
  name=$1
  path=$2
  mode=$3
  tmp=$(mktemp)
  cat >"$tmp"
  if [ -f "$path" ] && cmp -s "$tmp" "$path"; then
    ok "$name" "$path"
    rm -f "$tmp"
    return 1
  fi
  if [ "$MODE" = check ]; then
    need "$name" "$path"
    rm -f "$tmp"
    return 1
  fi
  mkdir -p "$(dirname "$path")"
  install -m "$mode" "$tmp" "$path"
  rm -f "$tmp"
  did "$name" "$path"
  return 0
}

want_dir() {
  path=$1
  mode=$2
  owner=$3
  if [ -d "$path" ] && [ "$(stat -c '%a %U:%G' "$path")" = "$mode $owner" ]; then
    ok 폴더 "$path"
    return
  fi
  if [ "$MODE" = check ]; then
    need 폴더 "$path ($mode $owner)"
    return
  fi
  mkdir -p "$path"
  chown "$owner" "$path"
  chmod "$mode" "$path"
  did 폴더 "$path ($mode $owner)"
}

# 1. 그룹과 로그인 사용자
if getent group "$GROUP" >/dev/null; then
  ok 그룹 "$GROUP"
elif [ "$MODE" = check ]; then
  need 그룹 "$GROUP 그룹이 없다"
else
  groupadd --system "$GROUP"
  did 그룹 "$GROUP"
fi
for g in "$GROUP" dialout input; do
  if in_group "$LOGIN" "$g"; then
    ok "로그인 사용자" "$LOGIN은 $g 그룹이다"
  elif [ "$MODE" = check ]; then
    need "로그인 사용자" "$LOGIN을 $g 그룹에 넣는다"
  else
    usermod -aG "$g" "$LOGIN"
    did "로그인 사용자" "$LOGIN을 $g 그룹에 넣었다 (다시 로그인해야 걸린다)"
  fi
done

# 2. 업로드 사용자
if id "$UPLOADER" >/dev/null 2>&1; then
  ok "업로드 사용자" "$UPLOADER"
elif [ "$MODE" = check ]; then
  need "업로드 사용자" "$UPLOADER가 없다"
else
  useradd --system --user-group --home-dir "$UPLOAD_HOME" --shell /usr/sbin/nologin "$UPLOADER"
  did "업로드 사용자" "$UPLOADER"
fi
for g in dialout "$GROUP"; do
  if in_group "$UPLOADER" "$g"; then
    if [ "$MODE" = check ]; then
      need "업로드 사용자" "$UPLOADER를 $g 그룹에서 뺀다"
    else
      gpasswd -d "$UPLOADER" "$g" >/dev/null
      did "업로드 사용자" "$UPLOADER를 $g 그룹에서 뺐다"
    fi
  fi
done
if id "$UPLOADER" >/dev/null 2>&1 || [ "$MODE" = apply ]; then
  want_dir "$UPLOAD_HOME" 700 "$UPLOADER:$UPLOADER"
fi

# 3. 판 폴더와 기록 폴더 (그룹 쓰기, 새로 만드는 것도 그룹이 따라온다)
want_dir /opt/horilab 2775 "root:$GROUP"
want_dir /opt/horilab/releases 2775 "root:$GROUP"
want_dir /var/lib/horilab 2775 "root:$GROUP"
want_dir /var/lib/horilab/records 2775 "root:$GROUP"

# 4. 실행 루프의 실시간 한도: SCHED_FIFO 80을 쓰고(99는 커널 몫), 메모리를 모두 잠근다 (다시 로그인해야 걸린다)
want_file "실시간 한도" "$LIMITS" 644 <<EOF || true
# horilab: 실행 루프 스레드의 SCHED_FIFO와 메모리 잠금 (docs/decisions/0026의 2). pi/controller/setup.sh가 쓴다
@$GROUP - rtprio 90
@$GROUP - memlock unlimited
EOF

# 5. sudo 규칙: 시험 끝과 끄기 절차에서 기록 올리기를 바로 돌린다 (0026의 5)
tmp=$(mktemp)
cat >"$tmp" <<EOF
# horilab: 기록 올리기를 바로 돌린다 (docs/decisions/0026의 5). pi/controller/setup.sh가 쓴다
%$GROUP ALL=(root) NOPASSWD: /usr/bin/systemctl start horilab-upload.service
EOF
visudo -cqf "$tmp" || {
  echo "sudo 규칙이 틀렸다" >&2
  exit 1
}
want_file "sudo 규칙" "$SUDOERS" 440 <"$tmp" || true
rm -f "$tmp"

# 6. 부팅 인자 (Ubuntu 26.04는 /boot/firmware/current/cmdline.txt, 22.04는 /boot/firmware/cmdline.txt)
has_args() {
  for a in $BOOT_ARGS; do
    printf '%s\n' "$1" | tr ' ' '\n' | grep -qx "$a" || return 1
  done
  [ "$(printf '%s\n' "$1" | tr ' ' '\n' | grep -c -e '^isolcpus=' -e '^irqaffinity=')" -eq 2 ]
}
CMDLINE=
for f in /boot/firmware/current/cmdline.txt /boot/firmware/cmdline.txt; do
  if [ -f "$f" ]; then
    CMDLINE=$f
    break
  fi
done
if [ -z "$CMDLINE" ]; then
  need "부팅 인자" "cmdline.txt를 찾지 못했다"
else
  line=$(cat "$CMDLINE")
  if has_args "$line"; then
    ok "부팅 인자" "$CMDLINE에 $BOOT_ARGS"
  elif [ "$MODE" = check ]; then
    need "부팅 인자" "$CMDLINE에 $BOOT_ARGS를 넣는다"
  else
    [ -f "$CMDLINE.before-horilab" ] || cp -p "$CMDLINE" "$CMDLINE.before-horilab"
    kept=$(printf '%s\n' "$line" | tr ' ' '\n' | grep -v -e '^isolcpus=' -e '^irqaffinity=' -e '^$' | tr '\n' ' ')
    printf '%s%s\n' "$kept" "$BOOT_ARGS" >"$CMDLINE.new"
    mv "$CMDLINE.new" "$CMDLINE"
    did "부팅 인자" "$CMDLINE에 $BOOT_ARGS (옛 줄은 $CMDLINE.before-horilab)"
  fi
  if has_args "$(cat /proc/cmdline)"; then
    ok "부팅 인자 (지금)" "걸려 있다"
  else
    REBOOT=1
    need "부팅 인자 (지금)" "다시 켜야 걸린다"
  fi
fi

# 7. 클럭 정책 performance: 켤 때마다 거는 유닛. Ubuntu의 ondemand 유닛이 있으면 막는다
if want_file "클럭 정책 유닛" "$UNITS/horilab-cpu.service" 644 <"$HERE/horilab-cpu.service"; then
  RELOAD=1
fi
if systemctl list-unit-files ondemand.service 2>/dev/null | grep -q '^ondemand.service'; then
  if [ "$(systemctl is-enabled ondemand.service 2>/dev/null || true)" = masked ]; then
    ok "클럭 정책" "ondemand.service를 막았다"
  elif [ "$MODE" = check ]; then
    need "클럭 정책" "ondemand.service를 막는다"
  else
    systemctl mask ondemand.service >/dev/null 2>&1
    did "클럭 정책" "ondemand.service를 막았다"
  fi
fi

# 8. journald 상한 (프로그램 로그, 0026의 5)
if want_file "로그 상한" "$JOURNALD_DIR/90-horilab.conf" 644 <<EOF; then
# horilab: 프로그램 로그의 상한 (docs/decisions/0026의 5). pi/controller/setup.sh가 쓴다
[Journal]
SystemMaxUse=200M
EOF
  systemctl restart systemd-journald
fi

# 9. 스왑: SD에 스왑을 두지 않는다 (0026의 5). 있으면 알리기만 한다
swaps=$(swapon --show --noheadings 2>/dev/null || true)
if [ -z "$swaps" ]; then
  ok 스왑 "없다"
else
  need 스왑 "있다, 손으로 끈다: $swaps"
fi

# 10. rclone
if [ ! -f "$HERE/rclone" ]; then
  need rclone "올린 rclone이 없다 (배포 명령으로 돌린다)"
elif [ -x "$RCLONE" ] && cmp -s "$HERE/rclone" "$RCLONE"; then
  ok rclone "$RCLONE"
elif [ "$MODE" = check ]; then
  need rclone "$RCLONE을 올린 판으로 둔다"
else
  install -m 755 "$HERE/rclone" "$RCLONE"
  did rclone "$RCLONE"
fi

# 11. 기록 올리기: 몇 분마다 도는 타이머 (2분 넘게 안 바뀐 파일만, 가장 낮은 우선순위)
if want_file "업로드 유닛" "$UNITS/horilab-upload.service" 644 <"$HERE/horilab-upload.service"; then
  RELOAD=1
fi
if want_file "업로드 타이머" "$UNITS/horilab-upload.timer" 644 <"$HERE/horilab-upload.timer"; then
  RELOAD=1
fi
if [ "$RELOAD" -eq 1 ]; then
  systemctl daemon-reload
fi
for unit in horilab-cpu.service horilab-upload.timer; do
  if [ "$(systemctl is-enabled "$unit" 2>/dev/null || true)" = enabled ]; then
    ok 유닛 "$unit 켜짐"
  elif [ "$MODE" = check ]; then
    need 유닛 "$unit을 켠다"
  else
    systemctl enable --now "$unit" >/dev/null 2>&1
    did 유닛 "$unit을 켰다"
  fi
done

# 12. 지금 클럭 정책 (유닛이 걸었는지)
governors=$(cat /sys/devices/system/cpu/cpufreq/policy*/scaling_governor 2>/dev/null | sort -u | tr '\n' ' ')
if [ "$governors" = "performance " ]; then
  ok "클럭 정책 (지금)" performance
else
  need "클럭 정책 (지금)" "${governors:-읽지 못함}"
fi

# 13. 드라이브 로그인 (사람이 브라우저로 한 번: uv run tools/deploy.py login <파이>)
if [ -f "$UPLOAD_HOME/rclone.conf" ]; then
  ok "드라이브 로그인" "$UPLOAD_HOME/rclone.conf"
else
  need "드라이브 로그인" "아직이다: uv run tools/deploy.py login <파이>"
fi

if [ "$REBOOT" -eq 1 ] && [ "$MODE" = apply ]; then
  echo "다시 켜야 부팅 인자가 걸린다: sudo reboot"
fi
if [ "$MODE" = check ] && [ "$TODO" -gt 0 ]; then
  echo "할 것 ${TODO}개"
  exit 1
fi
exit 0
