# CLAUDE.md

horibot: GitHub 조직 horilab의 로봇 레포. 3D 프린터로 출력한 부품과 Feetech 서보로 만드는 6축 로봇팔 호리봇(horibot)을 만든다 (조직과 레포는 [0040](docs/decisions/0040-github-organization.md)). 이전 프로젝트도 레포 이름이 horibot(zzingobomi/horibot)이어서, 그쪽은 문서에서도 말할 때도 늘 "이전 프로젝트"라고 부른다. 문서는 모두 한국어로 쓴다.

## 세션을 시작하면

1. [docs/status.md](docs/status.md)를 읽고 지금 단계와 다음 할 일을 확인한다.
2. [docs/bom.md](docs/bom.md)의 "지금 주문할 것"과 "배송 중"을 보고, 아직 주문하지 않은 것과 도착했을 때가 지난 것을 사용자에게 먼저 짧게 알린다.
3. 아티팩트 "작업 안내"(주소는 status.md)의 db에서 `notes`(사용자가 페이지에 남긴 메모)를 읽는다. 판을 뽑았다, 부품을 못 쓴다 같은 말은 `records`에 적고, 헷갈린 단계는 사용자에게 알린다. 대화창으로 들은 것과 페이지에 남긴 것이 어긋나지 않게 하려는 것이다.
4. 작업에 필요한 주제 문서만 읽는다. 문서 목록은 [README.md](README.md)에 있다.
5. `docs-local/`이 있으면 개인 참고 자료다. 이 폴더가 없는 PC도 있으니, 작업에 꼭 필요한 정보는 여기에 두지 않는다.
6. 처음 쓰는 PC면 [docs/setup.md](docs/setup.md)대로 준비한다.

## 작업 방식

- 구현보다 결정이 먼저다. 작업이 기대는 결정이 docs에 없으면 먼저 정한다.
- 설계(골격, 오차 예산, 팔 부품 형상)는 최신 Fable 모델이 한다. 사용자와는 Opus 세션에서 일하고, 설계는 그 세션에서 Fable 에이전트에 맡긴다. Fable 세션에서 설계 아닌 일과 쌓인 대화까지 맡으면 토큰이 너무 많이 든다.
  - 배치나 모양을 정하고 그리는 일(형상 코드와 설계 검사까지)은 Fable이 한다. 정해진 숫자를 옮기거나 실측값으로 바꾸는 일, 배치와 크기를 바꾸지 않는 작은 모양 고치기(정한 모서리를 둥글게 깎기 같은 것), 문서, 3D 페이지, 작업 안내, 커밋은 Opus가 한다. 숫자를 고치다 배치나 크기까지 바꿔야 하면 거기서 멈추고 Fable에 넘긴다.
  - 띄우기 전에 설명서(정할 것, 실측값, 제약, 돌릴 검사)와 생각 깊이(effort)를 고른 까닭과 함께 보이고 허락을 받는다. 승인한 값으로 띄운다. 설명서에는 원하는 것과 제약을 쓰고 이쪽의 안은 안 하나로만 넣으며, 이 문서와 [design.md](docs/design.md)의 설계 원칙을 읽게 한다.
  - 문제부터 같이 정하거나 취향이 걸린 것은 네 단계로 한다. 사용자와 먼저 정해 설명서를 쓴다. Fable은 먼저 읽고 묻기만 한다(문제를 잘못 잡았는지, 빠진 조건, 정할 갈림길). 답을 받으면 같은 에이전트가 설계하고, 취향이나 정한 조건을 바꿔야 하는 갈림길에서는 멈추고 돌아온다. 결과는 3D 페이지로 보이고, 고칠 것은 같은 에이전트에 넘긴다.
  - Fable이 낸 형상은 Opus가 검사를 다시 돌려 확인한다.
- 고민이 필요한 선택은 대안과 트레이드오프를 따져 근거와 함께 하나를 고른다. 비용과 이미 가진 장비도 같이 따진다. 당연한 관례는 그냥 따른다.
  - 설계는 로봇이 할 일과 그 자리에 필요한 것에서 시작한다. 가진 것(책상 크기, 가진 볼트 등)에 맞추려고 설계를 낮추지 않는다. 가진 것이 맞지 않으면 바꾸거나 맞는 것을 산다.
  - 치수, 배치, 구조처럼 계산과 검토로 답이 나오는 기술 결정은 Claude가 꼼꼼히 따져서 정한다. `docs/decisions/`에 기록하고 결과와 근거를 알린다.
  - 돈이 드는 것, 외형 취향, 목표와 범위처럼 사용자가 정할 것은 추천과 함께 묻고, 승인을 받으면 기록한다.
  - 설계 대안은 한 지표(무게, 되는 자세의 몫 하나)만 보고 고르지 않는다. 정밀도, 기구학과 동역학, 외형, 조립, 케이블과 자잘한 것, 다시 뽑지 않기를 같이 놓고 따진다.
  - 맞춤 정확도, 강도, 목표(정밀한 팔)가 출력 편의(서포트 없이 뽑기 등) 같은 기본 방침보다 먼저다. 기본 방침이 이들과 부딪히면 기본 방침을 접는다. 기본 방침 때문에 형상이나 방법을 바꿀 때는 정확도와 강도를 깎지 않는지 따로 확인해 알린다.
  - 소프트웨어의 방식과 라이브러리(거리 계산, 통신, 저장 형식 같은 것)도 코드를 쓰기 전에 고른다. 지금 쓰임과 앞으로의 쓰임(컨트롤러, 파이, 펜던트, 시뮬레이션), 정확도, 속도, 의존성, 같은 일을 두 벌 만들게 되는지를 놓고 GPU를 쓰는 방법을 포함한 후보를 견주어 하나를 고른다. 근거와 함께 사용자에게 보이고 허락을 받은 뒤 만든다. 후보를 재 보는 일(벤치마크)도 허락 뒤에 한다. "이미 깔려 있다"는 고를 까닭이 아니다. 고른 방식이 문제를 내면 그 자리를 덧대지 말고 고른 것부터 다시 본다. 2026-11-26에 처음 전원 순서의 부딪힘 미리 보기를 깔려 있던 scipy의 KD 트리로 바로 짰다가, 느리고 헛경보가 날 때마다 덧대다 버렸다.
- 이어받은 내용(이전 문서, 이전 프로젝트)도 그대로 믿지 않고 따져 본다. 이전 프로젝트에서는 사실(무엇을 썼고 무슨 일이 있었나)만 참고한다. 결론(원인 진단, 무엇이 낫다)은 그때의 설계나 시험이 잘못됐을 수 있어서 검증되지 않은 가설로 보고, 이 프로젝트의 계산, 시뮬레이션, 실측으로 다시 따진다.
  - 레포의 결정 문서도 그때 사정에서 내린 결정이다. 사정이 바뀌었으면 바뀐 사정으로 다시 판단하고, 다르게 가면 문서를 고친다. "문서에 그렇게 정했다"를 견주는 기준이나 고른 까닭으로 쓰지 않는다.
- 규칙과 절차(이 문서와 conventions.md 포함)는 목적에 맞게 쓴다. 상황에 맞지 않으면 그대로 따르지 말고, 더 나은 방법을 제안하거나 규칙을 고친다.
- 확인하지 않은 사실(부품 사양, 커넥터, 호환성, 가격)은 단정하지 않는다. 사진이나 조사로 확인한 뒤에 말한다. 사용자가 말하지 않은 규칙(특히 돈과 범위)이나 계산하지 않은 기술 주장은 문서에 넣지 않는다. 돈과 범위의 규칙은 사용자가 말한 뜻 그대로 적고, 근거가 어림이면 어림이라고 밝힌다.
- 부품 조사(제품, 품질, 사는 곳, 값, 배송 기간)는 묻지 않고 바로 한다. 조사 에이전트에는 필요한 도구(웹 검색, 페이지·사진 내려받기 등)를 다 쓰게 한다. 사용자가 허용 창을 누르지 않도록 권한은 설정에 미리 둔다. 한 사이트에 자동 요청을 몰아 보내지 않는다 (사용자 인터넷 주소가 그 사이트에서 막힐 수 있다). 그 밖의 웹 검색과 조사는 무엇을 왜 찾을지 말하고 허락을 받은 뒤에만 한다. 레포 문서와 [이전 프로젝트](https://github.com/zzingobomi/horibot) 기록은 묻기 전에 먼저 찾아본다.
- 에이전트(서브에이전트)는 부품 조사가 아니면 띄우기 전에 무엇을 왜 몇 개 맡기는지 말하고 허락을 받는다. 웹을 쓰지 않는 조사도 마찬가지다.
- 질문은 한 번에 1~2개만, 답이 필요한 시점에 한다. 답은 짧고 직접적으로 한다.
  - 사용자가 질문하면 그 답만 한다. 답하면서 시키지 않은 파일 수정이나 설계 변경을 시작하지 않고, 묻지 않은 분석(가진 것과 견주기, 조건, 권고)을 덧붙이지 않는다. 목록을 달라고 하면 목록만 준다. 답하다가 문제를 찾으면 고치지 말고 한 줄로 알린 뒤 지시를 기다린다.
  - 설명은 전문 용어 없이 쉬운 말로, 지금 할 일 위주로 한다. 꼭 필요하지 않은 확인 단계는 꺼내지 않는다. 부품 번호가 든 문장은 무엇을 하는지(뽑는다, 끼워 본다, 산다)를 먼저 쓰고, 살 것이 없으면 그렇게 밝힌다.
- Claude는 실물을 볼 수 없다. 맞는지는 측정값과 사진으로 확인한다. 다시 설계하고 다시 출력하는 일을 계획에 넣지 않고, 출력 전에 확인할 수 있는 것은 모두 확인한다 ([docs/design.md](docs/design.md#출력-전-확인)).
  - 실물 값(측정, 사진)을 기다리며 설계를 멈추지 않는다. 공개 자료(사양서, 도면, 공식 CAD)와 레포 문서, 이전 프로젝트 기록으로 값을 가정해 먼저 그리고, 실물 값이 오면 숫자를 바꿔 검사를 다시 돌린다. 실물이 없는 PC에서 할 수 있는 일을 최대한 해 두려는 것이다.
  - 가정한 값은 코드와 문서에 가정이라고 표시하고, [status.md](docs/status.md)의 사용자 순서에 뽑기 전에 확인할 것으로 모은다. 실물 값이 꼭 있어야 하는 때는 뽑기 직전이다.
  - 가정이 틀리면 숫자가 아니라 배치가 바뀌는 곳은, 그 부분만 고칠 수 있게 그린다.
- 장비와 프로그램이 버티는지(부하, 지연, 통신 시간, 서보 힘과 열)도 같다. 만들기 전에 잴 수 있는 것은 미리 잰다. 재기 전에 통과 기준과, 모자랄 때 고를 길과 그 순서를 정해 둔다. 다 만든 뒤에 장비 한계나 최적화 문제로 되돌아가는 시행착오를 줄이려는 것이다.
- 제작과 조립은 기본 공구만으로 쉽게 할 수 있는 것을 고른다. 대안을 비교할 때 조립 난이도도 같이 따진다 ([docs/design.md](docs/design.md#설계-원칙)).
- 모든 작업 PC에 NVIDIA GPU가 있다. 도구와 방법을 고를 때 GPU를 쓰는 방법을 먼저 따진다 ([docs/software.md](docs/software.md#개발-환경)).
- 실물 로봇을 움직이는 명령은 확인을 받은 뒤에 실행한다 ([docs/hardware.md](docs/hardware.md#안전)).
- 도구 설치 같은 환경 작업은 Claude가 직접 한다.
- 살 것을 놓치지 않게 챙긴다. 설계하다 살 것이 생기면 [0004](docs/decisions/0004-purchasing.md)의 원칙(품질 우선, 국내 배송 우선)대로 조사해 고르고, 고른 까닭(품질, 설계, 값, 배송, 조립)을 사용자에게 보고한다. 승인을 받으면 [부품표](docs/bom.md)에 확정하고 주문했는지 묻는다. 다음 단계로 넘어갈 때도 그 단계에 필요한데 아직 손에 없는 것을 알린다. 주문했다거나 받았다는 말을 들으면 부품표를 고친다.
- 할 일은 생긴 자리에서 [status.md](docs/status.md)에 적는다. 할 일마다 어디서 하는지(실물 없이 되는지, 집에서 실물이 있어야 하는지), 언제까지인지(무엇 전에), 무엇에 기대는지를 같이 적고, 그 순서대로 한다. 실물 단계(조립하는 날, 처음 전원, 시험)를 앞두면 그날 순서를 한 단계씩 따라가며 쓸 프로그램, 파일, 준비물이 다 있는지 맞춰 보고, 빠진 것을 할 일에 더한다. 실물 없이 할 수 있는 일은 그 단계 전에 끝낸다. 2026-11-26에 처음 전원의 할 일을 다시 맞춰 보니, 그날 팔을 움직일 순서 파일이 어깨 시험 하나만 적혀 있었고, 그 움직임이 부딪히지 않는지 미리 볼 도구와 D405 화면을 볼 프로그램이 빠져 있었다.
- 작업 단위가 끝나면 `docs/status.md`를 고치고, /compact 할 때라고 한 줄로 알린다. 다음 일이 전혀 다른 일이면 새 대화를 권한다. 사용자가 한 시간 넘게 자리를 비우기 전(출력을 기다릴 때, 하루를 마칠 때)에도 알린다. 쌓인 대화를 다시 읽는 토큰을 줄이려는 것이다.
- 사용자가 손으로 하는 일(출력, 측정, 배선, 실물 시험)은 Claude가 무엇이 무엇에 기대는지와 급한 정도를 따져 순서를 정하고, 한 번에 한 단계씩 짧게 이끈다: 무엇을, 어떻게, 끝나면 무엇을 알려 줄지. 조립 순서, 사진 찍는 법, 캘리퍼스로 재는 자리 같은 안내는 글로만 하지 않고 작업 안내 사이트에 3D로 보여 준다 ([0036](docs/decisions/0036-work-guide-site.md)). 순서는 [status.md](docs/status.md)의 "사용자 순서"에 두고, 단계가 끝나거나 사정이 바뀌면 고친다.
  - 출력할 부품은 한글 이름 옆에 STL 파일 이름과 바꿀 설정(프리셋, 채움)을 같이 쓰고, 작업 안내에도 부품마다 파일 이름을 보인다. 알리기 직전에 파일을 다시 만들어 지금 코드와 같은지 본다.
  - 서보를 쓰는 단계(조립, 배선, 시험)에는 어느 서보인지 ID를 같이 쓴다 (예: "ID 5 서보"). 작업 안내와 3D 페이지에도 보인다.
  - 조립 안내는 내기 전에, 단계마다 넣거나 끼우는 부품을 그 길을 따라 설계 형상으로 끝까지 밀어 보고 이미 단 부품과 겹치지 않는지 본다. 다 조립한 자리의 겹침, 손이 닿는지, 나중에 가려지는 나사만 따져서는 넣는 길에서 걸리는 것을 잡지 못한다. 2026-11-24 J1에서는 혼에 단 허브가 먼저 넣은 누름 틀의 다리 사이를 지나가지 못해, 조립하던 저녁에 분해해 순서를 바꿨다. 곧게 못 지나가면 순서를 바꾸거나, 기울여 넣는 길을 형상으로 확인해 안내에 그대로 보인다. 나사마다 공구가 머리까지 닿는지도 본다: 2026-11-25에는 먼저 꽂은 ㄱ자 젠더가 파이 판 나사 바로 위를 덮어 렌치가 닿지 않았다. 두 검사는 `guide check`가 돌린다.
  - 사용자가 조립하는 중에 물으면 아는 값과 설계 코드로 바로 답한다. 긴 형상 검사는 답이 지금 할 일을 바꿀 때만, 답한 뒤에 돌린다.
  - 측정이나 실물 시험을 하는 자리에서는 시작하기 전에, 뒤의 설계와 분석에 쓸 값(쓰기 전 상태, 조립 전에만 보이는 것, 화면 값, 사진, 무게)을 빠짐없이 정해 그 자리의 순서에 넣는다. 실물을 다시 준비하는 일이 가장 비싸다.
- 커밋은 바로 하지 않는다. 무엇을 왜 바꿨는지 사용자에게 보고하고, 최종 승인을 받으면 커밋하고 push한다. 커밋은 의미 있는 단위(결정 하나와 그 코드·시험·문서, 또는 기능 하나)로 한다 ([conventions.md](docs/conventions.md#git)). 정하는 중인 것(살 것, 고를 것)이 있으면 정해진 뒤에 관련 변경을 한 번에 보고한다. 커밋하기 전에 git 작성자(user.name, user.email)가 기존 커밋과 같은지 확인한다.

## 자주 쓰는 명령

- 환경 맞추기: `uv sync`
- 커밋 전 검사. 모두 통과해야 커밋한다:
  - `uv run ruff format --check .`
  - `uv run ruff check .`
  - `uv run pyright`
  - `uv run pytest` (CPU 수만큼의 프로세스로 나눠 돌린다, 1분 40초쯤. 파일 하나만 볼 때는 `-n 0`을 붙인다)
  - `cargo fmt --all --check`
  - `cargo clippy --workspace --all-targets -- -D warnings`
  - `cargo test --workspace`
- 팔 설계 검토와 3D 페이지 ([0029](docs/decisions/0029-joint-parts.md)):
  - `uv run python -m horibot_design.parts.wrist` (첫 관절: 겹침, 틈, 무게, 걸침, 층 가운데에 걸친 수평면, 공중에 뽑히는 벽 선, 옆 홈에 너트가 들어가는 길), `... parts.shoulder` (어깨), `... parts.base` (J1), `... parts.flange` (J6 플랜지), `... parts.camera_mount` (D405 받침), `... parts.gripper` (평행 그리퍼, [0030](docs/decisions/0030-parallel-gripper.md)), `... parts.links` (링크)
  - `uv run python -m horibot_design.placement [q1 … q6 (°)]` (부품을 운동학의 관절 각대로 팔 좌표에 놓고 겹침과 틈을 본다)
  - `uv run python -m horibot_design.studies.clamp_plate` (허브 판이 베어링 안쪽 링을 누르는 힘), `... studies.error_budget` (오차 예산)
  - `uv run python -m horibot_design.studies.ur_arm [compare|zone|lengths|view|calib|accel 60]` (팔 전체: 쌓임과 무게, UR3e와 같은 자세 시험, 주 작업 영역, 링크 길이 후보, D405 화면, 손목 카메라의 캘리브레이션 자세, 관절의 가속 상한)
  - `uv run python -m horibot_design.studies.wiring` (배선: 관절을 넘는 선 길이, 구간마다 쓸 선, D405 선, [0031](docs/decisions/0031-wiring.md))
  - `uv run python -m horibot_design.studies.home_view [gaps]` (홈 자세에서 관 글자가 가려지지 않는지, `gaps`는 글자·타이·묶인 선과 다른 부품 사이의 틈, [0019](docs/decisions/0019-ur-look.md))
  - `uv run python -m horibot_design.pages.joint_page`, `... pages.arm_page` (`build/pages/`에 3D 페이지를 만든다: `joint.html`, `shoulder.html`, `base.html`, `wrist_end.html`, `gripper.html`, `arm.html`. 올려 둔 페이지 "STS 관절 구조", "어깨 관절 구조", "J1 관절 구조", "손목 끝 구조", "평행 그리퍼 구조", "horibot 팔 전체"를 고칠 때는 아티팩트 목록에서 제목으로 찾아 같은 주소에 다시 올린다)
- 출력 준비 ([0035](docs/decisions/0035-slicer-automation.md)): `uv run python -m horibot_design.slicing plan --name <묶음> <부품...>` (부품은 `wrist`, `wrist.wrist_body:2`, `coupons.fit3_6806_bores@white`처럼 준다). 값은 `params/print.toml`에서 읽고, 모든 판이 통과해야 `build/print/<묶음>/`에 판마다 3MF를, 레포의 `data/prints/<묶음>/`에 판 계획(`plan.json`)과 판 그림을 낸다. 다시 뽑기와 새 일은 새 묶음 이름으로 낸다. 출력 안내는 작업 안내 사이트에 있다: 묶음을 `guide/guides/prints.py`의 출력 안내(`bundles`)에 더하고 사이트를 다시 만들어 올린다. 화면에서 자른 결과와 견주기(가정 B, 스튜디오나 프리셋 판을 바꾼 뒤 다시 한다)는 `... slicing reference <화면 프로젝트 3MF>`이다. 프로젝트 옆에 화면에서 낸 플레이트 슬라이스 파일 `<이름>.gcode.3mf`를 둔다.
- 작업 안내 사이트 ([0036](docs/decisions/0036-work-guide-site.md)): `uv run python -m horibot_design.guide build`(모든 안내를 검사하고 `build/guide/`에 쓴다), `... guide check`(검사만. 끼우는 길 검사와 나사에 공구가 닿는지도 본다. 조립 안내를 내기 전에 돌린다. 끝낸 안내에 남은 알려진 결과는 `guide/guides/__init__.py`의 `KNOWN`에 까닭과 같이 두고 알리기만 한다), `... guide sweep [안내 id]`(끼우는 길 검사만: 움직임마다 부품을 들어오는 길로 밀어 이미 붙은 부품과 겹치는지 본다. 단계가 관절을 돌려 두면 그 자세로 본다), 나사에 공구가 닿는지(`guide/reach.py`: 그 단계에서 조이는 나사마다 머리 위로 곧은 렌치가 막히는지, 막히면 L자 렌치를 돌릴 방향이 있는지 보고, 일부 방향으로만 돌면 렌치를 그려 넣는 방향을 안내에 보인다), `... guide selftest`(만든 사이트를 헤드리스 크롬으로 열어 모든 단계를 넘겨 본다). 올린 사이트 "작업 안내"를 고칠 때는 `build/guide/index.html`을 같은 주소에 다시 올리고, `manifest.json`과 바뀐 `data/` 파일을 같이 보낸다 (형상 파일 이름은 내용의 해시다). 아티팩트 한 판에는 파일이 511개까지라, 지난 판에만 있는 파일은 `files`에 null로 지우고, 한 번에 보내는 것은 255개 안으로 나눈다 (지울 것과 목록 파일을 먼저 보낸다). 확인 칸은 그 아티팩트 db의 `checks/<안내>`에, 메모는 `notes`에 있다. 확인 칸을 다 누른 단계가 끝낸 단계이고, 사진이 오면 마지막으로 체크한 안내의 아직 안 끝낸 첫 단계를 그 사진의 단계로 본다 (단계를 눌러 보기만 해서는 아무것도 저장되지 않는다). Claude가 남기는 단계 기록은 `records`에 쓴다 (사진은 assets에 올리고 id를 적는다). 판을 뽑았다는 말을 들으면 `{guide: 출력 안내 id, step: 판 단계 id, kind: "printed", at, note}`를, 못 쓰는 부품은 `{guide, step, kind: "bad", part: STL 이름, count, note}`를 쓴다 (사용자는 누르지 않는다). 모자란 부품은 다시 뽑을 묶음을 만들어 그 출력 안내에 더한다.
- 시뮬레이션 서버 ([0033](docs/decisions/0033-planning-and-simulation.md)): 모델을 먼저 `uv run python -m horibot_design.mjcf`로 쓰고(`build/sim/horibot.xml`) `uv run python -m horibot_sim [--port 5600] [--realtime] [--viewer]`로 띄운다. 컨트롤러가 실물 서보 대신 TCP로 붙는다. `--viewer`는 개발 PC에 MuJoCo 창을 띄워 설계 부품 모양의 팔을 그린다: 보기 모델을 먼저 `uv run python -m horibot_design.mjcf_view`로 쓴다(`build/sim/horibot_view.xml`과 `meshes/`, 30초쯤. 부품 형상을 고치면 다시 쓴다). 창에서 숫자 3은 부딪힘 검사의 조각을 켜고 끈다. `cargo test`의 시뮬레이션 통합 시험은 `.venv`의 Python으로 서버를 띄우므로 `uv sync`가 먼저다. 한 박자 시간은 서버를 빠른 모드로 띄운 뒤 `cargo run --release -p horibot-controller --example sim_timing [-- <서버 주소>]`로 잰다.
- D405 화면 보기 ([0039](docs/decisions/0039-camera-stream.md)): 카메라 파이에 올리기는 PC의 Git Bash에서 `tar -cf - --exclude=__pycache__ --exclude=.venv packages/camera pi/camera | ssh hori2 'mkdir -p ~/horibot && tar -xf - -C ~/horibot'`, 보내기는 `ssh hori2 'cd ~/horibot/pi/camera && ~/.local/bin/uv run send.py'`(끝내려면 Ctrl+C나 SIGTERM), PC에서 보기는 `uv run tools/d405_view.py [--host hori2] [--seconds 0] [--no-window]`다. 창에 컬러와 뎁스를 띄우고 1초마다 `records/camera/<날짜>_<시각>_d405-view-hori2/`에 저장한다 (파일 이름은 찍은 때 ms, 파이 시계라 컨트롤러 기록과 바로 견준다)
- 파이용 실행 파일 만들기 ([0026](docs/decisions/0026-pi-runtime-deploy-records.md)): `uv run tools/pi_build.py -p <크레이트> --bin <실행 파일>`
- 컨트롤러 파이 (0026, 절차는 [setup.md](docs/setup.md#9-컨트롤러-파이)): `uv run tools/deploy.py setup|login|deploy|rollback|status <파이>` (파이는 ssh에 주는 이름, 예: hori1). setup, deploy, rollback은 `--yes`가 있어야 바꾼다. `setup --yes`는 파이의 시스템 설정(사용자, 부팅 인자)을 바꾸므로 확인을 받은 뒤에 돌린다. 파이에서 컨트롤러 명령은 `ssh -t hori1 'cd /opt/horilab/current && ./horibot-controller <명령> --port /dev/serial/by-id/<어댑터> --records /var/lib/horilab/records ...'`로 돌리고, `./horibot-controller selfcheck`가 판, 실행 환경, 실시간 설정, 어댑터 경로를 보인다
- 경로 계획과 실행 루프 계산 시간 재기 (0026의 미리 재기 3): `cargo run --release -p horibot-controller --example plan_timing`. 파이에서는 `uv run tools/pi_build.py -p horibot-controller --example plan_timing`로 빌드해 `target/aarch64-unknown-linux-musl/release/examples/plan_timing`을 돌린다
- 부딪힘 검사의 형상 ([0041](docs/decisions/0041-collision-checking.md)): `uv run python -m horibot_design.collision build`(설계가 바뀐 부품만 볼록 조각으로 다시 나눠 `data/collision/`에 쓴다. 처음부터 다 나누면 이 PC로 26분), `... collision check`(자료가 지금 설계와 같은지. pytest도 본다). 부품 형상을 고치면 build를 돌려 같이 커밋한다. 0041의 숫자는 `uv run python -m horibot_design.studies.collision poses` → `cargo run --release -p horibot-controller --example collision_check -- build/collision/poses.json build/collision/pairs.csv`(두 질문의 시간과 조각 거리. 파이에서도 돈다) → `uv run python -m horibot_design.studies.collision compare`(메시 거리와 견준다)로 다시 낸다
- 실물 서보·카메라 테스트는 `uv run pytest --hardware -n 0`이다 (한 프로세스로만 돈다). 실물이 움직이므로 확인을 받은 뒤에만 돌린다.
- 서보 첫 연결(찾기, 덤프, 통신 시간, ID 바꾸기): `uv run python -m horibot_servo ports|scan|dump|timing|set-id` ([docs/servo-setup.md](docs/servo-setup.md)). `timing`과 `set-id`는 서보에 값을 쓰므로(움직이지는 않는다) 확인을 받은 뒤 `--yes`로 돌린다.
- 로봇 설정 확인과 값 적용: `cargo run -p horibot-controller -- settings|ports|apply` ([docs/servo-setup.md](docs/servo-setup.md#값-적용)). `apply --yes`는 서보 EEPROM에 쓰므로(움직이지는 않는다) 확인을 받은 뒤에 돌린다.
- 서보 통신 시간 ([0024](docs/decisions/0024-servo-bus.md)의 만드는 순서 6): `cargo run -p horibot-controller -- timing --port <포트> [--count 300] [--yes]` (컨트롤러 파이에서는 `./horibot-controller timing ...`). `--yes`는 토크가 꺼진 서보의 토크 한도에 읽은 값을 다시 쓰고, 응답 지연이 0이 아니면 잠근 채 0으로 바꾼다 (움직이지 않는다). 확인을 받은 뒤 돌린다.
- 영점 재기와 시험 순서 실행 ([0034](docs/decisions/0034-first-power-tools.md)):
  - 영점 재기: `cargo run -p horibot-controller -- zero --port <포트> --joint <관절> [--yes]`. `--yes`는 위치 오프셋을 굽는다(움직이지는 않는다). 사람이 핀을 꽂으며 Enter를 누르는 일이라 확인을 받은 뒤, 사용자가 보는 창으로 띄운다.
  - 시험 순서 실행: `cargo run -p horibot-controller -- run --script <파일> (--port <포트> | --sim 127.0.0.1:5600) [--yes]`. 실물에서 `--yes`는 서보를 움직이므로 확인을 받은 뒤 돌린다. 시뮬레이션에서 먼저 돌려 본다. 움직이기 전에 지금 팔 자세에서 순서의 이동을 모두 부딪힘 검사로 거른다 (여섯 관절의 영점이 다 있어야 한다. 시험대처럼 형상이 실물과 다를 때만 `--no-collision`). 컨트롤러 파이(리눅스)에서는 실행 루프를 실시간으로 돌리고, 걸지 못하면 토크를 켜기 전에 멈춘다 (`--no-realtime`으로 끈다). 컨트롤러 파이에서 `--yes`로 돌리면 게임패드가 있어야 한다: 사용자가 팔 옆에서 A(시작, 다음)와 B(멈춤)를 누른다 (없이 하려면 `--no-gamepad`). PC에서는 멈춤 단계가 있는 순서를 사용자가 Enter를 누르는 창에서 돌린다. 시뮬레이션에서 볼 때는 `--hold-max 5`로 버티기를 줄인다 (실물에서는 받지 않는다).
  - 처음 전원 순서는 `data/scripts/first-power/`(순서 셋)와 `data/scripts/to-rest.toml`(세운 자리에서 쉬는 자세로)이다 ([0034](docs/decisions/0034-first-power-tools.md)의 처음 전원 순서). 순서를 고치면 `cargo test -p horibot-controller --test first_power_scripts`(부딪힘 검사와 시뮬레이션에서 끝까지)와 `uv run pytest tests/test_first_power_scripts.py`(어깨 버티기 자세가 설계의 IK 해인지)로 본다.
- 맨 서보 확인: `cargo run -p horibot-controller -- check|check-state --port <포트> --id 20` ([docs/servo-setup.md](docs/servo-setup.md#맨-서보-확인)). `check --yes`는 서보를 움직이므로 확인을 받은 뒤, 사용자가 보는 창으로 띄운다 (손으로 하는 단계에서 사용자가 Enter를 누른다). `check-state`는 `--situation <번호>`로 Claude가 돌린다. 한 단계만 다시 할 때는 `--only <단계>`다.
- 시험 기록 올리고 받기 ([0022](docs/decisions/0022-records-and-sync.md)). 실물 시험이 끝나면 올리고, 분석을 시작할 때 받는다:
  - `uv run tools/sync.py push`
  - `uv run tools/sync.py pull` (경로를 주거나 `--max-age 3d`로 골라 받는다)

## 문서 쓰는 법

- 결정, 근거, 사실만 쓴다. 공개 레포라서 대화 내용을 옮기거나 개인에 대한 서술을 넣지 않는다.
- 정보는 한 곳에만 쓰고 다른 곳에서는 링크한다.
- 문서에는 지금 유효한 것만 둔다. 지난 내용과 바뀐 이력은 git 기록과 커밋 메시지에 있다. 바깥에서 가져온 사실은 원문을 확인했는지 같이 적는다.
- 문서, 코드, 테스트, git의 자세한 규칙은 [docs/conventions.md](docs/conventions.md)에 있다.
