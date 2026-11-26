# 참고 설계

다른 데스크톱 로봇팔에서 가져올 점과, 만든 사람들이 겪은 문제를 모았다. 설계를 따라 하려는 것이 아니다. 부품을 설계하기 전에 시행착오를 줄이려고 봤다. 호리봇을 견주는 기준 모델(UR3e)도 여기에 둔다.

조사는 2026-11에 했다.
- GitHub 문서와 이슈, 제조사 문서, MakerWorld 모델 67개와 그 댓글을 봤다.
- MakerWorld는 웹 API로 읽었다. Printables는 페이지가 막혀 검색 결과만 봤다.
- 숫자는 출처에 적힌 값이고, 직접 재 본 것은 아니다.

## 기준 모델: UR3e

호리봇이 할 수 있는 자세와 동작을 견주는 기준이다 ([goals.md](goals.md)). 따라 그리는 대상은 아니다. 아래 값은 2026-11-12에 UR 공식 자료에서 확인했다.

- 출처
  - [UR3e 사용자 매뉴얼 5.8판](https://s3-eu-west-1.amazonaws.com/ur-support-site/69043/99403_UR3e_User_Manual_en_Global.pdf): 부록 D 기술 사양, 4.2절 작업 공간, 11.1.5절 특이점
  - UR 홈페이지의 [DH 값](https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/)
  - UR이 GitHub에 공개한 로봇 모형 [Universal_Robots_ROS2_Description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description)의 `config/ur3e/`와 `meshes/ur3e/`. DH 값은 홈페이지와 같다.
- 공개 모형의 UR3e 파일은 BSD-3-Clause다. UR15, UR20 같은 새 모델의 형상만 UR의 별도 약관을 따른다. 레포에 넣으면 라이선스 문구를 같이 둔다.

| 항목 | UR3e |
|---|---|
| 관절 범위 | 플랜지(손목 3)는 끝없이 돌고, 나머지는 ±360°다. 다만 팔꿈치는 어깨 관절에 막혀 약 ±180°를 넘지 못한다. 그래서 공개 모형은 팔꿈치를 ±180°로 묶었다 (`joint_limits.yaml`의 주석). 공개 충돌 형상으로 재면 약 155°다 ([0027](decisions/0027-general-purpose-review.md#ur3e와-견주기)) |
| 관절 속도 | 손목 셋은 360°/s, 나머지 셋은 180°/s다. 공구 끝은 약 1 m/s다 |
| 짐 | 3 kg |
| 닿는 거리 | 베이스 관절에서 500 mm |
| 반복 정밀도 | ±0.03 mm (ISO 9283) |
| 무게 | 11.1 kg |
| 베이스 지름 | 128 mm |
| 작업 공간 | 베이스 바로 위와 아래의 원기둥 둘레는 피한다. 공구가 천천히 움직여도 관절이 빨리 돈다. 특이점은 팔꿈치가 다 펴질 때, TCP가 베이스 축에 가까울 때, 손목 2 관절이 손목 1 관절과 정렬될 때다 |

축 배치는 호리봇과 같다: J2∥J3∥J4, J5⊥J4, J6⊥J5 ([0013](decisions/0013-wrist-type.md)). 치수는 아래와 같다 (UR3e는 DH 값, 호리봇은 [arm.toml](../params/arm.toml)).

| 치수 | UR3e [mm] | 호리봇 [mm] | 호리봇 / UR3e |
|---|---|---|---|
| 설치면 → 어깨 축 (d1) | 151.85 | 160 | 1.05 |
| 위팔 (a2) | 243.55 | 220 | 0.90 |
| 아래팔 (a3) | 213.2 | 180 | 0.84 |
| J4 축 방향 옆 오프셋 합 (d4) | 131.05 | 79.21 | 0.60 |
| J5 축 방향 (d5) | 85.35 | 70.31 | 0.82 |
| J6 → 플랜지 (d6) | 92.1 | 32.6 | 0.35 |

- 위팔과 아래팔은 UR3e의 0.84~0.90배이고, 손목은 비율로 더 짧다.
- UR3e는 옆 오프셋을 어깨, 팔꿈치, 손목에 나눠 둔다. 공개 모형에서 위팔 형상은 베이스 축을 지나는 면에서 옆으로 120 mm, 아래팔 형상은 27 mm 자리에 놓인다 (`visual_parameters.yaml`). 둘이 약 93 mm 비켜 있어서 아래팔이 위팔 옆을 지나 접힌다.
- 호리봇도 자식이 출력 원판 위에 앉아 관절마다 옆으로 비킨다. 베이스 축을 지나는 면에서 위팔 관 가운데는 55.5 mm, 아래팔 관 가운데는 23.7 mm 자리라, 둘이 약 32 mm 비켜 있어 아래팔이 위팔 옆을 지나 접힌다 ([0029](decisions/0029-joint-parts.md), `uv run python -m horibot_design.studies.ur_arm`의 옆 자리).

## 참고한 팔

| 팔 | 구동 | 가져올 점 |
|---|---|---|
| [SO-ARM100 / SO-101](https://github.com/TheRobotStudio/SO-ARM100) | STS3215 6개 (5축 + 그리퍼) | 서보 자리 시험편(게이지)을 먼저 출력한다. 관절마다 출력 혼과 뒷면 혼을 다 쓴다 (손목 롤만 한쪽). 케이블 클립, 손목 D405 마운트 |
| [SO-ARM-102](https://github.com/roboninecom/SO-ARM-102) | STS3235, STS3250, STS3215 | 서보 대신 출력한 서보 모형을 넣어 프레임 휨만 따로 쟀다. 팔을 다 펴고 300 g을 달면 SO-101 프레임은 7.5 mm, 위상 최적화한 PET-CF 링크는 3.4~3.9 mm 처졌다 ([사양](https://github.com/roboninecom/SO-ARM-102/blob/main/docs/specifications.md)). 알려진 문제: 드라이버가 닿지 않는 M2 나사가 있다. 관절을 닫기 전에 케이블 여유를 확인해야 한다 |
| [ElRobot](https://github.com/norma-core/norma-core/tree/main/hardware/elrobot) | ST3215 8개 (7축 + 그리퍼) | 나사를 두 종류만 쓴다. 배선할 때마다 관절이 끝까지 돌아가는지 확인한 뒤 덮개를 나사로 덮는다. 부품마다 글자를 붙이고, 서보 ID는 조립 전에 정한다. 그림 설명서가 75단계다 |
| [AM-ARM200](https://github.com/liyiteng/AM-ARM) | STS3215 등 (6축 + 그리퍼) | 나사는 M3×10 한 종류다. 나사를 모두 느슨하게 끼워 맞춘 뒤에 조인다. 손목 케이블은 덮기 전에 넣고, 케이블이 당기지 않는지 관절을 범위 끝까지 돌려 본다 ([조립](https://github.com/liyiteng/AM-ARM/blob/main/am-arm200/hardware_assembly.md)) |
| [ArmDojo](https://github.com/miraclewhite-tech/armdojo-hardware) | STS3215 6개 (5축) | 모든 관절에서 두 혼에 판을 대고 두 판 사이를 가로대로 잇는다. M3 너트를 홈에 넣는다. 맞춤에 중요한 출력 설정(구멍 보정, 첫 층 퍼짐 보정)을 적어 둔다. 너트 홈과 혼 면에는 서포트를 두지 않는다. 출력해 만든 사람은 아직 없다 |
| [so101-toolchanger](https://github.com/chaseab/so101-toolchanger) | SO-101용 도구 교체 | 3×12 mm 강철 핀 세 개가 V홈에 앉고, 손목 롤을 45° 돌려 잠근다. 도구 거치대에서 200번 꽂아 199번 됐다 |
| [Koch v1.1](https://github.com/jess-moss/koch-v1-1) | Dynamixel | 플라스틱에 바로 박던 나사를 너트로 바꿨다. 스냅 끼움은 프린터마다 맞춤이 달라 나사로 바꿨다 |
| [Niryo One](https://niryo.com/niryo-one-upgraded-version/) | J1~J3 스테퍼, J4~J6 Dynamixel | 팔 안으로 지나가는 케이블을 두 가닥으로 줄여 연결 문제를 줄였다. 영점은 출력한 표시를 맞춰 잡는다 |
| [PAROL6](https://docs.source-robotics.com/PAROL-docs/latest/) | 스테퍼와 감속기 | 공압 튜브가 케이블을 잡아 주는 등뼈 노릇을 한다. 영점을 잡은 뒤, 핀이 모든 관절의 표시 구멍을 지나는지 본다. J1에 멈춤을 둬서 한 바퀴 넘게 돌다가 선이 뜯기지 않게 한다. 모터 옆 부품은 60~70°C까지 올라 PETG로 뽑는다 |
| [SO-101 책상 클램프](https://makerworld.com/en/models/1796562) | 부속 | 공구 없이 손으로 돌리는 출력 나사다. 핀 두 개로 베이스 자리를 잡고, 미끄럼 방지 TPU 패드를 댄다 |
| myCobot 280 (Elephant Robotics) | STS3215 급 서보 6개 (45.2 × 24.7 × 35 mm, 19.5 kg·cm @ 7.4 V, 4096 절대 위치), UR형 | 같은 급 서보로 만든 UR형 팔이다 (2026-11-12 확인).<br>- 관절 범위: J1 ±168°, J2 ±140°, J3·J4 ±150°, J5 -155°~+160°, J6 ±180° (공식 로봇 모형 [mycobot_280_m5.urdf](https://github.com/elephantrobotics/mycobot_ros/blob/noetic/mycobot_description/urdf/mycobot_280_m5/mycobot_280_m5.urdf))<br>- 치수: 위팔 110.4, 아래팔 96, 손목 오프셋 64.6 · 73.2 · 45.6 mm<br>- 공식 형상 파일로 보면 링크는 두께 약 53 mm인 납작한 캡슐이고, 양 끝 관절 집 지름이 48 mm다<br>- 작업 반지름 280 mm, 짐 250 g, 반복 정밀도 ±0.5 mm, 무게 850 g ([매뉴얼](https://robu.in/wp-content/uploads/2024/06/myCobot-en-3.pdf), 판매처가 올린 사본)<br>- 구조 부품에 베어링이 있다고만 적혀 있다. 관절을 받치는 방식과 선이 관절을 지나는 길은 공개 자료에서 찾지 못했다. 관절 연결부 특허는 [CN208196840U](https://patents.google.com/patent/CN208196840U/en), [CN208196791U](https://patents.google.com/patent/CN208196791U/en)다 |

서보를 쓰는 팔 중에 J1에 크로스 롤러 베어링을 쓰거나 ST-3120을 쓰는 예는 찾지 못했다.

## 쉬는 자세

전원을 끌 때 팔을 어떻게 두는지 찾아봤다 ([0037](decisions/0037-rest-pose.md)). 2026-11-21에 검색 결과의 요약으로 봤고, 원문은 열어 읽지 않았다.

- UR: 관절마다 브레이크가 있다. 끌 때와 비상 정지 때 모터 전원을 끊기 전에 브레이크가 걸렸는지 확인하고, 브레이크가 고장 나면 그 관절이 팔을 버티지 못한다고 설명서에 적혀 있다 ([Safe Brake System](https://www.universal-robots.com/manuals/EN/HTML/SW5_24/Content/prod-serv-man/UR20/serv-man-safe-brake.htm)). 끌 때 팔이 처지는 UR10e 사용자에게 홈 자세 근처에 도구를 받치는 받침대를 만들라는 답이 있었다 ([포럼](https://forum.universal-robots.com/t/arm-drops-at-power-off/27343)).
- Interbotix(Trossen) X 시리즈: 잠자는 자세(sleep pose)를 설정 파일에 두고, 토크를 끄기 전에 그 자세로 보낸다. 그 자세가 아니면 팔이 무너진다고 적혀 있다 ([xs_sdk](https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros_interface/ros2/overview/xs_sdk.html), [ALOHA 켜고 끄기](https://docs.trossenrobotics.com/aloha_docs/operation/bringup_shutdown.html)). 그 자세에서 팔이 어디에 얹히는지는 찾지 못했다.
- SO-101 (LeRobot): 연결을 끊으면 기본으로 토크를 끈다 (`disable_torque_on_disconnect`). 받침이나 쉬는 절차는 찾지 못했다 ([문서 미러](https://www.mintlify.com/huggingface/lerobot/robots/so-101)).
- AL5D (취미용 서보 팔): 전원을 끄면 뻗은 팔이 떨어지니, 끄기 전에 천천히 쉬는 자세로 보내라는 답이 있었다 ([포럼](https://community.robotshop.com/forum/t/position-of-arm-when-power-is-off/25511)).

## 외형

협동로봇처럼 보이게 하는 방법이다. 이 팔은 UR식으로 정했다 ([0019](decisions/0019-ur-look.md)).

- **UR식** ([UR3e](https://www.universal-robots.com/products/ur3e/))
  - 관절마다 둥근 집이 있고, 양쪽 끝에 색 뚜껑이 있다. 뚜껑은 나사로 고정하고, 관절을 손볼 때 연다.
  - 이음매마다 검은 띠(실링 고리)가 있고, 관절 사이는 민무늬 관이다. 베이스는 원뿔이고, 플랜지는 ISO 9409다.
  - 요소마다 한 색 부품으로 나눠 뽑을 수 있다.
  - [UR5e 1:5 출력 모형](https://makerworld.com/en/models/3408581)은 몸체를 한 색으로 뽑고 캡만 따로 뽑는다. 검은 띠 대신 얕은 홈을 팠는데, 사진에서는 그림자 선 정도로만 보인다.
- **서보로 움직이는 팔**
  - [myCobot 280](https://robotsguide.com/robots/mycobot)은 서보를 굽은 관 조각 안에 넣어서, 관절이 분할선으로만 보인다.
  - [RIG-Arm](https://makerworld.com/en/models/2913469)은 끝이 둥근 사각 단면 링크로 서보를 감싼다. 벽은 1.2 mm다.
  - [Kinova Gen3 lite](https://www.roscomponents.com/product/gen3-lite/)는 링크 끝을 관절 축 중심의 반원으로 만든다.
- **매끈한 한 덩어리 껍데기**(Franka, Kinova, Lite 6)는 큰 곡면이라 서포트가 많이 들고, 열기 어렵다.
- **주의할 것**
  - 열:
    - STS3215는 ±90°로 계속 움직이게 하니 약 110분 뒤 71°C가 되어 과열됐다. 15 kg·cm로 버티기만 할 때는 48°C에서 더 오르지 않았다 ([hardware.md](hardware.md#다른-사람이-잰-값)).
    - 뱀부랩 PLA Basic은 57°C, PETG-CF는 74°C부터 변형된다 (판매처가 옮긴 뱀부랩 자료: [PLA](https://www.crea3d.com/en/filaments/1276-bambu-pla-basic.html), [PETG-CF](https://www.crea3d.com/en/filaments/1308-bambu-petg-cf.html)).
    - 덮개가 서보 온도를 얼마나 올리는지는 확인하지 못했다.
  - 무게: 지름 50 mm, 길이 170 mm, 벽 1.2 mm인 덮개는 약 40 g이다(계산). 팔을 수평으로 폈을 때 J2에 약 1.1 kg·cm가 더 걸린다.
  - 끼워 맞추거나 붙인 뚜껑은 다시 열 수 없다 ([895746 댓글](https://makerworld.com/en/models/895746)).
  - 케이블이 밖으로 지나가면 협동로봇처럼 보이지 않는다 ([mockway](https://makerworld.com/en/models/3068526), [PRIMO](https://makerworld.com/en/models/2473047)).

## 피할 것

만든 사람들이 겪은 문제다.

- **서보 자리가 맞지 않는다.** 도면 치수나 한 판매처 부품에 맞춰 그리면 안 맞는 경우가 MakerWorld에서 가장 흔한 불평이다. 실물을 재고 시험편으로 맞춘다 ([design.md](design.md#출력-전-확인)).
- **PLA 프레임이 휜다.** SO-101은 서보를 빼고 프레임만으로 300 g에 7.5 mm 처진다. 이 팔은 링크가 더 길다.
- **서보가 뜨거워진다.** 시험값은 [hardware.md](hardware.md#다른-사람이-잰-값)에 있다. PLA는 55~60°C부터 무르다.
- **긴 링크가 층 사이로 갈라지고, 첫 층이 실패한다** (SO-101).
- **플라스틱 나사산과 핀 축이 헐거워진다** (Koch, MakerWorld 여러 모델). 너트 홈을 쓴다 ([0003](decisions/0003-fastening-nut-traps.md)).
- **케이블이 짧거나 끼이거나 감겨서 뜯긴다** (SO-ARM-102, AM-ARM, PAROL6, reBot). 끌리는 배선이 커넥터를 닳게 한 예도 있다.
- **나사에 드라이버가 닿지 않는다.** 판이 서보 나사를 가리는 경우도 있다 (SO-ARM-102).
- **그리퍼가 물체를 쥔 채 멈춰 있으면 서보가 과열되거나 부서진다** (ElRobot, LeRobot). 닫는 쪽 힘을 전류로 제한한다.
- **버스에 다른 서보를 섞으면 문제가 난다.** 전압판이 다르면 과전압 경고가 나고, 펌웨어가 다르면 통신 오류가 난다 ([lerobot#1010](https://github.com/huggingface/lerobot/issues/1010)).
- **끝까지 밀어 영점을 잡는 자동 캘리브레이션이 위험하다.** 시작 자세에 따라 팔을 책상에 눌렀다 (ElRobot). 영점은 핀으로 잡는다.
- **베이스가 흔들리거나 넘어진다** (PAROL6, myCobot). PAROL6은 베이스를 옆으로 밀어 들리거나 밀리지 않는지 본다.
- **D405 커넥터가 떨어진다.** USB 2로 떨어지거나 끊기는 문제가 있다. 나사 고정 케이블을 세게 조여 커넥터 납땜이 떨어진 예도 있다 ([librealsense#14406](https://github.com/realsenseai/librealsense/issues/14406)). 선을 고르는 조사와 다른 로봇의 사례(ALOHA 2, Stretch 3)는 [0031](decisions/0031-wiring.md#d405-선)에 있다.
- **출력한 감속기와 휘는 프레임은 반복 정밀도를 망친다** (Faze4 저자). 이 팔은 감속을 서보 안에서만 한다.
- **부품표, 나사 목록, 설명서가 없다.** 가장 흔한 불평이다. 부품표와 조립 순서를 같이 만든다 ([design.md](design.md#설계-방식)).
