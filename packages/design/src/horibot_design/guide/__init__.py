"""작업 안내 사이트 (0036): 조립, 출력, 재기, 배선, 설계 보기를 아티팩트 하나, 3D 뷰어 하나로 묶는다.

설계 코드(parts/)의 형상과 값 → 안내 정의(guides/, 안내 하나가 모듈 하나) → 만들기(build) → build/guide/
(페이지 하나와 data/)를 Claude가 아티팩트 "작업 안내"에 올린다. 쓰면서 생기는 진행, 확인 칸, 메모, 단계 기록은
아티팩트의 db에 쌓인다.

    uv run python -m horibot_design.guide build     # 모든 안내를 검사하고 build/guide/에 쓴다
    uv run python -m horibot_design.guide check     # 검사만 (형상은 만들지만 삼각형으로 바꾸지 않는다)
"""
