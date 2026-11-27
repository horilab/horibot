//! 레포의 부딪힘 검사 형상(data/collision/)과 로봇 설정으로 이름 붙은 자세와 이동을 본다 (0041, 0037).
//! 형상이 지금 설계와 같은지는 설계 패키지의 시험(packages/design/tests/test_collision.py)이 본다.

use std::path::{Path, PathBuf};

use horibot_controller::collision::{Fingers, Model, WINDOW_MM};
use horibot_controller::runner::Script;
use horibot_controller::settings::Settings;

fn data() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../data")
}

fn load() -> (Settings, Model) {
    let settings = Settings::load(&data().join("robot.toml")).unwrap_or_else(|e| panic!("{e}"));
    let model = Model::load(&data().join("collision"), &settings, Fingers::Swept)
        .unwrap_or_else(|e| panic!("{e}"));
    (settings, model)
}

fn deg(q: [f64; 6]) -> [f64; 6] {
    q.map(f64::to_radians)
}

fn pose(settings: &Settings, name: &str) -> [f64; 6] {
    *settings.pose(name).unwrap_or_else(|| panic!("자세 {name}"))
}

#[test]
fn named_poses_are_clear() {
    let (settings, model) = load();
    // 그리퍼가 로봇 설정에 있어서 도구 부품까지 다 읽는다
    assert_eq!(model.len(), 409);
    for name in ["rest", "rest_above", "home"] {
        let q = pose(&settings, name);
        let found = model.first_within(&q, model.margin());
        assert!(
            found.is_none(),
            "{name}: {}",
            found.map(|n| model.describe(&n)).unwrap_or_default()
        );
    }
}

#[test]
fn the_shelf_is_left_by_lifting_first() {
    let (settings, model) = load();
    let rest = pose(&settings, "rest");
    let above = pose(&settings, "rest_above");
    let home = pose(&settings, "home");
    // 쉬는 자세에서 홈으로 곧게 가면 아래팔이 선반을 파고든다 (설계 형상으로도 겹친다, 2026-11-27)
    let blocked = model.check_move(&rest, &home).expect_err("곧게는 막힌다");
    let text = model.describe(&blocked.nearest);
    assert!(text.contains("선반") && text.contains("아래팔"), "{text}");
    assert!(blocked.fraction < 0.2, "{blocked:?}");
    // 쉬는 자세 위를 거치면 지난다 (앉을 때는 거꾸로)
    for (a, b) in [(rest, above), (above, home), (home, above), (above, rest)] {
        let report = model
            .check_move(&a, &b)
            .unwrap_or_else(|x| panic!("{}", model.describe(&x.nearest)));
        assert!(report.nearest.distance >= model.margin());
    }
}

#[test]
fn a_script_is_checked_from_the_arm_pose() {
    let (settings, model) = load();
    let rest = pose(&settings, "rest");
    // 영점을 잰 J1, J2만 움직이는 순서: 쉬는 자세 위로 들고 앞으로 돌린 뒤 다시 앉는다
    let text = r#"
name = "어깨"
joints = ["j1", "j2"]
start = "rest"
[[steps]]
move = "rest_above"
[[steps]]
move = { j1 = 0.0, j2 = 30.0 }
[[steps]]
move = "rest_above"
[[steps]]
move = "rest"
"#;
    let script = Script::parse(text, &settings).unwrap_or_else(|e| panic!("{e}"));
    let summary = script
        .check_collisions(&model, &rest)
        .unwrap_or_else(|e| panic!("{e}"));
    assert!(summary.contains("움직이기 4개"), "{summary}");
    // 쉬는 자세에서 위팔을 더 숙이면 아래팔이 선반을 누른다: 첫 단계에서 막힌다
    let text = r#"
name = "누르기"
joints = ["j1", "j2"]
[[steps]]
move = { j2 = 60.0 }
"#;
    let script = Script::parse(text, &settings).unwrap_or_else(|e| panic!("{e}"));
    let error = script
        .check_collisions(&model, &rest)
        .expect_err("선반을 누른다");
    assert!(
        error.contains("1번째 단계") && error.contains("선반"),
        "{error}"
    );
}

#[test]
fn neighbour_links_stay_apart_over_the_joint_ranges() {
    // 실행 중에는 이웃 링크의 짝을 보지 않는다: 관절 범위 안 어디서도 닿지 않아야 한다 (0041의 결정 3).
    // 둘 사이 거리는 그 관절 각 하나에만 달려서, 다른 관절은 범위 가운데에 둔다
    let (settings, model) = load();
    let middle: [f64; 6] = std::array::from_fn(|k| {
        let joint = &settings.joints[k];
        (joint.lower + joint.upper) / 2.0
    });
    for joint in 1..=6 {
        let limits = &settings.joints[joint - 1];
        let steps = ((limits.upper - limits.lower).to_degrees() * 2.0).ceil() as usize;
        let mut worst: Option<(f64, f32, String)> = None;
        for n in 0..=steps {
            let mut q = middle;
            q[joint - 1] = limits.lower + (limits.upper - limits.lower) * n as f64 / steps as f64;
            let near = model.neighbours(&q, joint, WINDOW_MM);
            if worst.as_ref().is_none_or(|(_, d, _)| near.distance < *d) {
                worst = Some((
                    q[joint - 1].to_degrees(),
                    near.distance,
                    model.describe(&near),
                ));
            }
        }
        let (angle, distance, text) = worst.expect("자세가 있다");
        println!("J{joint}: 가장 가까움 {angle:.1}°에서 {text}");
        assert!(distance > 0.5, "J{joint} {angle:.1}°: {text}");
    }
}

#[test]
fn link_one_stays_off_the_safety_planes() {
    // 실행 중에는 링크 1(J1 몸통, 어깨 집, 선반)과 안전 평면을 보지 않는다: J1으로만 돌아 책상까지의 높이는 그대로이고,
    // 벽까지의 거리는 J1 각 하나에만 달린다. J1 범위 어디서도 여유 밖이어야 한다
    let (settings, model) = load();
    let j1 = &settings.joints[0];
    let mut worst: Option<(f64, f32, String)> = None;
    for n in 0..=640 {
        let mut q = [0.0, 0.0, 0.0, std::f64::consts::PI, 0.0, 0.0];
        q[0] = j1.lower + (j1.upper - j1.lower) * f64::from(n) / 640.0;
        let near = model.plane_clearance(&q, 1);
        if worst.as_ref().is_none_or(|(_, d, _)| near.distance < *d) {
            worst = Some((q[0].to_degrees(), near.distance, model.describe(&near)));
        }
    }
    let (angle, distance, text) = worst.expect("자세가 있다");
    println!("링크 1: 가장 가까움 J1 {angle:.1}°에서 {text}");
    assert!(distance > model.margin(), "J1 {angle:.1}°: {text}");
}

#[test]
fn folded_poses_are_blocked() {
    let (_, model) = load();
    for (q, want) in [
        // 위팔을 숙이고 아래팔을 접어 책상에 박는다
        (deg([0.0, 100.0, 100.0, 180.0, 0.0, 0.0]), "안전 평면 책상"),
        // 위팔을 뒤로 눕혀 벽에 박는다
        (deg([0.0, -90.0, 0.0, 180.0, 0.0, 0.0]), "안전 평면 벽"),
        // 아래팔을 다 접어 손목이 위팔에 박힌다 (두 칸 넘게 떨어진 링크)
        (deg([0.0, 0.0, 160.0, 180.0, 0.0, 0.0]), "위팔/"),
    ] {
        let n = model.nearest(&q, WINDOW_MM);
        let text = model.describe(&n);
        assert!(n.distance < model.margin() && text.contains(want), "{text}");
    }
}
