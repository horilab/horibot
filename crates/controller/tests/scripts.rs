//! 순서 파일(data/scripts/)을 실물 전에 본다: 레포의 로봇 설정과 부딪힘 검사의 형상(0041)으로 순서를 읽어 쉬는 자세에서
//! 모든 이동을 거르고, 시뮬레이션 서버에서 실행 루프로 끝까지 돌린다. 처음 전원 순서(docs/decisions/0034의 처음 전원
//! 순서), 쉬는 자세로 가는 순서, 떨림·처짐 재기 순서(0042의 재는 법)다. 영점을 아직 재지 않은 관절은 시뮬레이션처럼
//! 범위 가운데가 서보 값 2048로 읽히는 영점을 준다.

mod common;

use std::path::{Path, PathBuf};

use common::start_server;
use horibot_controller::check::Console;
use horibot_controller::collision::{Fingers, Model};
use horibot_controller::joints::{Caps, Joints, Servo};
use horibot_controller::runloop::RunLoop;
use horibot_controller::runner::{Runner, Script, Status, Step};
use horibot_controller::settings::{Joint, RAD_PER_STEP, Settings};
use horibot_controller::trial::{self, Trials};
use horibot_servo::sim::{SimBus, SimServo};

const DT: f64 = 0.01;

fn data() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../data")
}

/// 영점을 재기 전인 관절의 설계로 본 방향 (data/robot.toml의 관절마다 주석)
const DESIGN_DIRECTION: [(&str, i32); 2] = [("j3", 1), ("j4", -1)];

/// 레포의 로봇 설정. 영점이 없는 관절에는 범위 가운데가 2048로 읽히는 영점을 준다 (시뮬레이션 서보가 그렇게 달려 있다).
fn settings() -> Settings {
    let mut text = std::fs::read_to_string(data().join("robot.toml")).unwrap();
    let plain = Settings::parse(&text).unwrap_or_else(|e| panic!("{e}"));
    for joint in &plain.joints {
        if joint.calibration.is_some() {
            continue;
        }
        let direction = DESIGN_DIRECTION
            .iter()
            .find(|(name, _)| *name == joint.name)
            .unwrap_or_else(|| panic!("{}의 설계 방향을 시험에 적는다", joint.name))
            .1;
        let middle = (joint.lower + joint.upper) / 2.0;
        let zero_raw = 2048.0 - f64::from(direction) * (middle - joint.pin) / RAD_PER_STEP;
        let header = format!("[joints.{}]", joint.name);
        text = text.replacen(
            &header,
            &format!(
                "{header}\nposition_offset = 0\nzero_raw = {}\ndirection = {direction}",
                zero_raw.round()
            ),
            1,
        );
    }
    Settings::parse(&text).unwrap_or_else(|e| panic!("{e}"))
}

/// 폴더의 순서: (파일 이름, 내용), 이름 순
fn scripts_in(folder: &str) -> Vec<(String, String)> {
    let mut found: Vec<(String, String)> = std::fs::read_dir(data().join("scripts").join(folder))
        .unwrap()
        .map(|entry| {
            let path = entry.unwrap().path();
            let name = path.file_name().unwrap().to_string_lossy().into_owned();
            (name, std::fs::read_to_string(&path).unwrap())
        })
        .collect();
    found.sort();
    found
}

/// 처음 전원의 순서 셋
fn first_power() -> Vec<(String, String)> {
    scripts_in("first-power")
}

/// 떨림·처짐 재기의 순서 셋: J1 ~ J3 왕복, 짐을 든 처짐, J4 ~ J6 왕복
fn tuning() -> Vec<(String, String)> {
    scripts_in("tuning")
}

fn parse(name: &str, text: &str, settings: &Settings) -> Script {
    Script::parse(text, settings).unwrap_or_else(|e| panic!("{name}: {e}"))
}

fn same(a: &[f64], b: &[f64]) -> bool {
    a.len() == b.len() && a.iter().zip(b).all(|(x, y)| (x - y).abs() < 1e-9)
}

/// 순서의 움직이기 목표를 차례로 따라간 관절 각 (순서의 관절 순서)
fn poses_of(script: &Script, start: &[f64]) -> Vec<Vec<f64>> {
    let mut q = start.to_vec();
    let mut out = Vec::new();
    for step in &script.steps {
        if let Step::Move { targets, .. } = step {
            for &(index, angle) in targets {
                q[index] = angle;
            }
            out.push(q.clone());
        }
    }
    out
}

#[test]
fn scripts_leave_and_return_by_the_rest_pose_and_pass_the_collision_check() {
    let settings = settings();
    let model = Model::load(&data().join("collision"), &settings, Fingers::Swept)
        .unwrap_or_else(|e| panic!("{e}"));
    let rest = *settings.pose("rest").unwrap();
    let above = *settings.pose("rest_above").unwrap();
    let (first, tuning) = (first_power(), tuning());
    let names = |scripts: &[(String, String)]| -> Vec<String> {
        scripts.iter().map(|(name, _)| name.clone()).collect()
    };
    assert_eq!(first.len(), 3, "{:?}", names(&first));
    assert_eq!(
        names(&tuning),
        ["arm.toml", "sag.toml", "wrist.toml"],
        "떨림·처짐 재기 순서"
    );
    for (name, text) in first.iter().chain(&tuning) {
        let script = parse(name, text, &settings);
        // 여섯 관절을 다 켠다 (켜지 않은 관절은 힘없이 늘어진다)
        assert_eq!(
            script.joints,
            ["j1", "j2", "j3", "j4", "j5", "j6"],
            "{name}"
        );
        // 쉬는 자세에서 시작해, 쉬는 자세 위를 거쳐 떠나고 앉은 뒤 토크를 끈다
        let start = script
            .start
            .as_ref()
            .unwrap_or_else(|| panic!("{name}: 시작 자세"));
        assert!(same(&start.angles, &rest), "{name}: 시작 자세는 rest");
        let poses = poses_of(&script, &start.angles);
        assert!(same(&poses[0], &above), "{name}: 첫 이동은 rest_above");
        let n = poses.len();
        assert!(
            same(&poses[n - 2], &above) && same(&poses[n - 1], &rest),
            "{name}: 끝은 rest_above → rest"
        );
        assert_eq!(
            script.steps.last(),
            Some(&Step::Release),
            "{name}: 마지막은 토크 끄기"
        );
        let summary = script
            .check_collisions(&model, &rest)
            .unwrap_or_else(|e| panic!("{name}: {e}"));
        println!("{name}: {summary}");
        // 짐을 드는 순서는 뜨거우면 홈 자세로 돌아가 짐을 빼고, 그 길도 검사했다
        if name.contains("shoulder") || name.contains("sag") {
            let target = script
                .hot_return
                .unwrap_or_else(|| panic!("{name}: hot_return"));
            assert!(
                matches!(&script.steps[target], Step::Move { .. }),
                "{name}: 돌아갈 곳은 움직이기"
            );
            assert!(summary.contains("뜨거울 때 돌아가는 길"), "{summary}");
        }
    }
}

/// 어깨 버티기를 중간에 세운 자리(짐을 뺀 뒤)와 떨림·처짐 재기를 세운 자리에서 쉬는 자세로 돌아가는 순서가 지난다:
/// 어깨 버티기는 쉬는 자세 위로 곧게(to-rest.toml), 떨림·처짐 재기는 그 길이나 홈 자세를 거치는 길
/// (to-rest-via-home.toml) 가운데 하나로 (손목을 홈 자세보다 더 돌린 자리에서는 곧은 길이 위팔을 스친다).
#[test]
fn the_way_back_to_rest_passes_from_the_holding_and_measuring_poses() {
    let settings = settings();
    let model = Model::load(&data().join("collision"), &settings, Fingers::Swept)
        .unwrap_or_else(|e| panic!("{e}"));
    let back = |name: &str| {
        let text = std::fs::read_to_string(data().join("scripts").join(name)).unwrap();
        let script = parse(name, &text, &settings);
        assert!(script.start.is_none(), "{name}: 어디서든 돈다");
        script
    };
    let (direct, via_home) = (back("to-rest.toml"), back("to-rest-via-home.toml"));
    let rest = *settings.pose("rest").unwrap();
    let above = *settings.pose("rest_above").unwrap();
    // 같은 자세는 한 번만 본다 (검사가 오래 걸린다)
    let mut seen: Vec<[f64; 6]> = Vec::new();
    let mut fresh = |q: &[f64]| {
        let at: [f64; 6] = std::array::from_fn(|k| q[k]);
        let new = !seen.iter().any(|known| same(known, &at));
        if new {
            seen.push(at);
        }
        new.then_some(at)
    };
    let (shoulder, hold) = &first_power()[0];
    assert!(shoulder.contains("shoulder"), "{shoulder}");
    for q in poses_of(&parse(shoulder, hold, &settings), &rest) {
        if let Some(at) = fresh(&q) {
            direct
                .check_collisions(&model, &at)
                .unwrap_or_else(|e| panic!("{shoulder}의 {:?}에서: {e}", at.map(f64::to_degrees)));
        }
    }
    for (name, text) in tuning() {
        for q in poses_of(&parse(&name, &text, &settings), &rest) {
            // 쉬는 자세와 쉬는 자세 위는 순서의 시작과 끝이다
            let Some(at) = fresh(&q).filter(|at| !same(at, &rest) && !same(at, &above)) else {
                continue;
            };
            if let Err(straight) = direct.check_collisions(&model, &at) {
                via_home.check_collisions(&model, &at).unwrap_or_else(|e| {
                    panic!(
                        "{name}의 {:?}에서 곧게도 ({straight}) 홈 자세를 거쳐서도 ({e}) 막힌다",
                        at.map(f64::to_degrees)
                    )
                });
            }
        }
    }
}

/// 왕복 순서(J1 ~ J3, J4 ~ J6)는 홈 자세에서 관절 하나씩, 가운데(홈 자세에서 20° 안)에서 ±20°를 7.5°/s(25%)와
/// 15°/s(50%)로 오가고, 끝마다 5초 버틴다 (0042의 재는 법). 분석(tools/run_metrics.py)은 관절 하나만 움직인
/// 움직이기로 떨림을 잰다.
#[test]
fn the_sweeps_move_one_joint_at_a_time_around_home() {
    let settings = settings();
    let home = *settings.pose("home").unwrap();
    let rest = *settings.pose("rest").unwrap();
    let swept = [("arm.toml", [0, 1, 2]), ("wrist.toml", [3, 4, 5])];
    for (name, joints) in swept {
        let text = std::fs::read_to_string(data().join("scripts/tuning").join(name)).unwrap();
        let script = parse(name, &text, &settings);
        let poses = poses_of(&script, &rest);
        assert!(same(&poses[1], &home), "{name}: 두 번째 움직이기는 홈 자세");
        let moves: Vec<usize> = (0..script.steps.len())
            .filter(|&n| matches!(script.steps[n], Step::Move { .. }))
            .collect();
        // 쉬는 자세 위와 홈 자세 뒤부터 끝의 쉬는 자세 위 앞까지는 관절 하나씩 움직인다
        let measuring = &moves[2..moves.len() - 2];
        for &index in &joints {
            // 이 관절의 움직이기: (단계 번호, 각 [°], 빠르기 [%])
            let mine: Vec<(usize, f64, i32)> = measuring
                .iter()
                .filter_map(|&n| match &script.steps[n] {
                    Step::Move { targets, speed } if targets[0].0 == index => {
                        assert_eq!(targets.len(), 1, "{name} {}번째 단계: 관절 하나만", n + 1);
                        Some((n, targets[0].1.to_degrees(), (speed * 100.0).round() as i32))
                    }
                    _ => None,
                })
                .collect();
            let ends: Vec<&(usize, f64, i32)> = mine
                .iter()
                .filter(|(n, ..)| script.steps[n + 1] == Step::Hold(5.0))
                .collect();
            let joint = format!("{name} j{}", index + 1);
            assert_eq!(ends.len(), 4, "{joint}: 끝에서 5초씩 네 번 버틴다");
            let speeds: Vec<i32> = ends.iter().map(|(_, _, percent)| *percent).collect();
            assert_eq!(speeds, [25, 25, 50, 50], "{joint}: 7.5°/s와 15°/s");
            let (high, low) = (ends[0].1, ends[1].1);
            assert!(
                (high - low - 40.0).abs() < 1e-9 && ends[2].1 == high && ends[3].1 == low,
                "{joint}: ±20°를 오간다: {ends:?}"
            );
            let middle = (high + low) / 2.0;
            let home_deg = home[index].to_degrees();
            assert!(
                (middle - home_deg).abs() <= 20.0 + 1e-9,
                "{joint}: 가운데 {middle}°는 홈 자세 {home_deg}°에서 20° 안"
            );
            // 끝이 아닌 움직이기는 가운데나 홈 자세로 간다
            for (n, angle, _) in &mine {
                let to_middle = (angle - middle).abs() < 1e-9 || (angle - home_deg).abs() < 1e-9;
                assert!(
                    to_middle || ends.iter().any(|end| end.0 == *n),
                    "{joint} {}번째 단계: {angle}°",
                    n + 1
                );
            }
        }
    }
}

/// 짐을 든 처짐은 어깨 버티기의 가장 힘든 자세(반지름 30 cm)에서 버틴다: 어깨 버티기의 자세가 설계로 바뀌면 같이
/// 바꾼다 (어깨 버티기는 tests/test_first_power_scripts.py가 설계의 IK 해와 견준다).
#[test]
fn the_sag_script_holds_the_hardest_shoulder_hold_pose() {
    let settings = settings();
    let rest = *settings.pose("rest").unwrap();
    let (name, text) = &first_power()[0];
    let shoulder = parse(name, text, &settings);
    // 어깨 버티기의 움직이기: 쉬는 자세 위, 홈, 반지름 16 ~ 30 cm 여섯, 홈, 쉬는 자세 위, 쉬는 자세
    let shoulder_poses = poses_of(&shoulder, &rest);
    assert_eq!(shoulder_poses.len(), 11, "{name}");
    let thirty = &shoulder_poses[7];
    let sag_text = std::fs::read_to_string(data().join("scripts/tuning/sag.toml")).unwrap();
    let sag = parse("sag.toml", &sag_text, &settings);
    let sag_poses = poses_of(&sag, &rest);
    assert!(
        same(&sag_poses[2], thirty),
        "sag.toml의 버티는 자세 {:?}는 어깨 버티기 30 cm {:?}와 같아야 한다",
        sag_poses[2]
            .iter()
            .map(|a| a.to_degrees())
            .collect::<Vec<_>>(),
        thirty.iter().map(|a| a.to_degrees()).collect::<Vec<_>>()
    );
    let holds: Vec<f64> = sag
        .steps
        .iter()
        .filter_map(|step| match step {
            Step::Hold(seconds) => Some(*seconds),
            _ => None,
        })
        .collect();
    assert_eq!(holds, [15.0], "버티기 하나");
}

/// 멈춤 단계에서만 Enter를 누르는 사람 (움직이거나 버티는 동안 누르면 순서가 끝난다).
#[derive(Default)]
struct Operator {
    said: Vec<String>,
    waiting: bool,
}

impl Console for Operator {
    fn say(&mut self, text: &str) {
        self.said.push(text.to_owned());
    }
    fn ask(&mut self, _question: &str) -> String {
        String::new()
    }
    fn begin_wait(&mut self, prompt: &str) {
        self.said.push(prompt.to_owned());
        self.waiting = true;
    }
    fn enter_pressed(&mut self) -> bool {
        std::mem::take(&mut self.waiting)
    }
    fn status(&mut self, _text: &str) {}
}

/// 순서를 시뮬레이션에서 실행 명령처럼 돌린다 (버티기는 2초로 줄인다). `heat`는 (서보 ID, 온도, 시뮬레이션 시간
/// [s]): 그 시간부터 그 서보가 그 온도로 읽힌다. `trials`는 실행 명령의 --trial처럼 토크를 켠 뒤 쓰고 끝나면
/// 되돌린다 (되돌린 값이 쓰기 전 값과 같은지 본다). 끝난 상태, 끝의 목표 [rad], 말한 줄.
fn run_on_simulation(
    address: &str,
    settings: &Settings,
    mut script: Script,
    heat: Option<(u8, i32, f64)>,
    trials: &Trials,
) -> (Status, Vec<f64>, Vec<String>) {
    script.cap_holds(2.0);
    let mut joints: Vec<&Joint> = script
        .joints
        .iter()
        .map(|name| settings.joint(name).unwrap())
        .collect();
    let gripper = settings.gripper.as_ref().filter(|_| script.uses_gripper());
    if let Some(gripper) = gripper {
        joints.push(&gripper.joint);
    }
    let sim: Vec<SimServo> = joints
        .iter()
        .map(|joint| SimServo {
            id: joint.id,
            joint: joint.name.to_owned(),
            model: joint.model_number,
            direction: joint.calibration.map_or(1, |c| c.direction.number()),
            position_offset: joint.calibration.map_or(0, |c| c.position_offset),
        })
        .collect();
    let start: Vec<(String, f64)> = script
        .joints
        .iter()
        .zip(&script.start.as_ref().unwrap().angles)
        .map(|(name, angle)| ((*name).to_owned(), *angle))
        .collect();
    let mut bus = SimBus::connect_at(
        address,
        std::time::Duration::from_secs_f64(DT),
        &sim,
        &start,
    )
    .unwrap();
    if let Some((id, celsius, at)) = heat {
        bus.heat(id, celsius, at).unwrap();
    }
    // 시뮬레이션은 메모리 표를 흉내 내지 않는다 (실행 명령과 같다)
    let servos: Vec<Servo> = joints
        .iter()
        .map(|joint| {
            let mut servo = Servo::from_joint(joint).unwrap();
            servo.eeprom.clear();
            servo
        })
        .collect();
    let named: Vec<(&str, u8)> = joints.iter().map(|joint| (joint.name, joint.id)).collect();
    let joints = Joints::open(bus, servos, Caps::for_loop(&settings.run_loop)).unwrap();
    let mut loop_settings = settings.run_loop;
    if let Some(gain) = trials.sag_gain {
        loop_settings.sag_gain = gain;
    }
    let mut run = RunLoop::start(joints, loop_settings, gripper).unwrap();
    let before = trial::servo_values(&mut run, &named);
    let applied = trials.apply(&mut run).unwrap();
    for servo in &trials.servos {
        let now = trial::servo_values(&mut run, &[(servo.joint, servo.id)]);
        for (register, value) in &servo.values {
            assert_eq!(
                now[servo.joint][register.name], *value,
                "{} {}",
                servo.joint, register.name
            );
        }
    }
    let mut runner = Runner::new(script, &settings.run_loop, DT);
    let mut operator = Operator::default();
    let mut tick = 0_u32;
    let status = loop {
        let now = f64::from(tick) * DT;
        run.tick(DT);
        let status = runner.tick(&mut run, &mut operator, now);
        if status != Status::Running {
            break status;
        }
        tick += 1;
        assert!(tick < 200_000, "끝나지 않는다: {:?}", operator.said);
    };
    assert!(
        status != Status::Done || runner.released(),
        "토크를 끄고 끝난다"
    );
    applied.restore(&mut run).unwrap();
    assert_eq!(
        trial::servo_values(&mut run, &named),
        before,
        "시험 값을 되돌렸다"
    );
    (status, run.targets()[..6].to_vec(), operator.said)
}

#[test]
fn scripts_run_to_the_end_on_the_simulation() {
    let (_server, address) = start_server();
    let settings = settings();
    let rest = *settings.pose("rest").unwrap();
    let none = Trials::default();
    for (name, text) in first_power().into_iter().chain(tuning()) {
        let script = parse(&name, &text, &settings);
        let pauses = script
            .steps
            .iter()
            .filter(|step| matches!(step, Step::Pause(_)))
            .count();
        // J1 ~ J3 왕복은 재는 날처럼 시험 값을 주고 돌린다
        let trials = if name == "arm.toml" {
            let args: Vec<String> = ["j2,j3:p_gain=48", "acceleration=0", "sag_gain_per_s=2"]
                .iter()
                .map(|arg| (*arg).to_owned())
                .collect();
            Trials::parse(&args, &script.joints, &settings).unwrap()
        } else {
            none.clone()
        };
        let (status, end, said) = run_on_simulation(&address, &settings, script, None, &trials);
        assert_eq!(status, Status::Done, "{name}: {said:?}");
        assert!(same(&end, &rest), "{name}: 쉬는 자세에서 끝난다");
        let waited = said.iter().filter(|line| line.contains("-pause] ")).count();
        assert_eq!(waited, pauses, "{name}: 멈춤마다 기다렸다");
        if name.contains("shoulder") || name.contains("sag") {
            assert!(
                said.iter().filter(|line| line.contains("닿았다")).count() == 4,
                "{name}: 그리퍼를 두 번 열고 닫았다: {said:?}"
            );
        }
    }
}

/// 어깨 버티기 도중 J2가 온도 알림 기준에 닿으면, 더 힘든 자세로 가지 않고 홈 자세로 돌아가 짐을 빼고 쉬는 자세에서
/// 토크를 끄고 끝난다 (0034). 시뮬레이션 시간 30초는 반지름 20 ~ 24 cm 자세 둘레다 (버티기는 2초로 줄였다).
#[test]
fn the_shoulder_hold_goes_back_home_when_a_servo_gets_warm() {
    let (_server, address) = start_server();
    let settings = settings();
    let rest = *settings.pose("rest").unwrap();
    let (name, text) = first_power()
        .into_iter()
        .find(|(name, _)| name.contains("shoulder"))
        .unwrap();
    let script = parse(&name, &text, &settings);
    let target = script.hot_return.unwrap();
    // 가장 힘든 자세(30 cm)로 가는 움직이기: 돌아갈 곳 앞의 마지막 움직이기
    let hardest = (0..target)
        .rev()
        .find(|&n| matches!(script.steps[n], Step::Move { .. }))
        .unwrap();
    let warm = settings.run_loop.temperature_warn as i32;
    let (status, end, said) = run_on_simulation(
        &address,
        &settings,
        script,
        Some((2, warm, 30.0)),
        &Trials::default(),
    );
    assert_eq!(status, Status::Done, "{said:?}");
    assert!(same(&end, &rest), "쉬는 자세에서 끝난다");
    let back = format!("{}번째 단계로 돌아간다", target + 1);
    assert!(said.iter().any(|line| line.contains(&back)), "{said:?}");
    let skipped = format!("[{}-move]", hardest + 1);
    assert!(
        !said.iter().any(|line| line.starts_with(&skipped)),
        "30 cm 자세로 가지 않는다: {said:?}"
    );
}
