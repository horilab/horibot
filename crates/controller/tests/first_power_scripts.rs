//! 처음 전원의 순서 파일(data/scripts/)을 실물 전에 본다 (docs/decisions/0034의 처음 전원 순서): 레포의 로봇 설정과
//! 부딪힘 검사의 형상(0041)으로 순서를 읽어 쉬는 자세에서 모든 이동을 거르고, 시뮬레이션 서버에서 실행 루프로 끝까지
//! 돌린다. 영점을 아직 재지 않은 관절(J3, J4는 그 관절을 단 날 잰다)은 시뮬레이션처럼 범위 가운데가 서보 값 2048로
//! 읽히는 영점을 준다.

mod common;

use std::path::{Path, PathBuf};

use common::start_server;
use horibot_controller::check::Console;
use horibot_controller::collision::{Fingers, Model};
use horibot_controller::joints::{Caps, Joints, Servo};
use horibot_controller::runloop::RunLoop;
use horibot_controller::runner::{Runner, Script, Status, Step};
use horibot_controller::settings::{Joint, RAD_PER_STEP, Settings};
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

/// 처음 전원의 순서: (파일 이름, 내용), 이름 순
fn first_power() -> Vec<(String, String)> {
    let mut found: Vec<(String, String)> = std::fs::read_dir(data().join("scripts/first-power"))
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
        if let Step::Move(targets) = step {
            for &(index, angle) in targets {
                q[index] = angle;
            }
            out.push(q.clone());
        }
    }
    out
}

#[test]
fn first_power_scripts_leave_and_return_by_the_rest_pose_and_pass_the_collision_check() {
    let settings = settings();
    let model = Model::load(&data().join("collision"), &settings, Fingers::Swept)
        .unwrap_or_else(|e| panic!("{e}"));
    let rest = *settings.pose("rest").unwrap();
    let above = *settings.pose("rest_above").unwrap();
    let scripts = first_power();
    assert_eq!(
        scripts.len(),
        4,
        "{:?}",
        scripts.iter().map(|s| &s.0).collect::<Vec<_>>()
    );
    for (name, text) in &scripts {
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
    }
}

#[test]
fn the_way_back_to_rest_passes_from_the_holding_poses() {
    // 어깨 버티기를 중간에 세운 자리(짐을 뺀 뒤)에서 쉬는 자세로 돌아가는 순서가 지난다
    let settings = settings();
    let model = Model::load(&data().join("collision"), &settings, Fingers::Swept)
        .unwrap_or_else(|e| panic!("{e}"));
    let text = std::fs::read_to_string(data().join("scripts/to-rest.toml")).unwrap();
    let back = parse("to-rest.toml", &text, &settings);
    assert!(back.start.is_none(), "어디서든 돈다");
    let (name, hold) = &first_power()[0];
    let hold = parse(name, hold, &settings);
    let rest = *settings.pose("rest").unwrap();
    for q in poses_of(&hold, &rest) {
        let at: [f64; 6] = std::array::from_fn(|k| q[k]);
        back.check_collisions(&model, &at)
            .unwrap_or_else(|e| panic!("{:?}에서: {e}", at.map(f64::to_degrees)));
    }
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

/// 순서를 시뮬레이션에서 실행 명령처럼 돌린다 (버티기는 2초로 줄인다). 끝난 상태, 끝의 목표 [rad], 말한 줄.
fn run_on_simulation(
    address: &str,
    settings: &Settings,
    mut script: Script,
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
    let bus = SimBus::connect_at(
        address,
        std::time::Duration::from_secs_f64(DT),
        &sim,
        &start,
    )
    .unwrap();
    // 시뮬레이션은 메모리 표를 흉내 내지 않는다 (실행 명령과 같다)
    let servos: Vec<Servo> = joints
        .iter()
        .map(|joint| {
            let mut servo = Servo::from_joint(joint).unwrap();
            servo.eeprom.clear();
            servo
        })
        .collect();
    let joints = Joints::open(bus, servos, Caps::for_loop(&settings.run_loop)).unwrap();
    let mut run = RunLoop::start(joints, settings.run_loop, gripper).unwrap();
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
    (status, run.targets()[..6].to_vec(), operator.said)
}

#[test]
fn first_power_scripts_run_to_the_end_on_the_simulation() {
    let (_server, address) = start_server();
    let settings = settings();
    let rest = *settings.pose("rest").unwrap();
    for (name, text) in first_power() {
        let script = parse(&name, &text, &settings);
        let pauses = script
            .steps
            .iter()
            .filter(|step| matches!(step, Step::Pause(_)))
            .count();
        let (status, end, said) = run_on_simulation(&address, &settings, script);
        assert_eq!(status, Status::Done, "{name}: {said:?}");
        assert!(same(&end, &rest), "{name}: 쉬는 자세에서 끝난다");
        let waited = said.iter().filter(|line| line.ends_with("(Enter)")).count();
        assert_eq!(waited, pauses, "{name}: 멈춤마다 기다렸다");
        if name.contains("shoulder") {
            assert!(
                said.iter().filter(|line| line.contains("닿았다")).count() == 4,
                "{name}: 그리퍼를 두 번 열고 닫았다: {said:?}"
            );
        }
    }
}
