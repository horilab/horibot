//! 실행 루프를 시뮬레이션 서버에 붙여 돌린다 (docs/decisions/0033의 만드는 순서 1).
//!
//! Python 시뮬레이션 서버(`uv run python -m horibot_sim --port 0`)를 띄워 쓴다. 관절 층과 실행 루프는
//! 실물과 같은 코드이고, 서보 버스만 시뮬레이션이다.

use std::io::{BufRead, BufReader};
use std::path::Path;
use std::process::{Child, Command, Stdio};
use std::time::Duration;

use horibot_controller::joints::{Caps, Joints, Servo};
use horibot_controller::plan::{MotionLimits, move_joints};
use horibot_controller::runloop::{Mode, RunLoop, Trajectory};
use horibot_controller::settings::{Calibration, Direction, Joint, LoopSettings, RAD_PER_STEP};
use horibot_servo::sim::{SimBus, SimServo};

const PERIOD: Duration = Duration::from_millis(10);

/// 띄운 시뮬레이션 서버. 시험이 끝나면 끈다.
struct Server(Child);

impl Drop for Server {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}

/// 가상환경의 Python으로 서버를 바로 띄운다. `uv run`을 거치면 시험이 끝나 uv를 꺼도 Python 서버가 남아
/// (Windows는 손자 프로세스를 같이 끄지 않는다) 출력 통로를 붙잡는다. 가상환경은 `uv sync`로 만든다.
fn start_server() -> (Server, String) {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    let python = if cfg!(windows) {
        root.join(".venv/Scripts/python.exe")
    } else {
        root.join(".venv/bin/python")
    };
    // 시뮬레이션은 설계를 불러오지 않고 설계가 쓴 MJCF만 읽는다 (0007)
    let model = std::env::temp_dir().join(format!("horibot-sim-{}.xml", std::process::id()));
    let written = Command::new(&python)
        .args(["-m", "horibot_design.mjcf", "--out"])
        .arg(&model)
        .current_dir(&root)
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status()
        .expect("설계 MJCF를 쓰는 명령을 띄우지 못했다");
    assert!(written.success(), "설계 MJCF를 쓰지 못했다");
    let mut child = Command::new(&python)
        .args(["-m", "horibot_sim", "--port", "0", "--model"])
        .arg(&model)
        .arg("--servos")
        .arg(root.join("params/servos.toml"))
        .current_dir(&root)
        .stdout(Stdio::piped())
        .stderr(Stdio::null())
        .spawn()
        .unwrap_or_else(|e| {
            panic!(
                "{}로 시뮬레이션 서버를 띄우지 못했다: {e}",
                python.display()
            )
        });
    let stdout = child.stdout.take().expect("서버의 출력");
    let mut line = String::new();
    BufReader::new(stdout)
        .read_line(&mut line)
        .expect("서버가 준비됐다는 줄");
    let port = line
        .trim()
        .strip_prefix("listening ")
        .unwrap_or_else(|| panic!("서버의 첫 줄이 다르다: {line}"));
    (Server(child), format!("127.0.0.1:{port}"))
}

/// 서보를 단 관절: 이름, ID, 모델, 모델 번호. 실물과 같은 서보다 (팔의 무게가 실물이라 J2는 ST-3120이 든다)
const SERVOS: [(&str, u8, &str, u16); 2] = [("j2", 2, "ST-3120", 3337), ("j3", 3, "STS3250", 2825)];

/// 시뮬레이션의 서보는 범위 가운데(j2, j3은 0)가 2048로 읽히게 달려 있다 (방향 +1, 오프셋 0).
fn joint(&(name, id, model, model_number): &(&'static str, u8, &'static str, u16)) -> Joint {
    Joint {
        name,
        id,
        model,
        model_number,
        lower: -1.0,
        upper: 1.5,
        pin: 0.0,
        calibration: Some(Calibration {
            pin: 0.0,
            zero_raw: 2048,
            direction: Direction::Same,
            position_offset: 0,
        }),
        torque_limit: None,
        eeprom: Vec::new(),
    }
}

fn settings(sag_gain: f64) -> LoopSettings {
    LoopSettings {
        max_speed: 1.0,
        start_tolerance: 0.01,
        temperature_warn: 50.0,
        temperature_rest: 55.0,
        voltage_warn: 11.0,
        sag_gain,
        sag_max: 0.05,
        max_acceleration: 2.0,
        max_jerk: 10.0,
        max_tool_speed: 0.05,
        max_tool_rotation: 0.5,
        following_error: 0.2,
        following_time: 0.3,
        load_limit: 0.95,
        load_time: 2.0,
        // 로봇 설정과 같은 40 %. J2가 0.6 rad로 버티는 데 14 %가 들어서, 20 %면 처짐 보정이 넘친 힘이 한도에
        // 걸려 출렁인다
        torque_cap: 400,
    }
}

/// J2를 0.6 rad 기울이고 3초 버틴 뒤, J2가 읽은 관절 각과 부하.
fn tilt_and_hold(address: &str, sag_gain: f64) -> (f64, i32) {
    let sim: Vec<SimServo> = SERVOS
        .iter()
        .map(|&(joint, id, _, model)| SimServo {
            id,
            joint: joint.to_owned(),
            model,
            direction: 1,
        })
        .collect();
    let bus = SimBus::connect(address, PERIOD, &sim).unwrap();
    let robot: Vec<Servo> = SERVOS
        .iter()
        .map(|servo| Servo::from_joint(&joint(servo)).unwrap())
        .collect();
    let joints = Joints::open(bus, robot, Caps::for_loop(&settings(0.0))).unwrap();
    let mut run = RunLoop::start(joints, settings(sag_gain), None).unwrap();
    run.submit(Trajectory {
        points: vec![(0.0, vec![0.0, 0.0]), (1.0, vec![0.6, 0.0])],
    })
    .unwrap();
    for _ in 0..400 {
        run.tick(PERIOD.as_secs_f64());
    }
    assert_eq!(run.mode(), &Mode::Holding);
    let (_, j2) = run.states()[0];
    (f64::from(j2.position - 2048) * RAD_PER_STEP, j2.load)
}

#[test]
fn run_loop_moves_the_simulated_arm_and_sag_correction_closes_the_gap() {
    let (_server, address) = start_server();
    // 위치 서보는 팔의 무게만큼 처진 채 버틴다
    let (angle, load) = tilt_and_hold(&address, 0.0);
    let sag = (0.6 - angle).abs() / RAD_PER_STEP;
    assert!((1.0..20.0).contains(&sag), "처짐 {sag}칸");
    assert!(load.abs() > 10, "부하 {load}");
    // 처짐 보정을 켜면 읽는 위치가 목표에 닿는다
    let (angle, _) = tilt_and_hold(&address, 20.0);
    let sag = (0.6 - angle).abs() / RAD_PER_STEP;
    assert!(sag <= 1.0, "보정한 뒤 처짐 {sag}칸");
}

#[test]
fn planned_joint_move_runs_on_the_simulated_arm() {
    let (_server, address) = start_server();
    let sim: Vec<SimServo> = SERVOS
        .iter()
        .map(|&(joint, id, _, model)| SimServo {
            id,
            joint: joint.to_owned(),
            model,
            direction: 1,
        })
        .collect();
    let bus = SimBus::connect(&address, PERIOD, &sim).unwrap();
    let robot: Vec<Servo> = SERVOS
        .iter()
        .map(|servo| Servo::from_joint(&joint(servo)).unwrap())
        .collect();
    let joints = Joints::open(bus, robot, Caps::for_loop(&settings(0.0))).unwrap();
    let mut run = RunLoop::start(joints, settings(0.0), None).unwrap();
    let dt = PERIOD.as_secs_f64();
    // 저크를 제한한 관절 이동을 만들어 실행 루프에 넣는다
    let goal = [0.5, -0.3];
    let limits = MotionLimits::same(2, 0.8, 2.0, 10.0);
    let trajectory = move_joints(run.targets(), &goal, &limits, dt).unwrap();
    let duration = trajectory.points.last().unwrap().0;
    run.submit(trajectory).unwrap();
    let mut ticks = 1;
    while run.tick(dt) != &Mode::Holding {
        ticks += 1;
        assert!(ticks < 2000, "궤적이 끝나지 않았다");
    }
    assert!(
        (f64::from(ticks) * dt - duration).abs() < 0.05,
        "{ticks}박자, 궤적 {duration} s"
    );
    // 자리를 잡은 뒤 시뮬레이션 팔이 목표에 있다 (팔의 무게로 몇 칸 처질 수 있다)
    for _ in 0..100 {
        run.tick(dt);
    }
    for (index, (_, state)) in run.states().iter().enumerate() {
        let angle = f64::from(state.position - 2048) * RAD_PER_STEP;
        assert!((angle - goal[index]).abs() < 0.02, "{index}: {angle}");
    }
}

/// 실행 명령 시험의 로봇 설정: 관절 하나(j3), 시뮬레이션처럼 범위 가운데(0)가 2048로 읽힌다.
const RUN_ROBOT: &str = r#"
boundary_clearance_deg = 15

[run_loop]
max_speed_deg_s = 30
start_tolerance_deg = 0.5
temperature_warn_c = 50
temperature_rest_c = 55
voltage_warn_v = 11.0
sag_gain_per_s = 0
sag_max_deg = 3
max_acceleration_deg_s2 = 60
max_jerk_deg_s3 = 300
max_tool_speed_mm_s = 30
max_tool_rotation_deg_s = 20
following_error_deg = 10
following_time_s = 0.3
load_limit_percent = 95
load_time_s = 2.0
torque_cap_percent = 40

[kinematics]
origins_mm = [[0, 0, 0], [0, 0, 160], [0, 0, 220], [0, 0, 180], [0, 79.21, 0], [0, 0, 70.31]]
axes = [[0, 0, 1], [0, 1, 0], [0, 1, 0], [0, 1, 0], [0, 0, 1], [0, 1, 0]]
flange_position_mm = [0, 32.6, 0]
flange_rotation = [[0, 1, 0], [0, 0, 1], [1, 0, 0]]

[models.STS3250.registers]
response_delay = 0
max_voltage = 160
phase = 12
unload_condition = 45
led_alarm_condition = 45
p_gain = 32
d_gain = 32
min_starting_torque = 0
speed_p_gain = 10
speed_i_gain = 200
max_temperature = 80
min_voltage = 60
dead_zone_positive = 0
dead_zone_negative = 0
protection_current = 310
overcurrent_protection_time = 250

[joints.j3]
id = 3
model = "STS3250"
soft_limits_deg = [-30, 118]
pin_deg = 0
zero_raw = 2048
direction = 1
position_offset = 0
"#;

const RUN_SCRIPT: &str = r#"
name = "실행 명령 시험"
joints = ["j3"]

[[steps]]
move = { j3 = 5.0 }
[[steps]]
hold_s = 0.3
[[steps]]
move = { j3 = 0.0 }
"#;

/// 실행 명령(`run --sim`)을 끝까지 돌린다: 실행 루프 스레드, 기록 스레드, 터미널이 같이 돌아 순서를 마치고
/// 기록(meta.json, 박자마다 깨어나는 지연이 든 samples.csv)을 남기는지 본다 (0026의 2, 5).
#[test]
fn the_run_command_finishes_a_script_on_the_simulation() {
    let (_server, address) = start_server();
    let dir = std::env::temp_dir().join(format!("horibot-run-command-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    std::fs::write(dir.join("robot.toml"), RUN_ROBOT).unwrap();
    std::fs::write(dir.join("script.toml"), RUN_SCRIPT).unwrap();
    let output = Command::new(env!("CARGO_BIN_EXE_horibot-controller"))
        .arg("run")
        .arg("--settings")
        .arg(dir.join("robot.toml"))
        .arg("--script")
        .arg(dir.join("script.toml"))
        .args(["--sim", &address, "--yes", "--records"])
        .arg(dir.join("records"))
        .stdin(Stdio::null())
        .output()
        .expect("실행 명령을 띄우지 못했다");
    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(output.status.success(), "{stdout}\n{stderr}");
    let run_dir = std::fs::read_dir(dir.join("records/run"))
        .unwrap()
        .next()
        .expect("기록 폴더")
        .unwrap()
        .path();
    let meta: serde_json::Value =
        serde_json::from_str(&std::fs::read_to_string(run_dir.join("meta.json")).unwrap()).unwrap();
    assert_eq!(meta["end"], "done", "{meta}");
    assert!(
        meta["realtime"].as_str().unwrap().starts_with("끔"),
        "시뮬레이션은 보통 스레드로 돈다: {meta}"
    );
    assert_eq!(meta["dropped_records"], 0, "{meta}");
    assert!(meta["environment"]["host"].is_string());
    let ticks = meta["loop"]["ticks"].as_u64().unwrap();
    assert!(ticks > 50, "{ticks}박자");
    let samples = std::fs::read_to_string(run_dir.join("samples.csv")).unwrap();
    let mut lines = samples.lines();
    let header = lines.next().unwrap();
    assert!(
        header.starts_with("t_s,step,mode,wake_us,tick_us,j3_goal,"),
        "{header}"
    );
    assert_eq!(lines.count() as u64, ticks, "박자마다 한 줄");
    std::fs::remove_dir_all(&dir).unwrap();
}
