//! 로봇 컨트롤러 (docs/decisions/0023-robot-controller.md, 0024-servo-bus.md).
//!
//! 로봇 설정 확인, 포트 목록, 값 적용(0024 만드는 순서 2~3), 맨 서보 확인(만드는 순서 5), 영점 재기와 시험
//! 순서 실행(0034)이 있다. 레포 루트에서 돌린다:
//!
//!     cargo run -p horibot-controller -- settings
//!     cargo run -p horibot-controller -- apply --port COM5            (바꿀 칸만 보여 준다)
//!     cargo run -p horibot-controller -- apply --port COM5 --yes      (쓴다)
//!     cargo run -p horibot-controller -- check --port COM5 --id 20    (할 일을 보여 주고 상태를 읽는다)
//!     cargo run -p horibot-controller -- check --port COM5 --id 20 --yes
//!     cargo run -p horibot-controller -- check-state --port COM5 --id 20
//!     cargo run -p horibot-controller -- zero --port COM5 --joint j2 [--yes]
//!     cargo run -p horibot-controller -- run --script <파일> --port COM5 [--yes]
//!     cargo run -p horibot-controller -- run --script <파일> --sim 127.0.0.1:5600 --yes
//!     horibot-controller selfcheck                                    (컨트롤러 파이: 판과 실행 환경)
//!
//! 컨트롤러 파이(리눅스)에서 실물 서보로 `run`을 돌리면 실행 루프 스레드를 코어 3에 묶고 SCHED_FIFO 80으로
//! 돌리며 메모리를 잠근다. 기록은 다른 코어의 낮은 우선순위 스레드가 쓴다 (0026의 2, 5).

use std::collections::BTreeSet;
use std::fs;
use std::io::{self, BufRead, IsTerminal};
use std::path::{Path, PathBuf};
use std::process::ExitCode;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::mpsc::{self, Receiver, RecvTimeoutError, Sender, SyncSender};
use std::thread;
use std::time::{Duration, Instant};

use chrono::Local;
use horibot_controller::bus_timing;
use horibot_controller::check::{
    self, BENCH_RANGE, Check, Console, SPARE_IDS, STEPS, Terminal, WallClock,
};
use horibot_controller::collision::{Fingers, Model};
use horibot_controller::joints::{BENCH_CAPS, Caps, Joints, Servo, Snapshot};
use horibot_controller::prepare::{self, Change};
use horibot_controller::record::{self, Log};
use horibot_controller::rt;
use horibot_controller::runloop::{Mode, RunLoop};
use horibot_controller::runner::{Runner, Script, Status, Step};
use horibot_controller::settings::{Direction, GRIPPER_NAME, JOINT_NAMES, Joint, Settings};
use horibot_controller::system::{self, Environment, LoopThread};
use horibot_controller::zero;
use horibot_servo::feetech::registers::{self as reg, BAUD_RATES};
use horibot_servo::feetech::{Bus, Error, SerialLink, ServoBus};
use horibot_servo::sim::{SimBus, SimServo};
use serde_json::{Value, json};

const USAGE: &str = "\
쓰는 법: horibot-controller <명령> [옵션]

명령:
  settings            로봇 설정을 읽어 검사하고 보여 준다
  ports               이 컴퓨터의 시리얼 포트를 보여 준다
  apply               값 적용: 서보의 EEPROM을 로봇 설정과 견주어 다른 칸을 보여 주고,
                      --yes가 있으면 쓴다. 토크가 꺼져 있어야 하고, 서보는 움직이지 않는다
  check               맨 서보 확인: 로봇에 달지 않은 서보 하나의 동작을 단계마다 본다.
                      --yes가 없으면 할 일을 보여 주고 서보 상태를 한 번 읽는다 (움직이지 않는다).
                      --yes가 있으면 서보가 움직인다. --only로 단계를 고르면 그 단계만 한다
  check-state         맨 서보의 상태만 읽는다 (끊김 확인). 서보는 움직이지 않는다.
                      --situation이 없으면 상황을 묻는다
  zero                영점 재기: 관절 하나(--joint)의 영점 핀 자리를 읽어 위치 오프셋을 굽는다.
                      --yes가 없으면 지금 상태와 할 일만 보여 준다. --yes가 있으면 방향을 재고(로봇
                      설정에 방향이 없으면) 핀 자리를 읽어 굽는다. 토크가 꺼져 있어야 하고 움직이지 않는다.
                      그리퍼(--joint gripper)는 손가락끼리 맞닿은 자리를 읽고, 방향은 --direction으로 준다
  run                시험 순서(--script)를 실행 루프로 돌린다. --yes가 없으면 순서를 검사해 보여 주고
                      서보 상태만 읽는다. --yes가 있으면 서보가 움직인다. 움직이는 동안 Enter로 멈춘다.
                      움직이기 전에 지금 팔 자세에서 순서의 모든 움직이기를 부딪힘 검사로 거르고(로봇 설정
                      옆의 collision/), 막히면 시작하지 않는다.
                      리눅스에서 실물 서보로 돌리면 실행 루프를 실시간으로 돌린다 (코어 3, SCHED_FIFO 80)
  timing              서보 통신 시간 재기: 로봇 설정의 서보(그리퍼 포함)를 다 이은 채 짧은 읽기, 상태 읽기,
                      동기 읽기, 한 박자(동기 쓰기 + 동기 읽기)를 실행 루프처럼 10 ms마다 잰다.
                      --yes가 없으면 서보의 토크와 응답 지연만 읽어 보인다. 토크가 꺼져 있어야 하고
                      서보는 움직이지 않는다 (토크 한도에 읽은 값을 다시 쓰고, 응답 지연이 0이 아니면
                      잠근 채 0으로 바꿔 한 번 더 잰다). 리눅스에서는 실행 루프처럼 실시간으로 잰다
  selfcheck           판(커밋), 로봇 설정, 실행 환경(부팅 인자, 클럭 정책, 시각, 실시간 설정), 서보 어댑터
                      경로를 보여 준다. 로봇 설정을 읽지 못하면 실패한다. 서보에 닿지 않는다

옵션:
  --settings <파일>   로봇 설정 파일. 기본 data/robot.toml (레포 루트에서 돌릴 때)
  --port <포트>       서보 어댑터 포트 (예: COM5, /dev/ttyACM0)
  --joint <이름>      이 관절만 (여러 번 줄 수 있다, 그리퍼는 gripper). 기본은 로봇 설정의 모든 관절
  --id <ID>           맨 서보의 ID (check, check-state)
  --yes               보여 준 대로 서보에 쓰거나 움직인다
  --only <단계,단계>  맨 서보 확인에서 이 단계만 한다 (예: state,overload_hold,voltage)
  --situation <1~4>   끊김 확인의 상황 (check-state): 1 프로그램을 끈 뒤, 2 USB를 뺐다 꽂은 뒤,
                      3 + 플러그를 뺀 채, 4 + 플러그를 다시 꽂은 뒤
  --records <폴더>    기록 폴더. 기본 records (그 아래 servo/, run/에 쓴다)
  --direction <1|-1>  영점을 잴 때 방향을 재지 않고 이 값을 쓴다 (zero)
  --script <파일>     시험 순서 (run). 쓰는 법은 crates/controller/src/runner.rs
  --sim <주소>        서보 대신 시뮬레이션 서버에 붙는다 (run, 예: 127.0.0.1:5600)
  --no-realtime       리눅스에서도 실행 루프를 보통 스레드로 돌린다 (run, timing. 실시간 설정이 안 걸릴 때만)
  --no-collision      부딪힘 검사를 끈다 (run. 팔을 다 조립하지 않은 시험대처럼 형상이 실물과 다를 때만.
                      기록에 남는다)
  --hold-max <초>     순서의 버티기를 이 시간까지로 줄인다 (run --sim에서만. 시뮬레이션은 온도를 흉내 내지
                      않아 오래 버텨도 새로 볼 것이 없다. 기록에 남는다)
  --count <n>         동작마다 되풀이할 횟수 (timing, 기본 300)
";

/// 서보 응답 하나를 더 기다리는 여유. 서보 준비는 박자가 없어 넉넉히 둔다.
const LATENCY: Duration = Duration::from_millis(20);
/// 실행 루프가 박자마다 응답을 기다리는 여유
const LOOP_LATENCY: Duration = Duration::from_millis(5);
/// 실행 루프의 주기 (100 Hz, 0032의 후보)
const LOOP_PERIOD: Duration = Duration::from_millis(10);

#[derive(Debug, PartialEq)]
enum Command {
    Settings,
    Ports,
    Apply,
    Check,
    CheckState,
    Zero,
    Run,
    Timing,
    SelfCheck,
}

#[derive(Debug)]
struct Options {
    command: Command,
    settings: PathBuf,
    port: Option<String>,
    joints: Vec<String>,
    id: Option<u8>,
    yes: bool,
    only: Vec<String>,
    situation: Option<usize>,
    records: PathBuf,
    direction: Option<Direction>,
    script: Option<PathBuf>,
    sim: Option<String>,
    no_realtime: bool,
    no_collision: bool,
    hold_max: Option<f64>,
    count: Option<usize>,
}

fn parse_args(mut args: impl Iterator<Item = String>) -> Result<Options, String> {
    let mut command = None;
    let mut options = Options {
        command: Command::Settings,
        settings: PathBuf::from("data/robot.toml"),
        port: None,
        joints: Vec::new(),
        id: None,
        yes: false,
        only: Vec::new(),
        situation: None,
        records: PathBuf::from("records"),
        direction: None,
        script: None,
        sim: None,
        no_realtime: false,
        no_collision: false,
        hold_max: None,
        count: None,
    };
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--settings" => {
                options.settings = PathBuf::from(args.next().ok_or("--settings 뒤에 파일이 없다")?)
            }
            "--port" => options.port = Some(args.next().ok_or("--port 뒤에 포트가 없다")?),
            "--joint" => options
                .joints
                .push(args.next().ok_or("--joint 뒤에 이름이 없다")?),
            "--id" => {
                let text = args.next().ok_or("--id 뒤에 ID가 없다")?;
                options.id = Some(
                    text.parse()
                        .map_err(|_| format!("ID는 0~252의 수다: {text}"))?,
                );
            }
            "--yes" => options.yes = true,
            "--only" => {
                let text = args.next().ok_or("--only 뒤에 단계 이름이 없다")?;
                for name in text.split(',').filter(|name| !name.is_empty()) {
                    if !STEPS.iter().any(|(step, _)| *step == name) {
                        let names: Vec<&str> = STEPS.iter().map(|(step, _)| *step).collect();
                        return Err(format!(
                            "모르는 단계다: {name} (단계: {})",
                            names.join(", ")
                        ));
                    }
                    options.only.push(name.to_owned());
                }
            }
            "--situation" => {
                let text = args.next().ok_or("--situation 뒤에 번호가 없다")?;
                let number = text
                    .parse()
                    .ok()
                    .filter(|n| check::situation_name(*n).is_some());
                options.situation =
                    Some(number.ok_or_else(|| format!("상황은 1~4의 번호다: {text}"))?);
            }
            "--records" => {
                options.records = PathBuf::from(args.next().ok_or("--records 뒤에 폴더가 없다")?)
            }
            "--direction" => {
                let text = args.next().ok_or("--direction 뒤에 1이나 -1이 없다")?;
                options.direction = Some(match text.as_str() {
                    "1" => Direction::Same,
                    "-1" => Direction::Reversed,
                    _ => return Err(format!("방향은 1이나 -1이다: {text}")),
                });
            }
            "--script" => {
                options.script = Some(PathBuf::from(
                    args.next().ok_or("--script 뒤에 파일이 없다")?,
                ))
            }
            "--sim" => options.sim = Some(args.next().ok_or("--sim 뒤에 주소가 없다")?),
            "--no-realtime" => options.no_realtime = true,
            "--no-collision" => options.no_collision = true,
            "--hold-max" => {
                let text = args.next().ok_or("--hold-max 뒤에 초가 없다")?;
                options.hold_max = Some(
                    text.parse()
                        .ok()
                        .filter(|&s: &f64| s.is_finite() && s > 0.0)
                        .ok_or_else(|| format!("--hold-max는 0보다 큰 초다: {text}"))?,
                );
            }
            "--count" => {
                let text = args.next().ok_or("--count 뒤에 횟수가 없다")?;
                options.count = Some(
                    text.parse()
                        .ok()
                        .filter(|&n: &usize| n > 0)
                        .ok_or_else(|| format!("횟수는 1 이상의 수다: {text}"))?,
                );
            }
            "-h" | "--help" => return Err(String::new()),
            name if command.is_none() && !name.starts_with('-') => {
                command = Some(match name {
                    "settings" => Command::Settings,
                    "ports" => Command::Ports,
                    "apply" => Command::Apply,
                    "check" => Command::Check,
                    "check-state" => Command::CheckState,
                    "zero" => Command::Zero,
                    "run" => Command::Run,
                    "timing" => Command::Timing,
                    "selfcheck" => Command::SelfCheck,
                    other => return Err(format!("모르는 명령이다: {other}")),
                })
            }
            other => return Err(format!("모르는 옵션이다: {other}")),
        }
    }
    options.command = command.ok_or("명령이 없다")?;
    let needs_port = matches!(
        options.command,
        Command::Apply | Command::Check | Command::CheckState | Command::Zero | Command::Timing
    );
    if needs_port && options.port.is_none() {
        return Err(
            "이 명령에는 --port가 있어야 한다 (포트 목록: horibot-controller ports)".to_owned(),
        );
    }
    let needs_id = matches!(options.command, Command::Check | Command::CheckState);
    if needs_id && options.id.is_none() {
        return Err("이 명령에는 맨 서보의 --id가 있어야 한다 (예비 서보는 20)".to_owned());
    }
    if options.command == Command::Zero && options.joints.len() != 1 {
        return Err("영점은 관절 하나씩 잰다: --joint를 하나 준다".to_owned());
    }
    if options.command == Command::Run {
        if options.script.is_none() {
            return Err("run에는 --script가 있어야 한다".to_owned());
        }
        if options.port.is_some() == options.sim.is_some() {
            return Err("run은 --port(서보)나 --sim(시뮬레이션) 가운데 하나에 붙는다".to_owned());
        }
    }
    if options.hold_max.is_some() && (options.command != Command::Run || options.sim.is_none()) {
        return Err(
            "--hold-max는 run --sim에서만 쓴다 (실물에서는 순서의 버티기를 그대로 한다)".to_owned(),
        );
    }
    Ok(options)
}

fn main() -> ExitCode {
    let options = match parse_args(std::env::args().skip(1)) {
        Ok(options) => options,
        Err(error) => {
            if !error.is_empty() {
                eprintln!("{error}\n");
            }
            eprint!("{USAGE}");
            return ExitCode::from(2);
        }
    };
    let result = match options.command {
        Command::Settings => show_settings(&options),
        Command::Ports => show_ports(),
        Command::Apply => apply(&options),
        Command::Check => run_check(&options),
        Command::CheckState => check_state(&options),
        Command::Zero => measure_zero(&options),
        Command::Run => run_script(&options),
        Command::Timing => measure_bus_timing(&options),
        Command::SelfCheck => self_check(&options),
    };
    match result {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("오류: {error}");
            ExitCode::FAILURE
        }
    }
}

fn show_settings(options: &Options) -> Result<(), String> {
    let settings = Settings::load(&options.settings)?;
    println!("로봇 설정: {}", options.settings.display());
    print!("{}", settings.summary());
    Ok(())
}

fn show_ports() -> Result<(), String> {
    let names = horibot_servo::port_names().map_err(|e| format!("포트를 읽지 못했다: {e}"))?;
    if names.is_empty() {
        println!("시리얼 포트가 없다. 서보 어댑터의 USB를 꽂았는지 본다");
    }
    for name in names {
        println!("{name}");
    }
    for path in system::serial_by_id() {
        println!("{path} (고정 경로: 컨트롤러 파이에서는 이것을 --port로 준다)");
    }
    Ok(())
}

/// 자가 점검: 판, 로봇 설정, 실행 환경, 실시간 설정, 서보 어댑터 경로 (0026의 2, 4). 배포 명령이 새 판을 켠 뒤
/// 이것으로 판이 도는지 본다. 로봇 설정을 읽지 못할 때만 실패한다 (실행 환경은 판이 아니라 파이의 일이다).
fn self_check(options: &Options) -> Result<(), String> {
    println!("{}", system::version());
    let settings = Settings::load(&options.settings)?;
    println!(
        "통과 로봇 설정: {} (관절 {}개{})",
        options.settings.display(),
        settings.joints.len(),
        if settings.gripper.is_some() {
            ", 그리퍼"
        } else {
            ""
        }
    );
    let environment = Environment::read();
    for finding in environment.findings() {
        let mark = if finding.ok { "통과" } else { "확인" };
        println!("{mark} {}: {}", finding.name, finding.detail);
    }
    if !cfg!(target_os = "linux") {
        return Ok(());
    }
    match realtime_trial() {
        Ok(thread) => println!(
            "통과 실시간 설정: 메모리 잠금, {} {}, 코어 {:?}",
            thread.policy, thread.priority, thread.affinity
        ),
        Err(error) => println!("확인 실시간 설정: {error}"),
    }
    let by_id = system::serial_by_id();
    if by_id.is_empty() {
        println!("확인 서보 어댑터: /dev/serial/by-id/에 없다 (USB를 꽂았는지 본다)");
    }
    for path in by_id {
        println!("통과 서보 어댑터: {path}");
    }
    Ok(())
}

/// 서보 통신 시간 재기 ([`bus_timing`], 0024의 만드는 순서 6). --yes가 없으면 서보의 토크와 응답 지연만 읽어
/// 보인다. 리눅스에서는 실행 루프처럼 실시간 스레드(코어 3, SCHED_FIFO 80, 메모리 잠금)에서 잰다.
fn measure_bus_timing(options: &Options) -> Result<(), String> {
    let settings = Settings::load(&options.settings)?;
    let servos: Vec<&Joint> = if options.joints.is_empty() {
        let gripper = settings.gripper.as_ref().map(|g| &g.joint);
        settings.joints.iter().chain(gripper).collect()
    } else {
        selected(&settings, &options.joints)?
    };
    let port = options.port.as_deref().unwrap_or_default();
    let baud = BAUD_RATES[0].1;
    let bus = Bus::open(port, baud, LOOP_LATENCY).map_err(|e| e.to_string())?;
    time_servos(options, bus, &servos, record::run_info(port, baud), baud).map(|_| ())
}

/// 열린 버스로 통신 시간을 잰다 ([`measure_bus_timing`]). 잰 기록 폴더를 돌려준다 (--yes가 없으면 없다).
fn time_servos<B: ServoBus + Send + 'static>(
    options: &Options,
    mut bus: B,
    servos: &[&Joint],
    link: Value,
    baud: u32,
) -> Result<Option<PathBuf>, String> {
    let ids: Vec<u8> = servos.iter().map(|joint| joint.id).collect();
    let count = options.count.unwrap_or(bus_timing::DEFAULT_COUNT);
    let period_s = LOOP_PERIOD.as_secs_f64();
    let states = bus_timing::read_servos(&mut bus, &ids)?;
    for (joint, state) in servos.iter().zip(&states) {
        println!(
            "서보 {} ({}): 토크 {}, 응답 지연 7번 = {} ({} µs)",
            state.id,
            joint.name,
            if state.torque_enable == 0 {
                "꺼짐"
            } else {
                "켜짐"
            },
            state.response_delay,
            f64::from(state.response_delay) * bus_timing::RESPONSE_DELAY_STEP_S * 1e6
        );
    }
    let seconds = 2.0 * bus_timing::OPS.len() as f64 * count as f64 * period_s;
    if !options.yes {
        println!(
            "짧은 읽기, 상태 읽기, 동기 읽기, 한 박자(동기 쓰기 + 동기 읽기)를 {} ms마다 {count}번씩 잰다 (길면 {seconds:.0}초). 서보를 움직이지 않는다:",
            period_s * 1000.0
        );
        println!("- 토크가 켜진 서보가 있으면 재지 않는다");
        println!("- 동기 쓰기는 토크 한도에 읽은 값을 그대로 쓴다");
        println!(
            "- 응답 지연(7번)이 0이 아닌 서보가 있으면 잠근 채 0으로 바꿔 한 번 더 잰다 (서보 전원을 껐다 켜면 돌아간다)"
        );
        println!("재려면 --yes를 붙인다");
        return Ok(None);
    }
    if let Some(on) = states.iter().find(|s| s.torque_enable != 0) {
        return Err(format!("서보 {}의 토크가 켜져 있어서 재지 않는다", on.id));
    }
    let off = realtime_off_reason(options);
    let environment = Environment::read();
    if off.is_none() {
        for finding in environment.findings().iter().filter(|f| !f.ok) {
            println!("알림: {}: {}", finding.name, finding.detail);
        }
        rt::lock_memory().map_err(|e| {
            format!(
                "메모리를 잠그지 못했다: {e}. {}. 실시간 없이 재려면 --no-realtime을 붙인다",
                system::LIMITS_HINT
            )
        })?;
    }
    println!(
        "통신 시간을 잰다: 서보 {}개, 동작마다 {count}번 (길면 {seconds:.0}초)",
        ids.len()
    );
    let started = Local::now();
    let realtime = off.is_none();
    let measured = thread::Builder::new()
        .name("timing".to_owned())
        .stack_size(rt::PREFAULT_STACK + 1024 * 1024)
        .spawn(move || {
            let thread = if realtime {
                Some(system::make_loop_thread()?)
            } else {
                None
            };
            let mut clock = bus_timing::RtClock::default();
            let run = bus_timing::measure(&mut bus, &ids, count, period_s, baud, &mut clock)?;
            Ok::<_, String>((thread, run, bus))
        })
        .map_err(|e| format!("재는 스레드를 띄우지 못했다: {e}"))?
        .join()
        .map_err(|_| "재는 스레드가 죽었다".to_owned())?;
    let (thread, run, bus) = measured?;
    let folder = record::new_record_dir(&options.records, "servo", "timing", &started)?;
    let samples = folder.join("samples.csv");
    fs::write(&samples, run.samples_csv())
        .map_err(|e| format!("{}를 쓰지 못했다: {e}", samples.display()))?;
    let stats: serde_json::Map<String, Value> = bus
        .stats()
        .iter()
        .map(|(id, s)| {
            let stat = json!({
                "ok": s.ok,
                "no_reply": s.no_reply,
                "bad_reply": s.bad_reply,
                "consecutive_failures": s.consecutive_failures,
            });
            (id.to_string(), stat)
        })
        .collect();
    let mut meta = json!({
        "kind": "servo-timing",
        "started": record::time_text(&started),
        "ended": record::time_text(&Local::now()),
        "run": link,
        "program": system::version(),
        "environment": environment.to_json(),
        "realtime": match off {
            None => "재는 스레드: 코어 3, SCHED_FIFO 80, 메모리 잠금".to_owned(),
            Some(why) => format!("끔: {why}"),
        },
        "loop_thread": thread.as_ref().map_or(Value::Null, LoopThread::to_json),
        "link_stats": stats,
    });
    if let (Some(meta), Value::Object(result)) = (meta.as_object_mut(), run.to_json()) {
        meta.extend(result);
    }
    record::write_json(&folder.join("meta.json"), &meta)?;
    print!("{}", run.report());
    println!("기록: {}", folder.display());
    Ok(Some(folder))
}

/// 실행 루프와 같은 실시간 설정을 시험 스레드에 걸어 본다 (메모리 잠금, 코어 3, SCHED_FIFO 80).
fn realtime_trial() -> Result<LoopThread, String> {
    rt::lock_memory()
        .map_err(|e| format!("메모리를 잠그지 못했다: {e}. {}", system::LIMITS_HINT))?;
    thread::Builder::new()
        .name("realtime-trial".to_owned())
        .stack_size(rt::PREFAULT_STACK + 1024 * 1024)
        .spawn(system::make_loop_thread)
        .map_err(|e| format!("시험 스레드를 띄우지 못했다: {e}"))?
        .join()
        .map_err(|_| "시험 스레드가 죽었다".to_owned())?
}

/// 고른 관절. 고르지 않았으면 로봇 설정의 모든 관절.
fn selected<'a>(settings: &'a Settings, names: &[String]) -> Result<Vec<&'a Joint>, String> {
    if names.is_empty() {
        return Ok(settings.joints.iter().collect());
    }
    names
        .iter()
        .map(|name| {
            // 그리퍼 서보도 관절처럼 이름으로 고른다 (영점, 값 적용)
            settings
                .servo(name)
                .ok_or_else(|| format!("로봇 설정에 없는 관절이다: {name}"))
        })
        .collect()
}

fn changes_json(changes: &[Change]) -> Value {
    let changes: serde_json::Map<String, Value> = changes
        .iter()
        .map(|change| {
            (
                change.register.name.to_owned(),
                json!([change.current, change.target]),
            )
        })
        .collect();
    Value::Object(changes)
}

/// 값 적용 (0024): 서보마다 다른 칸을 보여 주고, --yes면 쓴 뒤 다시 읽어 확인한다.
/// 쓰기 전 덤프를 먼저 기록에 남기고 쓴다. 중간에 실패해도 되돌릴 자리가 남는다.
fn apply(options: &Options) -> Result<(), String> {
    let settings = Settings::load(&options.settings)?;
    let joints = selected(&settings, &options.joints)?;
    let port = options.port.as_deref().unwrap_or_default();
    let baud = BAUD_RATES[0].1;
    let mut bus = Bus::open(port, baud, LATENCY).map_err(|e| e.to_string())?;

    let mut found = Vec::new();
    for joint in joints {
        let before = prepare::inspect(&mut bus, joint)?;
        let changes = prepare::plan(&joint.eeprom, &before)?;
        if changes.is_empty() {
            println!(
                "{} (ID {}, {}): 로봇 설정과 같다",
                joint.name, joint.id, joint.model
            );
        } else {
            println!(
                "{} (ID {}, {}): 바꿀 칸 {}개",
                joint.name,
                joint.id,
                joint.model,
                changes.len()
            );
            for change in &changes {
                println!("  {}", change.describe());
            }
        }
        found.push((joint, before, changes));
    }
    let total: usize = found.iter().map(|(_, _, changes)| changes.len()).sum();
    if total == 0 {
        println!("쓸 것이 없다");
        return Ok(());
    }
    if !options.yes {
        println!("쓰려면 토크를 끈 채로 --yes를 붙여 다시 돌린다 (서보는 움직이지 않는다)");
        return Ok(());
    }

    let started = Local::now();
    let folder = record::new_record_dir(&options.records, "servo", "apply", &started)?;
    let meta_path = folder.join("meta.json");
    let robot_toml = fs::read_to_string(&options.settings)
        .map_err(|e| format!("{}을 읽지 못했다: {e}", options.settings.display()))?;
    let mut meta = json!({
        "kind": "servo-apply",
        "started": record::time_text(&started),
        "run": record::run_info(port, baud),
        "robot_toml": robot_toml,
        "joints": {},
    });
    for (joint, before, changes) in &found {
        meta["joints"][joint.name] = json!({
            "id": joint.id,
            "model": joint.model,
            "before": record::dump_json(before),
            "changes": changes_json(changes),
        });
    }
    record::write_json(&meta_path, &meta)?;

    let mut result = Ok(());
    for (joint, _, changes) in &found {
        if changes.is_empty() {
            continue;
        }
        match prepare::write(&mut bus, joint, changes) {
            Ok(after) => {
                meta["joints"][joint.name]["after"] = record::dump_json(&after);
                println!(
                    "{}: {}칸을 쓰고 다시 읽어 확인했다",
                    joint.name,
                    changes.len()
                );
            }
            Err(error) => {
                meta["joints"][joint.name]["error"] = json!(error);
                result = Err(error);
                break;
            }
        }
    }
    meta["ended"] = json!(record::time_text(&Local::now()));
    meta["end"] = json!(if result.is_ok() { "normal" } else { "error" });
    record::write_json(&meta_path, &meta)?;
    println!("기록: {}", folder.display());
    result
}

/// 맨 서보 하나를 연다: 응답하는 모델을 읽어 맨 서보로 관절 층을 만든다. 서보가 응답하지 않으면 없다.
/// 시리얼 어댑터에 물린 서보의 관절 층.
type SerialJoints = Joints<Bus<SerialLink>>;

fn open_bench(options: &Options, id: u8) -> Result<Option<(SerialJoints, u16)>, String> {
    let port = options.port.as_deref().unwrap_or_default();
    let mut bus = Bus::open(port, BAUD_RATES[0].1, LATENCY).map_err(|e| e.to_string())?;
    let model = match bus.ping(id) {
        Ok(model) => model,
        Err(Error::NoReply { .. }) => return Ok(None),
        Err(error) => return Err(error.to_string()),
    };
    let servo = Servo::bench(id, model, BENCH_RANGE.0, BENCH_RANGE.1);
    Ok(Some((Joints::open(bus, vec![servo], BENCH_CAPS)?, model)))
}

/// 맨 서보가 응답하지 않을 때 볼 것. 같은 서보를 Python 첫 연결 명령이 찾으면 선과 전원은 맞다.
fn no_reply(id: u8) -> String {
    format!(
        "서보 {id}가 응답하지 않는다. + 플러그를 꽂았는지, 서보 선과 USB, 포트 이름(ports)을 본다. 다 맞으면 \
         `uv run python -m horibot_servo scan`으로 같은 서보가 찾아지는지 본다 (찾아지면 선과 전원은 맞고 \
         컨트롤러 쪽 문제다)"
    )
}

fn servo_json(id: u8, model: u16) -> Value {
    json!({ "id": id, "model": model, "model_name": prepare::model_name(model) })
}

/// 맨 서보 확인 ([`check`]). --yes가 없으면 할 일을 보여 주고 서보 상태를 한 번 읽는다 (움직이지 않는다).
fn run_check(options: &Options) -> Result<(), String> {
    let id = options.id.ok_or("--id가 없다")?;
    if !SPARE_IDS.contains(&id) {
        return Err(format!(
            "맨 서보 확인은 예비 서보 ID({}~{})로만 한다. 관절에 단 서보를 크게 돌리지 않으려는 것이다",
            SPARE_IDS.start(),
            SPARE_IDS.end()
        ));
    }
    println!(
        "맨 서보 확인: 서보 {id} 하나만 어댑터에 꽂아 책상에 두고, 혼에는 아무것도 달지 않는다."
    );
    for (index, (name, text)) in STEPS.iter().enumerate() {
        let chosen = options.only.is_empty() || options.only.iter().any(|only| only == name);
        let mark = if chosen {
            ""
        } else {
            " (이번에는 하지 않는다)"
        };
        println!("  {}. {text}{mark}", index + 1);
    }
    println!(
        "토크 한도 {} (0.1 %), 속도 {}, 가속 {} (100 step/s²)로 움직인다. 서보 설정은 잠근 채 잠깐 바꾸고 \
         단계마다 되돌린다. 중간에 끊으면 서보 전원을 껐다 켠다 (잠근 채 바꾼 값이 사라진다).",
        BENCH_CAPS.torque_limit, BENCH_CAPS.speed, BENCH_CAPS.acceleration
    );
    if !options.yes {
        // 움직이기 전에 서보를 한 번 읽어 본다. 컨트롤러가 이 서보와 처음 말하는 자리다
        let (mut joints, model) = open_bench(options, id)?.ok_or_else(|| no_reply(id))?;
        let state = joints.snapshot(id)?;
        println!(
            "서보 {id} ({})가 응답한다: {}",
            prepare::model_name(model),
            check::describe(&state)
        );
        println!("--yes를 붙이면 서보가 움직인다. 준비되면 --yes를 붙여 다시 돌린다");
        return Ok(());
    }
    let (joints, model) = open_bench(options, id)?.ok_or_else(|| no_reply(id))?;
    let port = options.port.as_deref().unwrap_or_default();
    let started = Local::now();
    let folder = record::new_record_dir(&options.records, "servo", "check", &started)?;
    let meta_path = folder.join("meta.json");
    let mut meta = json!({
        "kind": "servo-check",
        "started": record::time_text(&started),
        "run": record::run_info(port, BAUD_RATES[0].1),
        "servo": servo_json(id, model),
        "caps": {
            "torque_limit": BENCH_CAPS.torque_limit,
            "speed": BENCH_CAPS.speed,
            "acceleration": BENCH_CAPS.acceleration,
        },
        "range": [BENCH_RANGE.0, BENCH_RANGE.1],
        "only": options.only,
        "results": {},
    });
    record::write_json(&meta_path, &meta)?;
    let log = Log::create(&folder, &Snapshot::columns())?;
    let mut run = Check::new(joints, id, Terminal::default(), WallClock::default(), log);
    run.only = options.only.clone();
    let result = run.run(&mut |results| {
        meta["results"] = Value::Object(results.clone());
        let _ = record::write_json(&meta_path, &meta);
    });
    meta["results"] = Value::Object(run.results.clone());
    meta["ended"] = json!(record::time_text(&Local::now()));
    meta["end"] = json!(if result.is_ok() { "normal" } else { "stopped" });
    record::write_json(&meta_path, &meta)?;
    println!("기록: {}", folder.display());
    result
}

/// 맨 서보의 상태만 읽는다 (끊김 확인). 상황은 --situation으로 받거나 묻고, 앞선 맨 서보 확인과 견준다.
fn check_state(options: &Options) -> Result<(), String> {
    let id = options.id.ok_or("--id가 없다")?;
    let mut console = Terminal::default();
    let situation = match options.situation {
        Some(number) => check::situation_name(number),
        None => check::ask_situation(&mut console),
    };
    let port = options.port.as_deref().unwrap_or_default();
    let started = Local::now();
    let mut meta = json!({
        "kind": "servo-check-state",
        "started": record::time_text(&started),
        "run": record::run_info(port, BAUD_RATES[0].1),
        "situation": situation,
    });
    match open_bench(options, id)? {
        Some((mut joints, model)) => {
            if situation == Some("power_out") {
                println!("+ 플러그를 뺐는데 서보가 응답한다: USB 전기로 서보가 살아 있다");
            }
            let previous = last_check_results(&options.records, id);
            let report =
                check::state_report(&mut joints, id, &mut console, previous.as_ref(), situation)?;
            meta["servo"] = servo_json(id, model);
            meta["responds"] = json!(true);
            meta["report"] = report;
        }
        None => {
            println!("서보 {id}가 응답하지 않는다");
            if situation == Some("power_out") {
                println!("+ 플러그를 뺐으니 맞다: USB 전기만으로는 서보가 살아 있지 않다");
            }
            meta["servo"] = json!({ "id": id });
            meta["responds"] = json!(false);
        }
    }
    let folder = record::new_record_dir(&options.records, "servo", "check-state", &started)?;
    record::write_json(&folder.join("meta.json"), &meta)?;
    println!("기록: {}", folder.display());
    Ok(())
}

/// 이 서보의 가장 최근 맨 서보 확인 기록에서, 잠금 단계까지 마친 것의 결과를 찾는다.
fn last_check_results(records: &Path, id: u8) -> Option<Value> {
    let mut folders: Vec<PathBuf> = fs::read_dir(records.join("servo"))
        .ok()?
        .filter_map(|entry| entry.ok().map(|entry| entry.path()))
        .collect();
    folders.sort(); // 폴더 이름이 시각으로 시작한다
    folders.iter().rev().find_map(|folder| {
        let text = fs::read_to_string(folder.join("meta.json")).ok()?;
        let meta: Value = serde_json::from_str(&text).ok()?;
        let same = meta["kind"] == "servo-check" && meta["servo"]["id"] == id;
        let results = &meta["results"];
        (same && results["lock"].get("p_before").is_some()).then(|| results.clone())
    })
}

/// 영점 재기 ([`zero`], 0024의 영점, 0034). --yes가 없으면 지금 상태와 할 일만 보여 준다.
fn measure_zero(options: &Options) -> Result<(), String> {
    let settings = Settings::load(&options.settings)?;
    let joint = selected(&settings, &options.joints)?[0];
    let port = options.port.as_deref().unwrap_or_default();
    let baud = BAUD_RATES[0].1;
    let mut bus = Bus::open(port, baud, LATENCY).map_err(|e| e.to_string())?;
    let before = prepare::inspect(&mut bus, joint)?;
    let (torque_off, offset_now) = zero::state_in(&before)?;
    let reading = zero::read_position(&mut bus, joint.id, zero::READS)?;
    let known = options
        .direction
        .or(joint.calibration.map(|calibration| calibration.direction));
    println!(
        "{} (ID {}, {}): 지금 서보 값 {reading}, 위치 오프셋 {offset_now}, 토크 {}",
        joint.name,
        joint.id,
        joint.model,
        if torque_off { "꺼짐" } else { "켜짐" }
    );
    // 그리퍼는 핀 대신 손가락끼리 맞닿은 자리(열림 폭 0)가 영점이다 (0030)
    let gripper = joint.name == GRIPPER_NAME;
    if gripper {
        println!("그리퍼의 영점은 손가락끼리 맞닿은 자리다 (열림 폭 0).");
    } else {
        println!(
            "영점 핀 자리의 관절 각은 {:.1}°다 (설계 값). 핀 구멍의 틈은 + 쪽(관절 각이 느는 쪽)으로 붙인다.",
            joint.pin.to_degrees()
        );
    }
    match known {
        Some(direction) => println!("방향 {:+} (재지 않는다)", direction.number()),
        // 그리퍼는 서보 기어가 뻑뻑해 손으로 벌리기 어렵다. 서보로 조금 돌려 본 사진으로 정한다
        None if gripper => {
            return Err(
                "그리퍼는 방향을 --direction으로 준다 (벌리는 쪽으로 서보 값이 늘면 1)".to_owned(),
            );
        }
        None => println!("방향은 핀을 꽂기 전에 관절을 + 쪽으로 돌려 잰다"),
    }
    if !torque_off {
        return Err(
            "토크가 켜져 있다. 팔을 받친 뒤 서보 전원을 껐다 켜면 토크가 꺼진다 (영점은 토크를 끈 채 잰다)"
                .to_owned(),
        );
    }
    if !options.yes {
        println!("--yes를 붙이면 순서대로 안내하고 위치 오프셋을 굽는다 (서보는 움직이지 않는다)");
        return Ok(());
    }
    if !std::io::stdin().is_terminal() {
        return Err(
            "영점은 사람이 핀을 꽂으며 Enter를 누르는 일이라 터미널 창에서 돌린다".to_owned(),
        );
    }

    let mut console = Terminal::default();
    let direction = match known {
        Some(direction) => direction,
        None => {
            console.ask("영점 핀을 빼고, 관절을 손으로 돌릴 수 있게 되면 Enter: ");
            let start = zero::read_position(&mut bus, joint.id, zero::READS)?;
            console.ask("관절을 + 쪽(관절 각이 느는 쪽)으로 20°쯤 돌린 채 Enter: ");
            let end = zero::read_position(&mut bus, joint.id, zero::READS)?;
            let direction = zero::direction_from(start, end)?;
            println!("서보 값 {start} → {end}: 방향 {:+}", direction.number());
            direction
        }
    };
    console.ask(if gripper {
        "손가락끼리 맞닿게 닫힌 채 Enter: "
    } else {
        "영점 핀을 꽂고, 핀 구멍의 틈을 + 쪽으로 밀어 붙인 채 Enter: "
    });
    let reading = zero::read_position(&mut bus, joint.id, zero::READS)?;
    let plan = zero::plan(
        joint,
        settings.boundary_clearance,
        direction,
        reading,
        offset_now,
    )?;
    println!(
        "핀 자리 {reading} (오프셋 {offset_now}): 오프셋 {}을 구우면 {}로 읽힌다",
        plan.offset_new, plan.target
    );

    let started = Local::now();
    let folder = record::new_record_dir(&options.records, "servo", "zero", &started)?;
    let meta_path = folder.join("meta.json");
    let mut meta = json!({
        "kind": "servo-zero",
        "started": record::time_text(&started),
        "run": record::run_info(port, baud),
        "joint": {
            "name": joint.name,
            "id": joint.id,
            "model": joint.model,
            "pin_deg": joint.pin.to_degrees(),
        },
        "direction": direction.number(),
        "direction_measured": known.is_none(),
        "plan": {
            "reading": plan.reading,
            "offset_now": plan.offset_now,
            "encoder": plan.encoder,
            "target": plan.target,
            "offset_new": plan.offset_new,
        },
        "before": record::dump_json(&before),
    });
    record::write_json(&meta_path, &meta)?;
    let burned = zero::burn_offset(&mut bus, joint, plan.offset_new).and_then(|after| {
        meta["after"] = record::dump_json(&after);
        bus.pause(Duration::from_millis(20));
        let zero_raw = zero::read_position(&mut bus, joint.id, zero::READS)?;
        meta["zero_raw"] = json!(zero_raw);
        zero::check_zero(&plan, zero_raw).map(|()| zero_raw)
    });
    meta["ended"] = json!(record::time_text(&Local::now()));
    match &burned {
        Ok(zero_raw) => {
            let lines = zero::settings_lines(*zero_raw, direction, plan.offset_new);
            meta["settings_lines"] = json!(lines);
            meta["end"] = json!("normal");
            let section = if gripper {
                "[gripper]".to_owned()
            } else {
                format!("[joints.{}]", joint.name)
            };
            println!(
                "로봇 설정(data/robot.toml)의 {section}에 적는다:\n{lines}그다음 apply로 각도 한계를 굽는다 (영점에서 계산한다)"
            );
        }
        Err(error) => {
            meta["error"] = json!(error);
            meta["end"] = json!("error");
        }
    }
    record::write_json(&meta_path, &meta)?;
    println!("기록: {}", folder.display());
    burned.map(|_| ())
}

/// 시험 순서를 실행 루프로 돌린다 ([`horibot_controller::runner`], 0034).
fn run_script(options: &Options) -> Result<(), String> {
    let settings = Settings::load(&options.settings)?;
    let path = options.script.as_ref().ok_or("--script가 없다")?;
    let text =
        fs::read_to_string(path).map_err(|e| format!("{}을 읽지 못했다: {e}", path.display()))?;
    let mut script = Script::parse(&text, &settings)?;
    if let Some(max) = options.hold_max {
        let shortened = script.cap_holds(max);
        println!("버티기 {shortened}개를 {max} s까지로 줄였다 (--hold-max, 시뮬레이션)");
    }
    print!("{}", script.describe());
    let mut joints: Vec<&Joint> = script
        .joints
        .iter()
        .map(|name| {
            settings
                .joint(name)
                .ok_or_else(|| format!("로봇 설정에 없는 관절이다: {name}"))
        })
        .collect::<Result<_, _>>()?;
    // 그리퍼 단계가 있으면 그리퍼 서보도 켠다 (순서를 읽을 때 그리퍼와 영점이 있는지 봤다)
    if script.uses_gripper()
        && let Some(gripper) = &settings.gripper
    {
        joints.push(&gripper.joint);
    }
    let servos: Vec<Servo> = joints
        .iter()
        .map(|joint| Servo::from_joint(joint))
        .collect::<Result<_, _>>()?;
    let caps = Caps::for_loop(&settings.run_loop);
    let robot_toml = fs::read_to_string(&options.settings)
        .map_err(|e| format!("{}을 읽지 못했다: {e}", options.settings.display()))?;
    // 부딪힘 검사의 형상은 서보에 붙기 전에 읽는다 (없거나 틀리면 붙지 않고 멈춘다)
    let model = if options.no_collision {
        None
    } else {
        let dir = options
            .settings
            .parent()
            .map_or_else(|| PathBuf::from("collision"), |p| p.join("collision"));
        Some(Model::load(&dir, &settings, Fingers::Swept)?)
    };
    if let Some(address) = &options.sim {
        let sim: Vec<SimServo> = joints
            .iter()
            .map(|joint| SimServo {
                id: joint.id,
                joint: joint.name.to_owned(),
                model: joint.model_number,
                // 영점이 없는 관절은 위 Servo::from_joint에서 이미 걸렀다
                direction: joint.calibration.map_or(1, |c| c.direction.number()),
                position_offset: joint.calibration.map_or(0, |c| c.position_offset),
            })
            .collect();
        // 시뮬레이션은 순서의 시작 자세에서 시작한다 (없으면 범위 가운데)
        let start: Vec<(String, f64)> = script
            .start
            .iter()
            .flat_map(|start| {
                script
                    .joints
                    .iter()
                    .zip(&start.angles)
                    .map(|(name, angle)| ((*name).to_owned(), *angle))
            })
            .collect();
        let mut bus =
            SimBus::connect_at(address, LOOP_PERIOD, &sim, &start).map_err(|e| e.to_string())?;
        let collision = check_script(&settings, &script, model.as_ref(), &mut bus, true)?;
        let source = Source {
            settings: &settings,
            script_text: &text,
            robot_toml: &robot_toml,
            collision,
            hold_max_s: options.hold_max,
        };
        let link = json!({ "sim": address });
        // 시뮬레이션은 메모리 표(EEPROM)를 흉내 내지 않아서, 토크를 켤 때 값 적용의 목표와 견주지 않는다
        let servos = servos
            .into_iter()
            .map(|mut servo| {
                servo.eeprom.clear();
                servo
            })
            .collect();
        return start_script(
            options,
            &source,
            script,
            Joints::open(bus, servos, caps)?,
            link,
        );
    }
    let port = options.port.as_deref().unwrap_or_default();
    let mut bus = Bus::open(port, BAUD_RATES[0].1, LOOP_LATENCY).map_err(|e| e.to_string())?;
    let collision = check_script(&settings, &script, model.as_ref(), &mut bus, false)?;
    let source = Source {
        settings: &settings,
        script_text: &text,
        robot_toml: &robot_toml,
        collision,
        hold_max_s: None,
    };
    let link = record::run_info(port, BAUD_RATES[0].1);
    start_script(
        options,
        &source,
        script,
        Joints::open(bus, servos, caps)?,
        link,
    )
}

/// 움직이기 전의 부딪힘 검사 (0041): 지금 팔 자세를 읽어 순서의 모든 움직이기를 거른다. 막히면 오류라 서보에
/// 손대지 않고 끝난다. 기록에 남길 결과를 낸다.
fn check_script<B: ServoBus>(
    settings: &Settings,
    script: &Script,
    model: Option<&Model>,
    bus: &mut B,
    sim: bool,
) -> Result<Value, String> {
    let Some(model) = model else {
        println!("부딪힘 검사: 끔 (--no-collision)");
        return Ok(json!({ "checked": false, "why": "--no-collision" }));
    };
    let start = arm_angles(bus, settings, script, sim)?;
    let summary = script.check_collisions(model, &start)?;
    println!("{summary}");
    Ok(json!({
        "checked": true,
        "margin_mm": model.margin(),
        "start_deg": start.map(f64::to_degrees),
        "summary": summary,
    }))
}

/// 지금 팔의 관절 각 [rad] (J1 ~ J6): 서보에서 읽어 영점으로 각을 낸다. 시뮬레이션은 순서에 없는 관절을 범위
/// 가운데로 본다 (시뮬레이션 서버가 그 자리에 거의 잠가 둔다, 0033). 영점이 없거나 읽지 못하는 관절이 있으면 팔
/// 자세를 몰라 검사할 수 없으므로 오류다.
fn arm_angles<B: ServoBus>(
    bus: &mut B,
    settings: &Settings,
    script: &Script,
    sim: bool,
) -> Result<[f64; 6], String> {
    let unknown = "부딪힘 검사에 팔 자세가 있어야 해서 움직이지 않는다 (팔을 다 조립하지 않은 시험이면 --no-collision)";
    let mut q = [0.0; 6];
    for (k, name) in JOINT_NAMES.iter().enumerate() {
        let joint = settings
            .joint(name)
            .ok_or_else(|| format!("로봇 설정에 {name}이 없다. {unknown}"))?;
        if sim && !script.joints.contains(name) {
            q[k] = (joint.lower + joint.upper) / 2.0;
            continue;
        }
        let calibration = joint
            .calibration
            .ok_or_else(|| format!("{name}의 영점을 재지 않았다. {unknown}"))?;
        let raw = bus
            .read(joint.id, &[reg::PRESENT_POSITION])
            .map_err(|e| format!("{name}의 위치를 읽지 못했다: {e}. {unknown}"))?
            .get(reg::PRESENT_POSITION)
            .ok_or_else(|| format!("{name}의 위치가 답에 없다. {unknown}"))?;
        q[k] = calibration.angle_at(f64::from(raw));
    }
    Ok(q)
}

/// 시험 순서를 돌릴 때 기록에 함께 남길 것.
struct Source<'a> {
    settings: &'a Settings,
    script_text: &'a str,
    robot_toml: &'a str,
    /// 움직이기 전의 부딪힘 검사 결과
    collision: Value,
    /// 줄인 버티기의 상한 [s] (--hold-max, 시뮬레이션에서만)
    hold_max_s: Option<f64>,
}

/// --yes가 없으면 서보 상태만 읽어 보이고, 있으면 실행 루프를 세워 순서를 돌린다.
fn start_script<B: ServoBus + Send + 'static>(
    options: &Options,
    source: &Source,
    script: Script,
    mut joints: Joints<B>,
    link: Value,
) -> Result<(), String> {
    if !options.yes {
        for id in joints.ids() {
            let state = joints.snapshot(id)?;
            println!("서보 {id}: {}", check::describe(&state));
        }
        println!(
            "--yes를 붙이면 토크를 켜고 순서대로 움직인다. 움직이거나 버티는 동안 Enter를 누르면 감속해 선다"
        );
        return Ok(());
    }
    let pauses = script
        .steps
        .iter()
        .any(|step| matches!(step, Step::Pause(_)));
    if pauses && !io::stdin().is_terminal() {
        return Err("멈춤 단계가 있는 순서는 Enter를 눌러야 해서 터미널 창에서 돌린다".to_owned());
    }
    drive_script(options, source, script, joints, link)
}

/// 실행 명령의 터미널: 같은 줄을 고쳐 쓰는 상태 줄 뒤에 알림을 낼 때 먼저 줄을 바꾼다.
struct RunConsole {
    terminal: Terminal,
    status_shown: bool,
}

impl RunConsole {
    fn end_status(&mut self) {
        if self.status_shown {
            println!();
            self.status_shown = false;
        }
    }
}

impl Console for RunConsole {
    fn say(&mut self, text: &str) {
        self.end_status();
        self.terminal.say(text);
    }

    fn ask(&mut self, question: &str) -> String {
        self.end_status();
        self.terminal.ask(question)
    }

    fn begin_wait(&mut self, prompt: &str) {
        self.end_status();
        self.terminal.begin_wait(prompt);
    }

    fn enter_pressed(&mut self) -> bool {
        self.terminal.enter_pressed()
    }

    fn status(&mut self, text: &str) {
        self.terminal.status(text);
        self.status_shown = true;
    }
}

/// 기록 칸: 모드, 그리고 관절마다 목표, 위치, 속도, 부하, 온도, 전압, 전류, 상태, 토크 스위치.
const RUN_FIELDS: [&str; 9] = [
    "goal",
    "position",
    "speed",
    "load",
    "temperature",
    "voltage",
    "current",
    "status",
    "torque",
];

fn mode_code(mode: &Mode) -> i32 {
    match mode {
        Mode::Holding => 0,
        Mode::Moving => 1,
        Mode::Stopping => 2,
        Mode::Jogging => 3,
        Mode::Fault(_) => 9,
    }
}

fn run_row<B: ServoBus>(run: &RunLoop<B>) -> Vec<i32> {
    let mut row = vec![mode_code(run.mode())];
    for (_, state) in run.states() {
        row.extend([
            state.goal_position,
            state.position,
            state.speed,
            state.load,
            state.temperature,
            state.voltage,
            state.current,
            i32::from(state.status.0),
            state.torque_enable,
        ]);
    }
    row
}

/// 사람이 볼 한 줄: 관절마다 각, 부하, 온도 (실행 루프가 판단에 쓰는 값. 기록에는 읽은 값 그대로 남는다).
fn run_status<B: ServoBus>(run: &RunLoop<B>, settings: &Settings, names: &[&str]) -> String {
    let parts: Vec<String> = run
        .states()
        .iter()
        .zip(names)
        .map(|((id, state), name)| {
            let temperature = run
                .temperatures()
                .iter()
                .find(|(known, _)| known == id)
                .map_or(state.temperature, |&(_, temperature)| temperature);
            let load = f64::from(state.load) / 10.0;
            if *name == GRIPPER_NAME {
                let width = run.gripper().map_or(f64::NAN, |(width, _)| width);
                return format!("그리퍼 {width:.1} mm 부하 {load:.1}% {temperature}°C");
            }
            let angle = settings
                .joint(name)
                .and_then(|joint| joint.calibration)
                .map_or(f64::NAN, |c| {
                    c.angle_at(f64::from(state.position)).to_degrees()
                });
            format!("{name} {angle:.1}° 부하 {load:.1}% {temperature}°C")
        })
        .collect();
    parts.join(" | ")
}

/// 실행 루프 스레드에서 기록 스레드로 넘기는 것. 실행 루프는 파일에 쓰지 않는다 (0026의 5).
enum Record {
    Sample {
        t: f64,
        step: String,
        values: Vec<i32>,
    },
    Event {
        t: f64,
        step: String,
        text: String,
    },
}

/// 실행 루프 스레드에서 터미널(이 프로그램의 첫 스레드)로 넘기는 줄.
enum Line {
    Say(String),
    Status(String),
}

/// 기록 줄의 크기: 박자마다 하나라 10초 몫이다. 차면 실행 루프는 기다리지 않고 버리고 센다 (0026의 5)
const RECORD_QUEUE: usize = 1000;
/// 터미널 줄의 크기. 차면 버린다 (끝난 까닭은 기록과 마지막 줄에 남는다)
const LINE_QUEUE: usize = 200;
/// 기록 스레드가 디스크에 확정하는 간격 (0026의 7)
const RECORD_SYNC: Duration = Duration::from_secs(1);
/// 이만큼 넘게 밀리면 밀린 박자를 따라잡으려 서두르지 않고 지금부터 다시 센다 [ns]
const CATCH_UP_LIMIT_NS: u64 = 200_000_000;
/// 깨어나는 지연의 기준 (0026의 2의 통과 기준) [µs]. 넘은 박자를 센다
const WAKE_LIMIT_US: u64 = 1_000;

/// 실행 루프가 순서의 사람 쪽 일을 넘기는 곳: 말은 터미널 줄로 보내고(기다리지 않는다), Enter는 표준 입력을 읽는
/// 스레드가 세운 표시로 본다.
struct LoopConsole {
    lines: SyncSender<Line>,
    enter: Arc<AtomicBool>,
}

impl Console for LoopConsole {
    fn say(&mut self, text: &str) {
        let _ = self.lines.try_send(Line::Say(text.to_owned()));
    }

    /// 실행 루프는 묻지 않는다 (순서는 Enter만 받는다).
    fn ask(&mut self, _question: &str) -> String {
        String::new()
    }

    fn begin_wait(&mut self, prompt: &str) {
        self.enter.store(false, Ordering::Relaxed); // 앞서 누른 Enter를 버린다
        let _ = self.lines.try_send(Line::Say(prompt.to_owned()));
    }

    fn enter_pressed(&mut self) -> bool {
        self.enter.swap(false, Ordering::Relaxed)
    }

    fn status(&mut self, text: &str) {
        let _ = self.lines.try_send(Line::Status(text.to_owned()));
    }
}

/// 표준 입력의 줄마다 Enter 표시를 세운다. 입력이 닫히면 끝난다 (Enter로 보지 않는다).
fn watch_enter(enter: Arc<AtomicBool>) {
    let _ = thread::Builder::new()
        .name("stdin".to_owned())
        .spawn(move || {
            system::make_background_thread();
            for line in io::stdin().lock().lines() {
                if line.is_err() {
                    break;
                }
                enter.store(true, Ordering::Relaxed);
            }
        });
}

/// 실행 루프의 박자 시간.
#[derive(Debug, Default, Clone, Copy)]
struct LoopStats {
    ticks: u64,
    wake_max_us: u64,
    tick_max_us: u64,
    /// 깨어나는 지연이 기준(1 ms)을 넘은 박자
    wake_over: u64,
    /// 한 주기보다 늦게 깬 박자
    late: u64,
}

impl LoopStats {
    fn add(&mut self, wake_us: u64, tick_us: u64, period_us: u64) {
        self.ticks += 1;
        self.wake_max_us = self.wake_max_us.max(wake_us);
        self.tick_max_us = self.tick_max_us.max(tick_us);
        self.wake_over += u64::from(wake_us > WAKE_LIMIT_US);
        self.late += u64::from(wake_us > period_us);
    }

    fn to_json(self) -> Value {
        json!({
            "ticks": self.ticks,
            "wake_max_us": self.wake_max_us,
            "tick_max_us": self.tick_max_us,
            "wake_limit_us": WAKE_LIMIT_US,
            "wake_over_limit": self.wake_over,
            "late_ticks": self.late,
        })
    }
}

/// 실행 루프 스레드에 넘기는 것.
struct LoopJob {
    script: Script,
    settings: Settings,
    /// 실행 루프를 실시간 스레드로 돌린다 (코어 3, SCHED_FIFO 80)
    realtime: bool,
}

/// 실행 루프 스레드가 끝낸 모양.
struct LoopEnd {
    status: Status,
    released: bool,
    mode: String,
    stats: LoopStats,
}

fn micros_i32(us: u64) -> i32 {
    i32::try_from(us).unwrap_or(i32::MAX)
}

fn push_event(records: &SyncSender<Record>, dropped: &AtomicU64, t: f64, step: &str, text: String) {
    let event = Record::Event {
        t,
        step: step.to_owned(),
        text,
    };
    if records.try_send(event).is_err() {
        dropped.fetch_add(1, Ordering::Relaxed);
    }
}

/// 실행 루프를 실시간으로 돌리지 않는 까닭. 리눅스에서 실물 서보에 붙으면 실시간으로 돈다 (없다).
fn realtime_off_reason(options: &Options) -> Option<&'static str> {
    if options.no_realtime {
        Some("--no-realtime")
    } else if !cfg!(target_os = "linux") {
        Some("리눅스가 아니다")
    } else if options.sim.is_some() {
        Some("시뮬레이션에 붙었다")
    } else {
        None
    }
}

/// 시험 순서를 돌린다. 이 스레드는 터미널을 맡고, 실행 루프와 기록 쓰기는 따로 스레드를 둔다 (0026의 2, 5):
/// 실행 루프 스레드는 실시간 설정을 건 뒤에 토크를 켠다 (못 걸면 서보에 손대지 않고 끝낸다).
fn drive_script<B: ServoBus + Send + 'static>(
    options: &Options,
    source: &Source,
    script: Script,
    joints: Joints<B>,
    link: Value,
) -> Result<(), String> {
    // 기록 칸과 상태 줄의 서보 이름 (관절 층의 순서: 관절 번호 순, 그리퍼는 마지막)
    let mut names = script.joints.clone();
    if script.uses_gripper() {
        names.push(GRIPPER_NAME);
    }
    let off = realtime_off_reason(options);
    let environment = Environment::read();
    let started = Local::now();
    let folder = record::new_record_dir(&options.records, "run", "run", &started)?;
    let meta_path = folder.join("meta.json");
    let mut meta = json!({
        "kind": "run",
        "started": record::time_text(&started),
        "link": link,
        "script_name": script.name,
        "script": source.script_text,
        "robot_toml": source.robot_toml,
        "collision": source.collision,
        "hold_max_s": source.hold_max_s,
        "period_s": LOOP_PERIOD.as_secs_f64(),
        "joints": names,
        "program": system::version(),
        "environment": environment.to_json(),
        "realtime": match off {
            None => "실행 루프: 코어 3, SCHED_FIFO 80, 메모리 잠금".to_owned(),
            Some(why) => format!("끔: {why}"),
        },
    });
    record::write_json(&meta_path, &meta)?;
    if off.is_none() {
        for finding in environment.findings().iter().filter(|f| !f.ok) {
            println!("알림: {}: {}", finding.name, finding.detail);
        }
        // 다른 스레드를 띄우기 전에 잠가서 모든 스레드의 스택이 잠긴다
        if let Err(e) = rt::lock_memory() {
            let error = format!(
                "메모리를 잠그지 못했다: {e}. {}. 실시간 없이 돌리려면 --no-realtime을 붙인다",
                system::LIMITS_HINT
            );
            return finish_failed(&meta_path, &mut meta, error);
        }
    }
    let mut columns = vec![
        "mode".to_owned(),
        "wake_us".to_owned(),
        "tick_us".to_owned(),
    ];
    for name in &names {
        for field in RUN_FIELDS {
            columns.push(format!("{name}_{field}"));
        }
    }
    let column_refs: Vec<&str> = columns.iter().map(String::as_str).collect();
    let log = Log::create_batched(&folder, &column_refs)?;
    let (record_tx, record_rx) = mpsc::sync_channel::<Record>(RECORD_QUEUE);
    let writer = thread::Builder::new()
        .name("records".to_owned())
        .spawn(move || write_records(record_rx, log))
        .map_err(|e| format!("기록 스레드를 띄우지 못했다: {e}"))?;

    let (ready_tx, ready_rx) = mpsc::channel();
    let (line_tx, line_rx) = mpsc::sync_channel::<Line>(LINE_QUEUE);
    let enter = Arc::new(AtomicBool::new(false));
    let dropped = Arc::new(AtomicU64::new(0));
    let job = LoopJob {
        script,
        settings: source.settings.clone(),
        realtime: off.is_none(),
    };
    let console = LoopConsole {
        lines: line_tx,
        enter: Arc::clone(&enter),
    };
    let loop_dropped = Arc::clone(&dropped);
    let runner = thread::Builder::new()
        .name("runloop".to_owned())
        .stack_size(rt::PREFAULT_STACK + 1024 * 1024)
        .spawn(move || drive_loop(job, joints, &ready_tx, &record_tx, console, &loop_dropped))
        .map_err(|e| format!("실행 루프 스레드를 띄우지 못했다: {e}"))?;
    let loop_thread = match ready_rx.recv() {
        Ok(Ok(thread)) => thread,
        Ok(Err(error)) => {
            let _ = runner.join();
            let _ = writer.join();
            return finish_failed(&meta_path, &mut meta, error);
        }
        Err(_) => {
            let error = match runner.join() {
                Ok(Err(error)) => error,
                _ => "실행 루프 스레드가 준비 전에 멈췄다".to_owned(),
            };
            let _ = writer.join();
            return finish_failed(&meta_path, &mut meta, error);
        }
    };
    meta["loop_thread"] = loop_thread
        .as_ref()
        .map_or(Value::Null, LoopThread::to_json);
    record::write_json(&meta_path, &meta)?;
    match (&loop_thread, off) {
        (Some(t), _) => println!(
            "실행 루프: {} {}, 코어 {:?}, 메모리 잠금",
            t.policy, t.priority, t.affinity
        ),
        (None, Some(why)) => println!("실행 루프: 보통 스레드 ({why})"),
        (None, None) => {}
    }

    watch_enter(enter);
    let mut console = RunConsole {
        terminal: Terminal::closed_is_silent(),
        status_shown: false,
    };
    // 실행 루프가 끝나 줄을 닫을 때까지 받는다
    for line in line_rx {
        match line {
            Line::Say(text) => console.say(&text),
            Line::Status(text) => console.status(&text),
        }
    }
    console.end_status();
    let end = runner
        .join()
        .map_err(|_| "실행 루프 스레드가 죽었다".to_owned())??;
    let written = writer
        .join()
        .map_err(|_| "기록 스레드가 죽었다".to_owned())?;
    let dropped = dropped.load(Ordering::Relaxed);

    meta["end"] = json!(match &end.status {
        Status::Stopped(_) => "stopped",
        _ => "done",
    });
    meta["result"] = json!(match &end.status {
        Status::Stopped(reason) => reason.clone(),
        _ => "순서를 마쳤다".to_owned(),
    });
    meta["released"] = json!(end.released);
    meta["mode_at_end"] = json!(end.mode);
    meta["loop"] = end.stats.to_json();
    meta["dropped_records"] = json!(dropped);
    if let Err(error) = &written {
        meta["records_error"] = json!(error);
    }
    meta["ended"] = json!(record::time_text(&Local::now()));
    record::write_json(&meta_path, &meta)?;
    let stats = end.stats;
    println!(
        "실행 루프: {}박자, 깨어나는 지연 최악 {} µs (1 ms를 넘은 박자 {}), 한 박자 일 최악 {} µs, 버린 기록 {dropped}",
        stats.ticks, stats.wake_max_us, stats.wake_over, stats.tick_max_us
    );
    if let Err(error) = written {
        println!("알림: 기록을 끝까지 쓰지 못했다: {error}");
    }
    if !end.released {
        println!(
            "서보는 마지막 자리에서 버틴다 (토크 켜짐). 끝내려면 팔을 받친 뒤 + 플러그를 뺀다 (0016)"
        );
    }
    println!("기록: {}", folder.display());
    match end.status {
        Status::Stopped(reason) => Err(reason),
        _ => Ok(()),
    }
}

/// 실행 루프를 세우지 못하고 끝낸다: 까닭을 기록에 남긴다.
fn finish_failed(meta_path: &Path, meta: &mut Value, error: String) -> Result<(), String> {
    meta["end"] = json!("error");
    meta["result"] = json!(error);
    meta["ended"] = json!(record::time_text(&Local::now()));
    record::write_json(meta_path, meta)?;
    Err(error)
}

/// 기록 스레드: 실행 루프가 넘긴 줄을 파일에 쓰고 1초쯤마다 디스크에 확정한다. 실행 루프가 끝나 줄이 닫히면
/// 남은 것을 쓰고 돌아온다. 쓰지 못하면 멈추고, 실행 루프는 그 뒤의 기록을 버린 것으로 센다 (제어는 그대로 돈다).
fn write_records(rx: Receiver<Record>, mut log: Log) -> Result<(), String> {
    system::make_background_thread();
    let mut synced = Instant::now();
    loop {
        match rx.recv_timeout(Duration::from_millis(200)) {
            Ok(Record::Sample { t, step, values }) => log.sample(t, &step, &values)?,
            Ok(Record::Event { t, step, text }) => log.event(t, &step, &text)?,
            Err(RecvTimeoutError::Timeout) => {}
            Err(RecvTimeoutError::Disconnected) => return log.sync(),
        }
        if synced.elapsed() >= RECORD_SYNC {
            log.sync()?;
            synced = Instant::now();
        }
    }
}

/// 실행 루프 스레드: 실시간 설정을 걸고, 토크를 켜고(실행 루프 시작), 준비됐다고 알린 뒤 박자마다 실행 루프와
/// 시험 순서를 돌린다. 박자는 정한 시각까지 자는 방식이라 늦게 깬 만큼이 다음 박자로 쌓이지 않는다.
fn drive_loop<B: ServoBus>(
    job: LoopJob,
    joints: Joints<B>,
    ready: &Sender<Result<Option<LoopThread>, String>>,
    records: &SyncSender<Record>,
    mut console: LoopConsole,
    dropped: &AtomicU64,
) -> Result<LoopEnd, String> {
    let fail = |error: String| {
        let _ = ready.send(Err(error.clone()));
        error
    };
    let thread = if job.realtime {
        Some(system::make_loop_thread().map_err(fail)?)
    } else {
        None
    };
    let gripper = job
        .settings
        .gripper
        .as_ref()
        .filter(|_| job.script.uses_gripper());
    let mut run = RunLoop::start(joints, job.settings.run_loop, gripper).map_err(fail)?;
    let _ = ready.send(Ok(thread));

    let mut names = job.script.joints.clone();
    if gripper.is_some() {
        names.push(GRIPPER_NAME);
    }
    let mut runner = Runner::new(
        job.script,
        &job.settings.run_loop,
        LOOP_PERIOD.as_secs_f64(),
    );
    let period_ns = u64::try_from(LOOP_PERIOD.as_nanos()).unwrap_or(u64::MAX);
    let period_us = period_ns / 1_000;
    let mut stats = LoopStats::default();
    let mut seen = BTreeSet::new();
    let start = rt::now_ns();
    let mut next = start;
    let mut last = start;
    let mut tick = |run: &mut RunLoop<B>, label: &str, stats: &mut LoopStats| -> f64 {
        next += period_ns;
        if rt::now_ns() > next + CATCH_UP_LIMIT_NS {
            next = rt::now_ns(); // 많이 밀렸으면 따라잡으려 서두르지 않는다
        }
        rt::sleep_until_ns(next);
        let woke = rt::now_ns();
        let dt = ((woke - last) as f64 / 1e9).max(1e-4);
        last = woke;
        run.tick(dt);
        let wake_us = woke.saturating_sub(next) / 1_000;
        let tick_us = rt::now_ns().saturating_sub(woke) / 1_000;
        stats.add(wake_us, tick_us, period_us);
        let mut values = run_row(run);
        values.splice(1..1, [micros_i32(wake_us), micros_i32(tick_us)]);
        let t = (woke - start) as f64 / 1e9;
        let sample = Record::Sample {
            t,
            step: label.to_owned(),
            values,
        };
        if records.try_send(sample).is_err() {
            dropped.fetch_add(1, Ordering::Relaxed);
        }
        t
    };
    let status = loop {
        let label = runner.label();
        let now = tick(&mut run, &label, &mut stats);
        let status = runner.tick(&mut run, &mut console, now);
        for event in runner.take_events() {
            push_event(records, dropped, now, &label, event);
        }
        for warning in run.warnings() {
            if seen.insert(warning.clone()) {
                console.say(&format!("알림: {warning}"));
                push_event(records, dropped, now, &label, warning);
            }
        }
        if stats.ticks.is_multiple_of(50) {
            console.status(&run_status(&run, &job.settings, &names));
        }
        if status != Status::Running {
            break status;
        }
    };
    // 멈췄으면 감속이 끝나 붙잡을 때까지 박자를 더 돌린다 (그 전에 끝내면 마지막 목표에서 갑자기 선다)
    if matches!(status, Status::Stopped(_)) {
        for _ in 0..1000 {
            if matches!(run.mode(), Mode::Holding | Mode::Fault(_)) {
                break;
            }
            tick(&mut run, "settle", &mut stats);
        }
    }
    Ok(LoopEnd {
        status,
        released: runner.released(),
        mode: format!("{:?}", run.mode()),
        stats,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn args(text: &str) -> impl Iterator<Item = String> {
        text.split_whitespace()
            .map(str::to_owned)
            .collect::<Vec<_>>()
            .into_iter()
    }

    #[test]
    fn settings_command_reads_the_file_option() {
        let options = parse_args(args("settings")).unwrap();
        assert_eq!(options.command, Command::Settings);
        assert_eq!(options.settings, PathBuf::from("data/robot.toml"));
        let options = parse_args(args("--settings other.toml settings")).unwrap();
        assert_eq!(options.settings, PathBuf::from("other.toml"));
    }

    #[test]
    fn apply_reads_port_joints_and_yes() {
        let options = parse_args(args("apply --port COM5")).unwrap();
        assert_eq!(options.command, Command::Apply);
        assert_eq!(options.port.as_deref(), Some("COM5"));
        assert!(options.joints.is_empty() && !options.yes);
        let options = parse_args(args(
            "apply --port COM5 --joint j3 --joint j2 --yes --records r",
        ))
        .unwrap();
        assert_eq!(options.joints, ["j3", "j2"]);
        assert!(options.yes);
        assert_eq!(options.records, PathBuf::from("r"));
    }

    #[test]
    fn bad_arguments_are_refused() {
        assert!(parse_args(args("")).is_err());
        assert!(parse_args(args("move")).is_err());
        assert!(parse_args(args("settings settings")).is_err());
        assert!(parse_args(args("settings --settings")).is_err());
        assert!(parse_args(args("apply")).is_err()); // --port가 없다
        assert!(parse_args(args("settings --what")).is_err());
        assert_eq!(parse_args(args("--help")).unwrap_err(), "");
    }

    #[test]
    fn check_needs_port_and_id() {
        let options = parse_args(args("check --port COM5 --id 20")).unwrap();
        assert_eq!(options.command, Command::Check);
        assert_eq!(options.id, Some(20));
        assert!(!options.yes);
        let options = parse_args(args("check-state --port COM5 --id 20")).unwrap();
        assert_eq!(options.command, Command::CheckState);
        assert_eq!(options.situation, None);
        let options = parse_args(args("check-state --port COM5 --id 20 --situation 3")).unwrap();
        assert_eq!(options.situation, Some(3));
        assert!(parse_args(args("check-state --port COM5 --id 20 --situation 5")).is_err());
        assert!(parse_args(args("check --port COM5")).is_err());
        assert!(parse_args(args("check --id 20")).is_err());
        assert!(parse_args(args("check --port COM5 --id 300")).is_err());
    }

    #[test]
    fn zero_and_run_take_their_options() {
        let options = parse_args(args("zero --port COM5 --joint j2 --direction -1")).unwrap();
        assert_eq!(options.command, Command::Zero);
        assert_eq!(options.direction, Some(Direction::Reversed));
        assert!(parse_args(args("zero --port COM5")).is_err()); // 관절이 없다
        assert!(parse_args(args("zero --port COM5 --joint j2 --joint j3")).is_err());
        assert!(parse_args(args("zero --port COM5 --joint j2 --direction 2")).is_err());
        let options = parse_args(args("run --script a.toml --sim 127.0.0.1:5600 --yes")).unwrap();
        assert_eq!(options.command, Command::Run);
        assert_eq!(options.script, Some(PathBuf::from("a.toml")));
        assert_eq!(options.sim.as_deref(), Some("127.0.0.1:5600"));
        assert!(parse_args(args("run --script a.toml")).is_err()); // 붙을 곳이 없다
        assert!(parse_args(args("run --script a.toml --port COM5 --sim x:1")).is_err());
        assert!(parse_args(args("run --port COM5")).is_err()); // 순서가 없다
        let options = parse_args(args("run --script a.toml --sim x:1 --hold-max 5")).unwrap();
        assert_eq!(options.hold_max, Some(5.0));
        // 버티기를 줄이는 것은 시뮬레이션에서만이다
        assert!(parse_args(args("run --script a.toml --port COM5 --hold-max 5")).is_err());
        assert!(parse_args(args("run --script a.toml --sim x:1 --hold-max 0")).is_err());
    }

    #[test]
    fn check_takes_step_names() {
        let options =
            parse_args(args("check --port COM5 --id 20 --only state,voltage --yes")).unwrap();
        assert_eq!(options.only, ["state", "voltage"]);
        let error = parse_args(args("check --port COM5 --id 20 --only volt")).unwrap_err();
        assert!(error.contains("모르는 단계"), "{error}");
    }

    #[test]
    fn check_runs_only_on_spare_ids() {
        let options = parse_args(args("check --port COM99 --id 3 --yes")).unwrap();
        assert!(run_check(&options).unwrap_err().contains("예비 서보"));
    }

    #[test]
    fn last_check_results_finds_the_newest_check_of_that_servo() {
        let root = std::env::temp_dir().join(format!("horibot-lock-test-{}", std::process::id()));
        let servo = root.join("servo");
        for (name, id, p) in [
            ("2026-11-12_100000_check-a", 20, Some(32)),
            ("2026-11-12_110000_check-a", 20, Some(30)),
            ("2026-11-12_113000_check-a", 20, None), // 잠금 단계 전에 끊겼다
            ("2026-11-12_120000_check-a", 21, Some(28)),
        ] {
            fs::create_dir_all(servo.join(name)).unwrap();
            let lock = p.map_or(json!({}), |p| json!({ "p_before": p, "p_trial": p + 8 }));
            let meta = json!({
                "kind": "servo-check",
                "servo": { "id": id },
                "results": { "lock": lock },
            });
            record::write_json(&servo.join(name).join("meta.json"), &meta).unwrap();
        }
        let results = last_check_results(&root, 20).unwrap();
        assert_eq!(results["lock"]["p_before"], 30);
        assert!(last_check_results(&root, 5).is_none());
        fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn joints_can_be_selected() {
        let settings = Settings::load(&PathBuf::from(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../data/robot.toml"
        )))
        .unwrap();
        assert_eq!(
            selected(&settings, &[]).unwrap().len(),
            settings.joints.len()
        );
        assert_eq!(
            selected(&settings, &["j3".to_owned()]).unwrap()[0].name,
            "j3"
        );
        assert!(selected(&settings, &["j7".to_owned()]).is_err());
    }

    #[test]
    fn timing_takes_its_options() {
        let options = parse_args(args("timing --port COM5 --count 50 --no-realtime")).unwrap();
        assert_eq!(options.command, Command::Timing);
        assert_eq!(options.count, Some(50));
        assert!(options.no_realtime && !options.yes);
        assert!(parse_args(args("timing")).is_err(), "포트가 있어야 한다");
        assert!(parse_args(args("timing --port COM5 --count 0")).is_err());
    }

    /// 통신 시간 명령을 가짜 서보로 끝까지 돌린다: 재고, Python 기록과 같은 꼴로 남기고, 서보는 그대로다.
    #[test]
    fn timing_measures_fake_servos_and_records_like_the_python_command() {
        use horibot_servo::feetech::fake::{FakeServo, open_fake_bus};

        let settings = Settings::load(&PathBuf::from(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../data/robot.toml"
        )))
        .unwrap();
        let servos = selected(&settings, &["j1".to_owned(), "j5".to_owned()]).unwrap();
        let fakes = servos
            .iter()
            .map(|joint| FakeServo::with(joint.id, joint.model_number, 2048));
        let root = std::env::temp_dir().join(format!("horibot-timing-test-{}", std::process::id()));
        let _ = fs::remove_dir_all(&root);
        let text = format!(
            "timing --port fake --count 3 --yes --no-realtime --records {}",
            root.display()
        );
        let options = parse_args(args(&text)).unwrap();
        let folder = time_servos(
            &options,
            open_fake_bus(fakes),
            &servos,
            json!({}),
            1_000_000,
        )
        .unwrap()
        .expect("기록 폴더");
        let meta: Value =
            serde_json::from_str(&fs::read_to_string(folder.join("meta.json")).unwrap()).unwrap();
        assert_eq!(meta["kind"], "servo-timing");
        assert_eq!(meta["ids"], json!([1, 5]));
        assert_eq!(meta["summaries"].as_array().unwrap().len(), 4);
        assert_eq!(meta["tick_write"], "torque_limit");
        assert!(meta["realtime"].as_str().unwrap().starts_with("끔"));
        let samples = fs::read_to_string(folder.join("samples.csv")).unwrap();
        assert_eq!(samples.lines().count(), 1 + 4 * 3);
        assert!(folder.starts_with(root.join("servo")));
        fs::remove_dir_all(&root).unwrap();
    }
}
