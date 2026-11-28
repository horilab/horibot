//! 시험 값 (docs/decisions/0042): 실행 명령의 `--trial`로 이번 실행에만 서보 게인, 서보 안 가속 상한, 실행 루프의
//! 처짐 보정 이득을 바꿔 보고, 순서가 끝나면 되돌린다. 떨림과 처짐을 줄일 설정을 다 만든 팔에서 재서 고르는 데 쓴다.
//!
//! `--trial [<관절>[,<관절>…]:]<이름>=<값>`을 여러 번 준다. 관절을 적지 않으면 순서의 모든 팔 관절이다 (그리퍼는
//! 바꾸지 않는다). 예: `--trial j2,j3:p_gain=48 --trial acceleration=0`.
//!
//! | 이름 | 무엇 | 받는 값 |
//! |---|---|---|
//! | `p_gain` | 서보 게인 P(21) | 16 ~ 64 |
//! | `d_gain` | 서보 게인 D(22) | 32 ~ 64 |
//! | `i_gain` | 서보 게인 I(23) | 0 ~ 4 |
//! | `acceleration` | 서보 안 가속 상한(41, 100 step/s²) | 0(서보의 최대), 또는 토크를 켤 때 쓰는 값([`Caps::for_loop`]) ~ 254 |
//! | `sag_gain_per_s` | 실행 루프의 처짐 보정 이득 [1/s] (관절을 적지 않는다) | 0 ~ 5 |
//!
//! - 받는 범위는 0042의 재는 순서(P 32 → 48 → 64, D 32 → 64, I 게인 둘쯤, 처짐 보정 이득 셋쯤)를 덮는 만큼이다. 팔을
//!   망가뜨리지 않으려고 좁게 둔다: P를 크게 올리면 버틸 때 흔들리고(헌팅) 서보가 뜨거워질 수 있고, 낮추면 짐을 못
//!   버틴다. D는 지금 값(32) 밑으로 내리지 않는다: 흔들림을 잡는 몫이라 낮추면 흔들리기 쉬워진다. I 게인은 메모리 표에
//!   크기의 단위가 없어 1부터 올린다. 가속 상한을 토크를 켤 때 쓰는 값보다 낮추면 서보가 궤적을 늦게 따라가서 받지
//!   않는다. 넓힐 때는 0042와 같이 고친다.
//! - 게인은 토크를 켜고 지금 자리를 붙잡은 뒤 잠근 채 쓴다 (바로 듣고, 서보 전원을 끄면 사라진다, 0024). 토크를 켤
//!   때 서보의 EEPROM 값을 로봇 설정(값 적용의 목표)과 견주므로([`Joints::enable`]) 먼저 쓰면 켜지 않는다. 가속
//!   상한은 토크를 켤 때 쓰는 값 위에 쓴다.
//! - 순서가 끝나면(마쳤거나, 사람이 세웠거나, 고장) 쓰기 전에 읽은 값으로 되돌린다: 다음 실행이 시험 값을 물려받지
//!   않고, 버틸 때 흔들리면 B로 세워 처음 값으로 돌아간다. 프로그램이 죽어 되돌리지 못하면, 다음 실행은 토크를 켤 때
//!   서보의 EEPROM 값이 로봇 설정과 다르다며 켜지 않는다: 팔을 받치고 서보 전원을 껐다 켠다.
//! - 쓴 값과 쓰기 전 값, 되돌렸는지는 실행 기록(meta.json)의 `trial`에, 모든 서보의 게인과 가속 상한은 `servo_values`에
//!   남는다.

use horibot_servo::feetech::ServoBus;
use horibot_servo::feetech::registers::{self as reg, Register};
use serde_json::{Map, Value, json};

use crate::joints::Caps;
#[cfg(doc)]
use crate::joints::Joints;
use crate::runloop::RunLoop;
use crate::settings::Settings;

/// 서보 게인의 받는 범위: (레지스터, 가장 작은 값, 가장 큰 값). 이름은 레지스터 이름이다
const GAINS: [(Register, i32, i32); 3] = [
    (reg::P_GAIN, 16, 64),
    (reg::D_GAIN, 32, 64),
    (reg::I_GAIN, 0, 4),
];
/// 처짐 보정 이득의 가장 큰 값 [1/s]: 시간 상수 0.2초(박자 20개). 처짐 보정은 서보 바깥의 느린 고리라, 너무 빠르면
/// 서보와 맞물려 흔들릴 수 있다
const MAX_SAG_GAIN: f64 = 5.0;
/// 처짐 보정 이득의 이름 (로봇 설정의 키와 같다)
const SAG_GAIN: &str = "sag_gain_per_s";
/// 실행 기록에 남기는 서보 값
const RECORDED: [Register; 4] = [reg::P_GAIN, reg::D_GAIN, reg::I_GAIN, reg::ACCELERATION];

/// 서보 하나에 쓸 시험 값.
#[derive(Debug, Clone, PartialEq)]
pub struct ServoTrial {
    pub joint: &'static str,
    pub id: u8,
    pub values: Vec<(Register, i32)>,
}

/// 이번 실행에만 바꿀 값 (`--trial`).
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Trials {
    /// 서보마다 (관절 번호 순)
    pub servos: Vec<ServoTrial>,
    /// 실행 루프의 처짐 보정 이득 [1/s]
    pub sag_gain: Option<f64>,
}

/// 서보 값 하나의 이름과 받는 범위를 본다. 맞으면 레지스터를 낸다. `lowest_acceleration`은 토크를 켤 때 쓰는 가속
/// 상한이다.
fn servo_register(name: &str, value: i32, lowest_acceleration: i32) -> Result<Register, String> {
    if let Some(&(register, low, high)) = GAINS.iter().find(|(known, ..)| known.name == name) {
        return if (low..=high).contains(&value) {
            Ok(register)
        } else {
            Err(format!("{name}는 {low} ~ {high}다: {value}"))
        };
    }
    if name == reg::ACCELERATION.name {
        return if value == 0 || (lowest_acceleration..=254).contains(&value) {
            Ok(reg::ACCELERATION)
        } else {
            Err(format!(
                "acceleration은 0(서보의 최대)이나 {lowest_acceleration}(토크를 켤 때 쓰는 값) ~ 254다: {value} \
                 (낮추면 서보가 궤적을 늦게 따라간다)"
            ))
        };
    }
    Err(format!(
        "모르는 이름이다: {name} (p_gain, d_gain, i_gain, acceleration, {SAG_GAIN})"
    ))
}

impl Trials {
    /// `--trial`의 값들을 읽는다. `joints`는 순서의 팔 관절이다. 잘못된 것은 모두 모아 알린다.
    pub fn parse(
        args: &[String],
        joints: &[&'static str],
        settings: &Settings,
    ) -> Result<Self, String> {
        let lowest_acceleration = Caps::for_loop(&settings.run_loop).acceleration;
        let mut trials = Self::default();
        let mut problems = Vec::new();
        for arg in args {
            let mut problem = |text: String| problems.push(format!("--trial {arg}: {text}"));
            let Some((left, value)) = arg.split_once('=') else {
                problem("<이름>=<값>으로 준다 (예: j2:p_gain=48)".to_owned());
                continue;
            };
            let value = value.trim();
            let (names, name) = match left.split_once(':') {
                Some((names, name)) => (Some(names), name.trim()),
                None => (None, left.trim()),
            };
            if name == SAG_GAIN {
                if names.is_some() {
                    problem("처짐 보정 이득은 실행 루프 하나라 관절을 적지 않는다".to_owned());
                    continue;
                }
                match value.parse::<f64>() {
                    Ok(gain) if (0.0..=MAX_SAG_GAIN).contains(&gain) => {
                        if trials.sag_gain.replace(gain).is_some() {
                            problem(format!("{SAG_GAIN}를 두 번 적었다"));
                        }
                    }
                    _ => problem(format!("{SAG_GAIN}는 0 ~ {MAX_SAG_GAIN}다: {value}")),
                }
                continue;
            }
            let Ok(number) = value.parse::<i32>() else {
                problem(format!("{name}의 값은 정수다: {value}"));
                continue;
            };
            let register = match servo_register(name, number, lowest_acceleration) {
                Ok(register) => register,
                Err(text) => {
                    problem(text);
                    continue;
                }
            };
            let chosen: Vec<&'static str> = match names {
                None => joints.to_vec(),
                Some(list) => list
                    .split(',')
                    .map(str::trim)
                    .filter_map(|wanted| {
                        let found = joints.iter().copied().find(|known| *known == wanted);
                        if found.is_none() {
                            problem(format!(
                                "{wanted}은 순서의 팔 관절이 아니다 ({})",
                                joints.join(", ")
                            ));
                        }
                        found
                    })
                    .collect(),
            };
            for joint in chosen {
                let Some(id) = settings.joint(joint).map(|known| known.id) else {
                    problem(format!("로봇 설정에 {joint}이 없다"));
                    continue;
                };
                let index = match trials.servos.iter().position(|servo| servo.joint == joint) {
                    Some(index) => index,
                    None => {
                        trials.servos.push(ServoTrial {
                            joint,
                            id,
                            values: Vec::new(),
                        });
                        trials.servos.len() - 1
                    }
                };
                let values = &mut trials.servos[index].values;
                if values.iter().any(|(known, _)| *known == register) {
                    problem(format!("{joint}의 {name}를 두 번 적었다"));
                } else {
                    values.push((register, number));
                }
            }
        }
        if !problems.is_empty() {
            return Err(problems.join("\n"));
        }
        trials.servos.sort_by_key(|servo| servo.id);
        Ok(trials)
    }

    pub fn is_empty(&self) -> bool {
        self.servos.is_empty() && self.sag_gain.is_none()
    }

    /// 사람이 볼 한 줄.
    pub fn describe(&self) -> String {
        let mut parts: Vec<String> = self
            .servos
            .iter()
            .map(|servo| {
                let values: Vec<String> = servo
                    .values
                    .iter()
                    .map(|(register, value)| format!("{} {value}", register.name))
                    .collect();
                format!("{} {}", servo.joint, values.join(", "))
            })
            .collect();
        if let Some(gain) = self.sag_gain {
            parts.push(format!("처짐 보정 이득 {gain}/s"));
        }
        format!(
            "시험 값 (이번 실행에만, 끝나면 되돌린다): {}",
            parts.join(" | ")
        )
    }

    /// 기록(meta.json의 trial.asked): 관절마다 {이름: 값}과 처짐 보정 이득.
    pub fn to_json(&self) -> Value {
        let servos: Map<String, Value> = self
            .servos
            .iter()
            .map(|servo| {
                let values: Map<String, Value> = servo
                    .values
                    .iter()
                    .map(|(register, value)| (register.name.to_owned(), json!(value)))
                    .collect();
                (servo.joint.to_owned(), Value::Object(values))
            })
            .collect();
        let mut found = Map::new();
        found.insert("servos".to_owned(), Value::Object(servos));
        found.insert(SAG_GAIN.to_owned(), json!(self.sag_gain));
        Value::Object(found)
    }

    /// 서보마다 쓰기 전 값을 읽고 시험 값을 쓴다. 실행 루프를 세운 뒤(토크를 켜고 지금 자리를 붙잡은 뒤) 첫 박자
    /// 전에 부른다. 하나라도 못 쓰면 그때까지 쓴 것을 되돌리고 오류다 (팔은 붙잡은 자리에서 버틴다). 처짐 보정 이득은
    /// 실행 루프를 세울 때 로봇 설정에 넣어 준다 (여기서 쓰지 않는다).
    pub fn apply<B: ServoBus>(&self, run: &mut RunLoop<B>) -> Result<Applied, String> {
        let mut applied = Applied::default();
        for servo in &self.servos {
            let registers: Vec<Register> =
                servo.values.iter().map(|(register, _)| *register).collect();
            let before = match run.read_servo(servo.id, &registers) {
                Ok(before) => before,
                Err(e) => {
                    let error = format!("{}: 쓰기 전 값을 읽지 못했다: {e}", servo.joint);
                    return Err(applied.undo_after(run, error));
                }
            };
            applied.servos.push(Written {
                joint: servo.joint,
                id: servo.id,
                values: before
                    .iter()
                    .zip(&servo.values)
                    .map(|(&(register, old), &(_, new))| (register, old, new))
                    .collect(),
            });
            if let Err(e) = run.set_servo_values(servo.id, &servo.values) {
                let error = format!("{}: 시험 값을 쓰지 못했다: {e}", servo.joint);
                return Err(applied.undo_after(run, error));
            }
        }
        Ok(applied)
    }
}

/// 쓴 시험 값: 순서가 끝나면 이것으로 되돌린다.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Applied {
    pub servos: Vec<Written>,
}

/// 서보 하나에 쓴 시험 값.
#[derive(Debug, Clone, PartialEq)]
pub struct Written {
    pub joint: &'static str,
    pub id: u8,
    /// (레지스터, 쓰기 전 값, 쓴 값)
    pub values: Vec<(Register, i32, i32)>,
}

impl Applied {
    pub fn is_empty(&self) -> bool {
        self.servos.is_empty()
    }

    /// 쓰기 전 값으로 되돌린다. 한 서보를 못 되돌려도 나머지는 되돌리고, 못 한 것을 모아 알린다.
    pub fn restore<B: ServoBus>(&self, run: &mut RunLoop<B>) -> Result<(), String> {
        let mut failed = Vec::new();
        for servo in &self.servos {
            let before: Vec<(Register, i32)> = servo
                .values
                .iter()
                .map(|&(register, old, _)| (register, old))
                .collect();
            if let Err(e) = run.set_servo_values(servo.id, &before) {
                failed.push(format!("{}: {e}", servo.joint));
            }
        }
        if failed.is_empty() {
            Ok(())
        } else {
            Err(format!(
                "시험 값을 되돌리지 못했다 ({}). 팔을 받치고 서보 전원을 껐다 켜면 처음 값으로 돌아간다",
                failed.join(", ")
            ))
        }
    }

    /// 쓰다 실패했을 때: 그때까지 쓴 것을 되돌리고, 그 결과를 오류 글에 붙인다.
    fn undo_after<B: ServoBus>(&self, run: &mut RunLoop<B>, error: String) -> String {
        match self.restore(run) {
            Ok(()) if self.is_empty() => error,
            Ok(()) => format!("{error}. 앞서 쓴 시험 값은 되돌렸다"),
            Err(undone) => format!("{error}. {undone}"),
        }
    }

    /// 기록(meta.json의 trial.written): 관절마다 {이름: {before, trial}}.
    pub fn to_json(&self) -> Value {
        let servos: Map<String, Value> = self
            .servos
            .iter()
            .map(|servo| {
                let values: Map<String, Value> = servo
                    .values
                    .iter()
                    .map(|&(register, old, new)| {
                        (
                            register.name.to_owned(),
                            json!({ "before": old, "trial": new }),
                        )
                    })
                    .collect();
                (servo.joint.to_owned(), Value::Object(values))
            })
            .collect();
        Value::Object(servos)
    }
}

/// 실행 기록에 남길 서보 값: 서보마다 게인(P, D, I)과 서보 안 가속 상한. 읽지 못한 서보는 까닭을 남긴다.
/// `servos`는 (이름, 서보 ID)다.
pub fn servo_values<B: ServoBus>(run: &mut RunLoop<B>, servos: &[(&str, u8)]) -> Value {
    let mut found = Map::new();
    for &(name, id) in servos {
        let value = match run.read_servo(id, &RECORDED) {
            Ok(values) => Value::Object(
                values
                    .iter()
                    .map(|(register, value)| (register.name.to_owned(), json!(value)))
                    .collect(),
            ),
            Err(e) => json!({ "error": e }),
        };
        found.insert(name.to_owned(), value);
    }
    Value::Object(found)
}

#[cfg(test)]
mod tests {
    use horibot_servo::feetech::Bus;
    use horibot_servo::feetech::fake::{FakePort, FakeServo, open_fake_bus};

    use super::*;
    use crate::joints::{Joints, Servo, programmed};
    use crate::settings::{TEST_GRIPPER, TEST_J3};

    /// j3(영점을 준 것)와 그리퍼가 있는 시험용 로봇 설정
    fn settings() -> Settings {
        let text = format!(
            "{}{TEST_GRIPPER}",
            TEST_J3.replace(
                "[joints.j3]",
                "[joints.j3]\nzero_raw = 2048\ndirection = 1\nposition_offset = 0",
            )
        );
        Settings::parse(&text).unwrap()
    }

    fn args(text: &[&str]) -> Vec<String> {
        text.iter().map(|arg| (*arg).to_owned()).collect()
    }

    /// j3와 그리퍼 서보의 실행 루프 (그리퍼는 시험 값을 받지 않지만, 쓰다 실패하는 서보로 쓴다)
    fn run_loop(settings: &Settings) -> RunLoop<Bus<FakePort>> {
        let gripper = settings.gripper.as_ref().unwrap();
        let servos = vec![
            Servo::from_joint(settings.joint("j3").unwrap()).unwrap(),
            Servo::from_joint(&gripper.joint).unwrap(),
        ];
        let bus = open_fake_bus(servos.iter().map(|servo| {
            programmed(
                FakeServo::with(servo.id, gripper.joint.model_number, 2048),
                servo,
            )
        }));
        let joints = Joints::open(bus, servos, Caps::for_loop(&settings.run_loop)).unwrap();
        RunLoop::start(joints, settings.run_loop, Some(gripper)).unwrap()
    }

    #[test]
    fn trial_values_are_read_and_checked() {
        let settings = settings();
        let trials = Trials::parse(
            &args(&["p_gain=48", "j3:acceleration=0", "sag_gain_per_s=2"]),
            &["j3"],
            &settings,
        )
        .unwrap();
        assert_eq!(
            trials.servos,
            [ServoTrial {
                joint: "j3",
                id: 3,
                values: vec![(reg::P_GAIN, 48), (reg::ACCELERATION, 0)],
            }]
        );
        assert_eq!(trials.sag_gain, Some(2.0));
        assert!(!trials.is_empty());
        let line = trials.describe();
        assert!(
            line.contains("j3 p_gain 48, acceleration 0") && line.contains("처짐 보정 이득 2/s"),
            "{line}"
        );
        assert_eq!(trials.to_json()["servos"]["j3"]["p_gain"], 48);
        assert!(Trials::parse(&[], &["j3"], &settings).unwrap().is_empty());
        // 받는 범위 밖, 모르는 이름, 순서에 없는 관절, 두 번, 관절을 붙인 처짐 보정 이득
        let lowest = Caps::for_loop(&settings.run_loop).acceleration;
        let error = Trials::parse(
            &args(&[
                "p_gain=80",
                "d_gain=8",
                "i_gain=5",
                &format!("acceleration={}", lowest - 1),
                "speed=3",
                "j9:p_gain=40",
                "j3:d_gain=40",
                "d_gain=48",
                "j3:sag_gain_per_s=1",
                "sag_gain_per_s=9",
                "p_gain",
                "p_gain=4.5",
            ]),
            &["j3"],
            &settings,
        )
        .unwrap_err();
        for expected in [
            "p_gain는 16 ~ 64다: 80",
            "d_gain는 32 ~ 64다: 8",
            "i_gain는 0 ~ 4다: 5",
            "토크를 켤 때 쓰는 값",
            "모르는 이름이다: speed",
            "j9은 순서의 팔 관절이 아니다",
            "j3의 d_gain를 두 번 적었다",
            "관절을 적지 않는다",
            "sag_gain_per_s는 0 ~ 5다: 9",
            "<이름>=<값>",
            "정수다: 4.5",
        ] {
            assert!(error.contains(expected), "{expected}: {error}");
        }
    }

    /// 시험 값은 토크를 켠 뒤 잠근 채 쓰고(EEPROM에 남지 않는다), 되돌리면 쓰기 전 값이다.
    #[test]
    fn trial_values_are_written_and_restored() {
        let settings = settings();
        let mut run = run_loop(&settings);
        let caps = Caps::for_loop(&settings.run_loop);
        let trials = Trials::parse(
            &args(&["p_gain=48", "d_gain=64", "acceleration=0"]),
            &["j3"],
            &settings,
        )
        .unwrap();
        let before = servo_values(&mut run, &[("j3", 3)]);
        assert_eq!(before["j3"]["p_gain"], 32);
        assert_eq!(before["j3"]["acceleration"], caps.acceleration);
        let applied = trials.apply(&mut run).unwrap();
        let servo = &run.joints_for_test().wire().servos[0];
        assert_eq!(
            (
                servo.get(reg::P_GAIN),
                servo.get(reg::D_GAIN),
                servo.get(reg::ACCELERATION),
                servo.get(reg::LOCK)
            ),
            (48, 64, 0, 1)
        );
        assert_eq!(servo.eeprom_saves, 0, "EEPROM에 굽지 않는다");
        assert_eq!(servo.get(reg::TORQUE_ENABLE), reg::TORQUE_ON);
        let written = applied.to_json();
        assert_eq!(
            written["j3"]["p_gain"],
            json!({ "before": 32, "trial": 48 })
        );
        applied.restore(&mut run).unwrap();
        let servo = &run.joints_for_test().wire().servos[0];
        assert_eq!(
            (
                servo.get(reg::P_GAIN),
                servo.get(reg::D_GAIN),
                servo.get(reg::ACCELERATION)
            ),
            (32, 32, caps.acceleration)
        );
    }

    /// 한 서보에 쓰다 실패하면 앞서 쓴 서보를 되돌린다.
    #[test]
    fn a_failed_trial_undoes_the_values_already_written() {
        let settings = settings();
        let mut run = run_loop(&settings);
        let trials = Trials {
            servos: vec![
                ServoTrial {
                    joint: "j3",
                    id: 3,
                    values: vec![(reg::P_GAIN, 48)],
                },
                ServoTrial {
                    joint: "gripper",
                    id: 11,
                    values: vec![(reg::P_GAIN, 48)],
                },
            ],
            sag_gain: None,
        };
        run.joints_for_test().wire().servos[1].faults.silent = true;
        let error = trials.apply(&mut run).unwrap_err();
        assert!(
            error.contains("gripper: 쓰기 전 값을 읽지 못했다") && error.contains("되돌렸다"),
            "{error}"
        );
        assert_eq!(run.joints_for_test().wire().servos[0].get(reg::P_GAIN), 32);
        // 게인과 가속 상한 말고는 쓰지 않는다
        let refused = run.set_servo_values(3, &[(reg::P_GAIN, 40), (reg::TORQUE_LIMIT, 1000)]);
        assert!(refused.unwrap_err().contains("torque_limit"));
        assert_eq!(run.joints_for_test().wire().servos[0].get(reg::P_GAIN), 32);
    }
}
