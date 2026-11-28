//! 실행 명령의 시험 순서 (docs/decisions/0034): 첫 동력 시험처럼 사람이 지켜보는 짧은 시험을 실제 실행 루프로
//! 돌린다. 순서는 TOML 파일 하나다:
//!
//! ```toml
//! name = "어깨 버티기"
//! joints = ["j1", "j2"]     # 켤 관절 (로봇 설정에 영점이 있어야 한다)
//! speed_percent = 50        # 실행 루프의 속도·가속·저크 한계에 곱한다 (기본 100)
//! start = "rest"            # 시작 자세: 로봇 설정의 이름 붙은 자세나 { j1 = -55.0, j2 = 52.0 }처럼 모든 관절의 각 [°]
//! start_tolerance_deg = 12  # 토크를 켠 자리가 시작 자세에서 이만큼 넘게 다르면 움직이지 않는다 (기본 5)
//!
//! [[steps]]
//! pause = "위팔 끝에 추를 걸었으면"   # Enter를 기다리며 버틴다
//! [[steps]]
//! move = { j2 = 90.0 }      # 관절 각 [°]. 적지 않은 관절은 그 자리. "home"처럼 이름 붙은 자세도 된다
//! speed_percent = 25        # 이 움직이기만의 빠르기 (move 단계에만. 적지 않으면 순서의 speed_percent)
//! [[steps]]
//! grip = 0.0                # 그리퍼를 이 열림 폭 [mm]으로. 가다 막혀 서면 잡은 것이다 (그리퍼를 같이 켠다)
//! [[steps]]
//! hold_s = 600              # 버틴다. 온도가 알림 기준에 닿으면 일찍 끝낸다
//! [[steps]]
//! move = { j2 = 0.0 }
//! hot_return = true         # 뜨거울 때 돌아갈 곳 (move 단계 하나에만): 아래 설명
//! [[steps]]
//! pause = "팔을 받쳤으면 (다음에 토크를 끈다)"
//! [[steps]]
//! release = true            # 모든 관절의 토크를 끈다 (마지막 단계만)
//! ```
//!
//! 움직이거나 버티는 동안 Enter를 누르면 감속해 서고 순서를 끝낸다. 고장이 나면 순서를 끝낸다 (서보는 마지막
//! 목표를 잡고 있다). 움직이기는 목표에 닿아야 끝난다: 서보가 뜨거워 쉬느라 목표 전에 서면 순서를 끝낸다.
//! 목표는 읽을 때 실행 루프와 같은 검사로 거른다 (소프트 한계와 같은 각은 받는다).
//!
//! 움직이기는 관절 공간의 곧은 길이다([`move_joints`]). 궤적은 [`Planner`]가 계산한다: 실행 명령에서는 따로 된
//! 계산 스레드가 계산하고, 실행 루프는 결과가 올 때까지(한두 박자) 버틴다. 시작 자세를 적으면 토크를 켠 자리가 그
//! 자세인지 먼저 본다: 순서가 지나는 길은 그 자세에서 시작한 길이라, 다른 자리에서 시작하면 미리 따져 본 길과
//! 달라진다 (0034).
//!
//! 순서는 실행 루프가 온도와 전압을 판단할 수 있을 때(판단 창이 찬 뒤, 토크를 켜고 약 1초) 시작한다
//! ([`RunLoop::settled`]). 그동안 팔은 토크를 켠 자리에서 버틴다.
//!
//! 실행 명령은 움직이기 전에 순서의 모든 움직이기를 부딪힘 검사로 거른다 ([`Script::check_collisions`], 0041):
//! 지금 팔의 자세(읽은 관절 각)에서 시작해 단계마다 곧은 길을 검사하고, 막히면 어느 단계 어느 짝인지 알리고 시작하지
//! 않는다. 쉬는 자세는 쉬는 자세 위(rest_above)를 거쳐 떠나고 앉는다 (곧게 가면 아래팔이 선반을 긁는다).
//!
//! 뜨거울 때 돌아갈 곳(`hot_return`, 0034): 그 단계 앞에서 서보 온도가 알림 기준에 닿으면, 하던 움직이기는 끝까지
//! 가고(버티기는 바로 끝낸다) 남은 단계를 건너뛰어 그 단계로 간다. 그다음은 순서대로다 (예: 짐을 사람이 받아 빼고
//! 쉬는 자세로). 첫 움직이기를 마친 뒤에만 돌아가고(시작 자세에서 곧게 가는 길은 쉬는 자세 위를 거치지 않는다),
//! 그 뒤 그 앞의 단계 어디서든 그 자리로 곧게 가는 길도 움직이기 전에 부딪힘 검사로 거른다. 이 표시가 있는 순서는
//! 서보가 이미 알림 기준에 닿아 있으면 시작하지 않는다. 적지 않으면 버티기만 일찍 끝내고 다음 단계로 간다 (쉬는
//! 자세로 가는 순서처럼 뜨거워도 돌아야 하는 순서). 실행 루프는 쉬는 기준에서 따로 선다 (마지막 보호).

use std::collections::BTreeMap;

use horibot_servo::feetech::ServoBus;
use serde::Deserialize;

use crate::check::Console;
use crate::collision::Model;
use crate::joints::Servo;
#[cfg(doc)]
use crate::plan::move_joints;
use crate::plan::{MotionLimits, Planner};
use crate::runloop::{Grip, Mode, RunLoop};
use crate::settings::{JOINT_NAMES, Joint, LoopSettings, Settings};

/// 시작 자세에서 받는 차이의 기본값 [°]
const START_TOLERANCE_DEG: f64 = 5.0;
/// 그리퍼가 이 시간 안에 서지 않으면 순서를 끝낸다 [s] (다 벌린 폭 60 mm를 20 mm/s로 3초)
const GRIP_TIMEOUT: f64 = 10.0;

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct ScriptFile {
    name: String,
    joints: Vec<String>,
    speed_percent: Option<f64>,
    start: Option<PoseFile>,
    start_tolerance_deg: Option<f64>,
    #[serde(default)]
    steps: Vec<StepFile>,
}

/// 자세: 로봇 설정의 이름 붙은 자세, 또는 관절마다의 각 [°].
#[derive(Debug, Deserialize)]
#[serde(untagged)]
enum PoseFile {
    Name(String),
    Angles(BTreeMap<String, f64>),
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct StepFile {
    pause: Option<String>,
    #[serde(rename = "move")]
    move_to: Option<PoseFile>,
    speed_percent: Option<f64>,
    hold_s: Option<f64>,
    grip: Option<f64>,
    release: Option<bool>,
    hot_return: Option<bool>,
}

/// 단계 하나.
#[derive(Debug, Clone, PartialEq)]
pub enum Step {
    /// 안내를 보이고 Enter를 기다린다 (그동안 버틴다)
    Pause(String),
    /// 관절 이동: (순서의 관절 번호, 목표 [rad])와 실행 루프의 이동 한계에 곱하는 몫 (0~1. 단계에 적지 않으면 순서의
    /// 빠르기)
    Move {
        targets: Vec<(usize, f64)>,
        speed: f64,
    },
    /// 버틴다 [s]
    Hold(f64),
    /// 그리퍼를 열림 폭 [mm]으로 움직이고 설 때까지 기다린다
    Grip(f64),
    /// 모든 관절의 토크를 끈다
    Release,
}

/// 시작 자세: 토크를 켠 자리가 이 자세에서 받는 차이 안이어야 움직인다.
#[derive(Debug, Clone, PartialEq)]
pub struct Start {
    /// 관절마다 (순서의 관절 순서) 각 [rad]
    pub angles: Vec<f64>,
    /// 받는 차이 [rad]
    pub tolerance: f64,
}

/// 읽고 검사한 시험 순서.
#[derive(Debug, Clone, PartialEq)]
pub struct Script {
    pub name: String,
    /// 켤 관절 (관절 번호 순, 실행 루프의 관절 순서와 같다)
    pub joints: Vec<&'static str>,
    /// 실행 루프의 이동 한계에 곱하는 몫 (0~1): 빠르기를 적지 않은 움직이기의 빠르기
    pub speed: f64,
    pub start: Option<Start>,
    pub steps: Vec<Step>,
    /// 뜨거울 때 돌아갈 단계 (움직이기 단계의 번호, 0부터)
    pub hot_return: Option<usize>,
}

/// 자세를 순서의 관절마다 (관절 번호, 각 [rad])로 푼다. 이름 붙은 자세는 순서의 모든 관절을 정한다. 소프트
/// 한계 밖이면 문제로 모은다.
fn resolve(
    pose: &PoseFile,
    joints: &[&Joint],
    settings: &Settings,
    at: &str,
    problems: &mut Vec<String>,
) -> Vec<(usize, f64)> {
    let named: Vec<(&str, f64)> = match pose {
        PoseFile::Name(name) => match settings.pose(name) {
            None => {
                problems.push(format!("{at}: 로봇 설정에 없는 자세다: {name}"));
                return Vec::new();
            }
            Some(angles) => joints
                .iter()
                .filter_map(|joint| {
                    let index = JOINT_NAMES.iter().position(|known| *known == joint.name)?;
                    Some((joint.name, angles[index].to_degrees()))
                })
                .collect(),
        },
        PoseFile::Angles(map) => {
            if map.is_empty() {
                problems.push(format!("{at}: 관절이 없다"));
            }
            map.iter()
                .map(|(name, &degrees)| (name.as_str(), degrees))
                .collect()
        }
    };
    let mut found = Vec::new();
    for (name, degrees) in named {
        match joints.iter().position(|joint| joint.name == name) {
            None => problems.push(format!("{at}: {name}은 joints에 없다")),
            Some(index) => {
                let joint = joints[index];
                let angle = degrees.to_radians();
                let reaches = Servo::from_joint(joint).is_ok_and(|servo| servo.reaches(angle));
                if reaches {
                    found.push((index, angle));
                } else {
                    problems.push(format!(
                        "{at}: {name} {degrees}°는 소프트 한계({:.1}°~{:.1}°) 밖이다",
                        joint.lower.to_degrees(),
                        joint.upper.to_degrees()
                    ));
                }
            }
        }
    }
    found
}

/// 막힌 길을 사람이 읽을 글로: 어느 길의 몇 % 자리, 그 자세, 가까운 짝.
fn blocked_text(model: &Model, what: &str, blocked: &crate::collision::Blocked) -> String {
    let angles: Vec<String> = JOINT_NAMES
        .iter()
        .zip(blocked.q)
        .map(|(name, a)| format!("{name} {:.1}°", a.to_degrees()))
        .collect();
    format!(
        "부딪힘 검사: {what}에서 막힌다. 길의 {:.0}% 자리({})에서 {} (여유 {} mm)",
        blocked.fraction * 100.0,
        angles.join(", "),
        model.describe(&blocked.nearest),
        model.margin()
    )
}

impl Script {
    /// 그리퍼 단계가 있다 (실행 루프에 그리퍼 서보를 같이 켠다).
    pub fn uses_gripper(&self) -> bool {
        self.steps.iter().any(|step| matches!(step, Step::Grip(_)))
    }

    /// 시험 순서를 읽고 로봇 설정과 맞춰 본다. 관절은 영점이 있어야 하고, 목표는 소프트 한계 안이어야 하고(실행
    /// 루프와 같은 검사: [`Servo::reaches`]), 토크 끄기는 마지막 단계여야 한다. 잘못된 곳은 모두 모아 알린다.
    pub fn parse(text: &str, settings: &Settings) -> Result<Self, String> {
        let file: ScriptFile =
            toml::from_str(text).map_err(|e| format!("시험 순서를 읽지 못했다: {e}"))?;
        let mut problems = Vec::new();
        let mut joints: Vec<&crate::settings::Joint> = Vec::new();
        for name in &file.joints {
            match settings.joint(name) {
                None => problems.push(format!("로봇 설정에 없는 관절이다: {name}")),
                Some(joint) if joint.calibration.is_none() => problems.push(format!(
                    "{name}: 영점을 재지 않아 움직이지 않는다 (zero로 잰다)"
                )),
                Some(joint) if joints.iter().any(|known| known.name == joint.name) => {
                    problems.push(format!("관절을 두 번 적었다: {name}"))
                }
                Some(joint) => joints.push(joint),
            }
        }
        if file.joints.is_empty() {
            problems.push("joints에 켤 관절이 없다".to_owned());
        }
        joints.sort_by_key(|joint| joint.id);
        let speed = file.speed_percent.unwrap_or(100.0);
        if !(speed.is_finite() && speed > 0.0 && speed <= 100.0) {
            problems.push(format!("speed_percent는 0보다 크고 100 이하다: {speed}"));
        }
        let tolerance = file.start_tolerance_deg.unwrap_or(START_TOLERANCE_DEG);
        if file.start_tolerance_deg.is_some() && file.start.is_none() {
            problems.push("start_tolerance_deg는 start가 있을 때만 적는다".to_owned());
        }
        if !(tolerance.is_finite() && tolerance > 0.0 && tolerance <= 30.0) {
            problems.push(format!(
                "start_tolerance_deg는 0보다 크고 30 이하다: {tolerance}"
            ));
        }
        let start = file.start.as_ref().map(|pose| {
            let found = resolve(pose, &joints, settings, "start", &mut problems);
            let mut angles = vec![f64::NAN; joints.len()];
            for (index, angle) in found {
                angles[index] = angle;
            }
            if let Some(missing) = angles.iter().position(|angle| angle.is_nan())
                && !joints.is_empty()
            {
                problems.push(format!(
                    "start: 켜는 관절의 각을 모두 적는다 ({}이 없다)",
                    joints[missing].name
                ));
            }
            Start {
                angles,
                tolerance: tolerance.to_radians(),
            }
        });
        let mut steps = Vec::new();
        let mut hot_return = None;
        let last = file.steps.len().saturating_sub(1);
        for (n, step) in file.steps.iter().enumerate() {
            let at = format!("{}번째 단계", n + 1);
            let kinds = usize::from(step.pause.is_some())
                + usize::from(step.move_to.is_some())
                + usize::from(step.hold_s.is_some())
                + usize::from(step.grip.is_some())
                + usize::from(step.release.is_some());
            if kinds != 1 {
                problems.push(format!(
                    "{at}: pause, move, hold_s, grip, release 가운데 하나만 적는다"
                ));
                continue;
            }
            if let Some(flag) = step.hot_return {
                if step.move_to.is_none() {
                    problems.push(format!("{at}: hot_return은 move 단계에만 적는다"));
                } else if !flag {
                    problems.push(format!("{at}: hot_return은 true만 적는다"));
                } else if hot_return.replace(n).is_some() {
                    problems.push(format!("{at}: hot_return은 한 단계에만 적는다"));
                }
            }
            if let Some(percent) = step.speed_percent {
                if step.move_to.is_none() {
                    problems.push(format!("{at}: speed_percent는 move 단계에만 적는다"));
                } else if !(percent.is_finite() && percent > 0.0 && percent <= 100.0) {
                    problems.push(format!(
                        "{at}: speed_percent는 0보다 크고 100 이하다: {percent}"
                    ));
                }
            }
            if let Some(text) = &step.pause {
                steps.push(Step::Pause(text.clone()));
            } else if let Some(pose) = &step.move_to {
                let targets = resolve(pose, &joints, settings, &at, &mut problems);
                let percent = step.speed_percent.unwrap_or(speed);
                steps.push(Step::Move {
                    targets,
                    speed: percent / 100.0,
                });
            } else if let Some(seconds) = step.hold_s {
                if !(seconds.is_finite() && seconds > 0.0) {
                    problems.push(format!("{at}: hold_s는 0보다 크다: {seconds}"));
                }
                steps.push(Step::Hold(seconds));
            } else if let Some(width) = step.grip {
                match &settings.gripper {
                    None => problems.push(format!("{at}: 로봇 설정에 그리퍼가 없다")),
                    Some(gripper) if gripper.joint.calibration.is_none() => problems.push(format!(
                        "{at}: 그리퍼의 닫은 자리를 재지 않아 움직이지 않는다 (zero로 잰다)"
                    )),
                    Some(gripper)
                        if !(width.is_finite() && (0.0..=gripper.max_width).contains(&width)) =>
                    {
                        problems.push(format!(
                            "{at}: grip은 열림 폭 0~{} mm다: {width}",
                            gripper.max_width
                        ))
                    }
                    Some(_) => {}
                }
                steps.push(Step::Grip(width));
            } else if step.release == Some(true) {
                if n != last {
                    problems.push(format!(
                        "{at}: release는 마지막 단계여야 한다 (토크를 끈 뒤에는 움직이지 않는다)"
                    ));
                }
                steps.push(Step::Release);
            } else {
                problems.push(format!("{at}: release는 true만 적는다"));
            }
        }
        if !problems.is_empty() {
            return Err(problems.join("\n"));
        }
        Ok(Self {
            name: file.name,
            joints: joints.iter().map(|joint| joint.name).collect(),
            speed: speed / 100.0,
            start,
            steps,
            hot_return,
        })
    }

    /// 순서의 움직이기를 부딪힘 검사로 거른다 (0041). `start`는 지금 팔의 관절 각 [rad] (J1 ~ J6)이다. 단계마다
    /// 순서의 관절만 목표로 바뀌고 나머지 관절은 그 자리다. 뜨거울 때 돌아갈 곳(`hot_return`)이 있으면, 그 앞의
    /// 단계마다 그 단계를 시작하는 자리에서 돌아갈 곳으로 곧게 가는 길도 본다. 통과하면 사람이 볼 요약을, 막히면
    /// 어느 단계 어느 짝인지 알리는 글을 낸다.
    pub fn check_collisions(&self, model: &Model, start: &[f64; 6]) -> Result<String, String> {
        let mut q = *start;
        if let Some(found) = model.first_within(&q, model.margin()) {
            return Err(format!(
                "부딪힘 검사: 지금 자세가 이미 여유({} mm) 안이다: {}",
                model.margin(),
                model.describe(&found)
            ));
        }
        let mut moves = 0;
        let mut samples = 0;
        let mut closest: Option<(String, crate::collision::Nearest)> = None;
        // 뜨거울 때 돌아가는 길의 출발점: (단계 번호, 그 단계를 시작하는 자리)
        // 돌아가기는 첫 움직이기를 마친 뒤에만 한다 (쉬는 자세에서 곧게 가면 아래팔이 선반을 긁는다, 0037)
        let mut departures: Vec<(usize, [f64; 6])> = Vec::new();
        let mut left = false;
        for (n, step) in self.steps.iter().enumerate() {
            if left
                && self.hot_return.is_some_and(|target| n < target)
                && departures.last().is_none_or(|(_, last)| *last != q)
            {
                departures.push((n, q));
            }
            let Step::Move { targets, .. } = step else {
                continue;
            };
            left = true;
            let next = self.moved(&q, targets)?;
            let what = format!("{}번째 단계(움직이기)", n + 1);
            let report = model
                .check_move(&q, &next)
                .map_err(|blocked| blocked_text(model, &what, &blocked))?;
            moves += 1;
            samples += report.samples;
            if closest
                .as_ref()
                .is_none_or(|(_, c)| report.nearest.distance < c.distance)
            {
                closest = Some((format!("{}번째 단계", n + 1), report.nearest));
            }
            q = next;
        }
        let mut returns = 0;
        if let Some(target) = self.hot_return
            && let Some(Step::Move { targets, .. }) = self.steps.get(target)
        {
            for (n, from) in departures {
                let to = self.moved(&from, targets)?;
                let what = format!(
                    "{}번째 단계에서 뜨거울 때 돌아갈 곳({}번째 단계)으로 가는 길",
                    n + 1,
                    target + 1
                );
                let report = model
                    .check_move(&from, &to)
                    .map_err(|blocked| blocked_text(model, &what, &blocked))?;
                returns += 1;
                samples += report.samples;
                if closest
                    .as_ref()
                    .is_none_or(|(_, c)| report.nearest.distance < c.distance)
                {
                    closest = Some((what, report.nearest));
                }
            }
        }
        let near = match closest {
            None => String::new(),
            Some((where_, c)) => format!(", 가장 가까운 것은 {where_}의 {}", model.describe(&c)),
        };
        let back = if self.hot_return.is_some() {
            format!(", 뜨거울 때 돌아가는 길 {returns}개")
        } else {
            String::new()
        };
        Ok(format!(
            "부딪힘 검사: 움직이기 {moves}개를 지난다{back} (자세 {samples}개 검사, 여유 {} mm{near})",
            model.margin()
        ))
    }

    /// 자세 `q`(J1 ~ J6)에서 움직이기의 목표를 적용한 자세. 순서에 없는 관절은 그 자리다.
    fn moved(&self, q: &[f64; 6], targets: &[(usize, f64)]) -> Result<[f64; 6], String> {
        let mut next = *q;
        for &(index, angle) in targets {
            let name = self.joints[index];
            let k = JOINT_NAMES
                .iter()
                .position(|known| *known == name)
                .ok_or_else(|| format!("{name}은 팔의 관절이 아니다"))?;
            next[k] = angle;
        }
        Ok(next)
    }

    /// 버티기를 `max_s`초까지로 줄이고 줄인 버티기 수를 낸다 (실행 명령의 --hold-max: 시뮬레이션은 온도를
    /// 흉내 내지 않아 오래 버텨도 새로 볼 것이 없다).
    pub fn cap_holds(&mut self, max_s: f64) -> usize {
        let mut shortened = 0;
        for step in &mut self.steps {
            if let Step::Hold(seconds) = step
                && *seconds > max_s
            {
                *seconds = max_s;
                shortened += 1;
            }
        }
        shortened
    }

    /// 사람이 볼 순서: 단계마다 한 줄.
    pub fn describe(&self) -> String {
        let mut lines = vec![format!(
            "시험 순서 \"{}\": 관절 {}, 속도 {:.0}%",
            self.name,
            self.joints.join(", "),
            self.speed * 100.0
        )];
        if let Some(start) = &self.start {
            let parts: Vec<String> = self
                .joints
                .iter()
                .zip(&start.angles)
                .map(|(name, angle)| format!("{name} {:.1}°", angle.to_degrees()))
                .collect();
            lines.push(format!(
                "  시작 자세: {} (±{:.0}°)",
                parts.join(", "),
                start.tolerance.to_degrees()
            ));
        }
        for (n, step) in self.steps.iter().enumerate() {
            let text = match step {
                Step::Pause(text) => format!("멈춤: {text} (누르면 다음)"),
                Step::Move { targets, speed } => {
                    let parts: Vec<String> = targets
                        .iter()
                        .map(|&(index, angle)| {
                            format!("{} {:.1}°", self.joints[index], angle.to_degrees())
                        })
                        .collect();
                    let pace = if (speed - self.speed).abs() > 1e-9 {
                        format!(" (속도 {:.0}%)", speed * 100.0)
                    } else {
                        String::new()
                    };
                    let back = if self.hot_return == Some(n) {
                        " (뜨거우면 여기로 돌아온다)"
                    } else {
                        ""
                    };
                    format!("움직이기: {}{pace}{back}", parts.join(", "))
                }
                Step::Hold(seconds) => {
                    format!("버티기: {seconds} s (온도가 알림 기준에 닿으면 일찍 끝낸다)")
                }
                Step::Grip(width) => {
                    format!("그리퍼: 열림 폭 {width} mm로 (가다 막혀 서면 잡은 것)")
                }
                Step::Release => "토크 끄기: 모든 관절".to_owned(),
            };
            lines.push(format!("  {}. {text}", n + 1));
        }
        lines.join("\n") + "\n"
    }
}

/// 순서의 진행.
#[derive(Debug, Clone, PartialEq)]
pub enum Status {
    Running,
    /// 모든 단계를 마쳤다
    Done,
    /// 사람이 멈췄거나 고장이 나 순서를 끝냈다
    Stopped(String),
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum Phase {
    Begin,
    Waiting,
    /// 움직이기의 궤적을 계산하는 중 (팔은 버틴다)
    Planning,
    Moving,
    Holding {
        until: f64,
    },
    Gripping {
        until: f64,
    },
}

/// 시험 순서를 실행 루프에 건다. 박자마다 [`RunLoop::tick`] 뒤에 [`Runner::tick`]을 부른다.
pub struct Runner {
    script: Script,
    planner: Planner,
    temperature_warn: f64,
    index: usize,
    phase: Phase,
    /// 지금 움직이기 단계의 목표 [rad] (실행 루프의 관절 순서)
    goal: Vec<f64>,
    events: Vec<String>,
    released: bool,
    /// 시작 자세를 봤다 (첫 박자에 한 번)
    checked_start: bool,
    /// 뜨거워 돌아갈 곳으로 건너뛰었다 (한 번만)
    returned: bool,
    /// 첫 움직이기를 마쳤다: 그 전에는 돌아가지 않는다 (시작 자세에서 곧게 가는 길은 검사하지 않았다)
    left_start: bool,
}

impl Runner {
    /// 궤적을 맡긴 자리에서 계산한다 (시험과 시뮬레이션: 박자 수가 늘 같다). `period`는 실행 루프의 박자 길이 [s]다.
    pub fn new(script: Script, settings: &LoopSettings, period: f64) -> Self {
        let motion = MotionLimits::from_settings(settings, script.joints.len());
        Self::with_planner(script, settings, Planner::inline(motion, period))
    }

    /// 궤적을 따로 된 계산 스레드에서 계산한다 (실행 명령). 실행 루프 스레드가 아닌 곳에서 부른다
    /// ([`Planner::spawn`]).
    pub fn with_planner_thread(
        script: Script,
        settings: &LoopSettings,
        period: f64,
    ) -> Result<Self, String> {
        let motion = MotionLimits::from_settings(settings, script.joints.len());
        Ok(Self::with_planner(
            script,
            settings,
            Planner::spawn(motion, period)?,
        ))
    }

    fn with_planner(script: Script, settings: &LoopSettings, planner: Planner) -> Self {
        Self {
            script,
            planner,
            temperature_warn: settings.temperature_warn,
            index: 0,
            phase: Phase::Begin,
            goal: Vec::new(),
            events: Vec::new(),
            released: false,
            checked_start: false,
            returned: false,
            left_start: false,
        }
    }

    /// 서보 온도가 알림 기준에 닿았고 뜨거울 때 돌아갈 곳이 아직 앞에 있으면 그 단계로 건너뛴다. 건너뛰었으면
    /// 참이다 (한 번만 건너뛴다).
    fn return_if_hot<B: ServoBus>(&mut self, run: &RunLoop<B>, console: &mut dyn Console) -> bool {
        let Some(target) = self.script.hot_return else {
            return false;
        };
        if self.returned || !self.left_start || self.index >= target {
            return false;
        }
        let hottest = run.hottest().map_or(0, |(_, temperature)| temperature);
        if f64::from(hottest) < self.temperature_warn {
            return false;
        }
        let text = format!(
            "[{}] 온도 {hottest}°C가 알림 기준 {}°C에 닿아, 남은 단계를 건너뛰고 {}번째 단계로 돌아간다",
            self.label(),
            self.temperature_warn,
            target + 1
        );
        self.note(console, text);
        self.returned = true;
        self.index = target;
        self.phase = Phase::Begin;
        true
    }

    /// 순서가 모든 관절의 토크를 껐다 (끈 뒤에는 박자를 돌리지 않아 읽은 상태로는 알 수 없다).
    pub fn released(&self) -> bool {
        self.released
    }

    /// 돌리는 순서.
    pub fn script(&self) -> &Script {
        &self.script
    }

    /// 지금 단계 (기록의 step 칸): `3-hold`처럼 번호와 종류.
    pub fn label(&self) -> String {
        match self.script.steps.get(self.index) {
            None => "end".to_owned(),
            Some(step) => {
                let kind = match step {
                    Step::Pause(_) => "pause",
                    Step::Move { .. } => "move",
                    Step::Hold(_) => "hold",
                    Step::Grip(_) => "grip",
                    Step::Release => "release",
                };
                format!("{}-{kind}", self.index + 1)
            }
        }
    }

    /// 지난 부름 뒤에 생긴 알림 (기록의 events.csv에 남긴다).
    pub fn take_events(&mut self) -> Vec<String> {
        std::mem::take(&mut self.events)
    }

    fn note(&mut self, console: &mut dyn Console, text: String) {
        console.say(&text);
        self.events.push(text);
    }

    fn next(&mut self) {
        self.index += 1;
        self.phase = Phase::Begin;
    }

    /// 박자 하나만큼 순서를 진행한다. `now`는 시작부터의 시간 [s]이다.
    pub fn tick<B: ServoBus>(
        &mut self,
        run: &mut RunLoop<B>,
        console: &mut dyn Console,
        now: f64,
    ) -> Status {
        if let Mode::Fault(reason) = run.mode() {
            let text = format!("고장으로 순서를 끝낸다: {reason}");
            self.note(console, text.clone());
            return Status::Stopped(text);
        }
        // 게임패드의 B나 끊긴 게임패드: 어느 단계에서든 선다. 움직이던 관절과 그리퍼는 감속해 그 자리에 서고, 멈춤과
        // 버티기에서는 팔이 이미 서 있다
        if let Some(why) = console.stop_requested() {
            run.stop();
            let text = format!("[{}] {why}", self.label());
            self.note(console, text.clone());
            return Status::Stopped(text);
        }
        if !self.checked_start {
            // 실행 루프가 온도와 전압을 판단할 수 있을 때까지(토크를 켜고 약 1초) 버티며 기다린다: 몇 박자만 읽은
            // 값으로 시작할지 정하면 튀어 읽힌 값 하나에 흔들린다
            if !run.settled() {
                return Status::Running;
            }
            self.checked_start = true;
            if let Some(start) = self.script.start.clone() {
                // 실행 루프는 토크를 켠 자리(읽은 위치)를 목표로 붙잡고 시작한다
                let off: Vec<String> = run
                    .targets()
                    .iter()
                    .zip(&start.angles)
                    .zip(&self.script.joints)
                    .filter(|((now, want), _)| (*now - *want).abs() > start.tolerance)
                    .map(|((now, want), name)| {
                        format!(
                            "{name} {:.1}° (시작 자세 {:.1}°)",
                            now.to_degrees(),
                            want.to_degrees()
                        )
                    })
                    .collect();
                if !off.is_empty() {
                    let text = format!(
                        "팔이 시작 자세(±{:.0}°)에 있지 않아 움직이지 않는다: {}. 순서는 시작 자세에서 \
                         시작하는 길로 짰다",
                        start.tolerance.to_degrees(),
                        off.join(", ")
                    );
                    self.note(console, text.clone());
                    return Status::Stopped(text);
                }
                self.note(console, "시작 자세를 확인했다".to_owned());
            }
            // 뜨거울 때 돌아갈 곳이 있는 순서(짐을 드는 시험)는 식은 서보로 시작한다. 쉬는 자세로 가는 순서는 이
            // 검사가 없어 뜨거워도 돈다
            let hottest = run.hottest();
            if self.script.hot_return.is_some()
                && let Some((id, temperature)) = hottest
                && f64::from(temperature) >= self.temperature_warn
            {
                let text = format!(
                    "서보 {id}가 이미 {temperature}°C로 알림 기준 {}°C에 닿아 있어 시작하지 않는다. 식힌 뒤 다시 \
                     돌린다",
                    self.temperature_warn
                );
                self.note(console, text.clone());
                return Status::Stopped(text);
            }
        }
        loop {
            // 단계를 시작할 때마다 본다: 하던 움직이기는 끝까지 가고 나서 돌아간다
            if self.phase == Phase::Begin && self.return_if_hot(run, console) {
                continue;
            }
            let Some(step) = self.script.steps.get(self.index).cloned() else {
                return Status::Done;
            };
            match (self.phase, step) {
                (Phase::Begin, Step::Pause(text)) => {
                    // 누를 키(Enter, 게임패드의 A)는 콘솔이 붙인다
                    console.begin_wait(&format!("[{}] {text}", self.label()));
                    self.phase = Phase::Waiting;
                    return Status::Running;
                }
                (Phase::Waiting, Step::Pause(_)) => {
                    if !console.enter_pressed() {
                        return Status::Running;
                    }
                    self.next();
                }
                (Phase::Begin, Step::Move { targets, speed }) => {
                    let mut goal = run.targets().to_vec();
                    for &(index, angle) in &targets {
                        goal[index] = angle;
                    }
                    if let Err(e) = self.planner.request(run.targets(), &goal, speed) {
                        let text = format!("[{}] 움직이지 못했다: {e}", self.label());
                        self.note(console, text.clone());
                        return Status::Stopped(text);
                    }
                    self.goal = goal;
                    self.phase = Phase::Planning;
                }
                (Phase::Planning, Step::Move { .. }) => {
                    if console.enter_pressed() {
                        let text = format!("[{}] 사람이 멈췄다", self.label());
                        self.note(console, text.clone());
                        return Status::Stopped(text);
                    }
                    // 계산하는 동안 실행 루프는 지금 목표를 붙잡고 있어, 궤적의 시작이 지금 목표 그대로다
                    let Some(planned) = self.planner.poll() else {
                        return Status::Running;
                    };
                    let submitted = planned.and_then(|trajectory| {
                        let seconds = trajectory.points.last().map_or(0.0, |point| point.0);
                        run.submit(trajectory).map(|()| seconds)
                    });
                    match submitted {
                        Ok(seconds) => {
                            let text = format!("[{}] 움직인다 ({seconds:.1} s)", self.label());
                            self.note(console, text);
                            self.phase = Phase::Moving;
                            return Status::Running;
                        }
                        Err(e) => {
                            let text = format!("[{}] 움직이지 못했다: {e}", self.label());
                            self.note(console, text.clone());
                            return Status::Stopped(text);
                        }
                    }
                }
                (Phase::Moving, Step::Move { .. }) => {
                    if console.enter_pressed() {
                        run.stop();
                        let text = format!("[{}] 사람이 멈췄다", self.label());
                        self.note(console, text.clone());
                        return Status::Stopped(text);
                    }
                    if *run.mode() != Mode::Holding {
                        return Status::Running;
                    }
                    // 목표에 닿아야 끝난다. 서보가 뜨거워 쉬느라 목표 전에 섰으면 끊긴 움직임을 마친 것으로 치지 않고
                    // 순서를 끝낸다 (팔은 선 자리에서 버틴다)
                    let reached = run
                        .targets()
                        .iter()
                        .zip(&self.goal)
                        .all(|(now, goal)| (now - goal).abs() <= 1e-9);
                    if !reached {
                        let why = run
                            .resting()
                            .map_or_else(String::new, |reason| format!(": {reason}"));
                        let text = format!(
                            "[{}] 목표에 닿기 전에 서서 순서를 끝낸다{why}",
                            self.label()
                        );
                        self.note(console, text.clone());
                        return Status::Stopped(text);
                    }
                    self.left_start = true;
                    self.next();
                }
                (Phase::Begin, Step::Hold(seconds)) => {
                    let text = format!("[{}] {seconds} s 버틴다", self.label());
                    self.note(console, text);
                    self.phase = Phase::Holding {
                        until: now + seconds,
                    };
                    return Status::Running;
                }
                (Phase::Holding { until }, Step::Hold(_)) => {
                    if console.enter_pressed() {
                        let text = format!("[{}] 사람이 멈췄다", self.label());
                        self.note(console, text.clone());
                        return Status::Stopped(text);
                    }
                    if self.return_if_hot(run, console) {
                        continue;
                    }
                    let hottest = run.hottest().map_or(0, |(_, temperature)| temperature);
                    if f64::from(hottest) >= self.temperature_warn {
                        let text = format!(
                            "[{}] 온도 {hottest}°C가 알림 기준 {}°C에 닿아 버티기를 끝낸다",
                            self.label(),
                            self.temperature_warn
                        );
                        self.note(console, text);
                        self.next();
                        continue;
                    }
                    if now < until {
                        return Status::Running;
                    }
                    self.next();
                }
                (Phase::Begin, Step::Grip(width)) => match run.grip(width) {
                    Ok(()) => {
                        let text = format!("[{}] 그리퍼를 열림 폭 {width} mm로", self.label());
                        self.note(console, text);
                        self.phase = Phase::Gripping {
                            until: now + GRIP_TIMEOUT,
                        };
                        return Status::Running;
                    }
                    Err(e) => {
                        let text = format!("[{}] 그리퍼를 움직이지 못했다: {e}", self.label());
                        self.note(console, text.clone());
                        return Status::Stopped(text);
                    }
                },
                (Phase::Gripping { until }, Step::Grip(_)) => {
                    if console.enter_pressed() {
                        run.stop();
                        let text = format!("[{}] 사람이 멈췄다", self.label());
                        self.note(console, text.clone());
                        return Status::Stopped(text);
                    }
                    let text = match run.gripper() {
                        Some((_, Grip::Reached(width))) => {
                            format!("[{}] 닿았다 (열림 폭 {width:.1} mm)", self.label())
                        }
                        Some((_, Grip::Grasped(width))) => {
                            format!("[{}] 잡았다 (열림 폭 {width:.1} mm)", self.label())
                        }
                        _ if now < until => return Status::Running,
                        _ => {
                            let text = format!(
                                "[{}] 그리퍼가 {GRIP_TIMEOUT} s 안에 서지 않아 순서를 끝낸다",
                                self.label()
                            );
                            self.note(console, text.clone());
                            return Status::Stopped(text);
                        }
                    };
                    self.note(console, text);
                    self.next();
                }
                (Phase::Begin, Step::Release) => {
                    return match run.release_all() {
                        Ok(()) => {
                            let text = format!("[{}] 모든 관절의 토크를 껐다", self.label());
                            self.note(console, text);
                            self.index += 1;
                            self.released = true;
                            Status::Done
                        }
                        Err(e) => {
                            let text = format!("[{}] 토크를 끄지 못했다: {e}", self.label());
                            self.note(console, text.clone());
                            Status::Stopped(text)
                        }
                    };
                }
                // 단계마다 시작할 때만 Begin이고, 진행 상태는 그 단계의 종류와 짝이 맞는다
                (_, _) => {
                    self.phase = Phase::Begin;
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use horibot_servo::feetech::Bus;
    use horibot_servo::feetech::fake::{FakePort, FakeServo, open_fake_bus};
    use horibot_servo::feetech::registers as reg;

    use super::*;
    use crate::joints::{Caps, Joints, Servo, programmed};
    use crate::settings::{TEST_GRIPPER, TEST_J3};

    /// 시험용 로봇 설정: TEST_J3에 영점을 준다.
    fn settings() -> Settings {
        Settings::parse(&calibrated_j3()).unwrap()
    }

    fn calibrated_j3() -> String {
        TEST_J3.replace(
            "[joints.j3]",
            "[joints.j3]\nzero_raw = 2048\ndirection = 1\nposition_offset = 0",
        )
    }

    /// 그리퍼와 이름 붙은 자세가 있는 시험용 로봇 설정 (rest의 j3는 20°)
    fn full_settings() -> Settings {
        let text = format!(
            "{}{TEST_GRIPPER}\n[poses]\nrest_deg = [0, 0, 20, 180, 0, 0]\n",
            calibrated_j3()
        );
        Settings::parse(&text).unwrap()
    }

    /// j3와 그리퍼 서보 (가짜 서보는 둘 다 서보 값 2048: j3 0°, 그리퍼는 닫힘)
    fn run_loop_with_gripper(settings: &Settings) -> RunLoop<Bus<FakePort>> {
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

    /// 누를 줄을 미리 정해 두는 콘솔: 박자마다 Enter를 눌렀는지를 순서대로 준다. `stop_at`번째 박자에는 서라는
    /// 요청(게임패드의 B)을 낸다.
    #[derive(Default)]
    struct Keys {
        presses: Vec<bool>,
        said: Vec<String>,
        stop_at: Option<usize>,
        ticks: usize,
    }

    impl Console for Keys {
        fn say(&mut self, text: &str) {
            self.said.push(text.to_owned());
        }
        fn ask(&mut self, _question: &str) -> String {
            String::new()
        }
        fn begin_wait(&mut self, prompt: &str) {
            self.said.push(prompt.to_owned());
        }
        fn enter_pressed(&mut self) -> bool {
            if self.presses.is_empty() {
                false
            } else {
                self.presses.remove(0)
            }
        }
        fn status(&mut self, _text: &str) {}
        fn stop_requested(&mut self) -> Option<String> {
            self.ticks += 1;
            (Some(self.ticks) == self.stop_at).then(|| "사람이 멈췄다 (게임패드 B)".to_owned())
        }
    }

    fn run_loop(settings: &Settings) -> RunLoop<Bus<FakePort>> {
        run_loop_reading(settings, |_| {})
    }

    /// `run_loop`처럼 세우되, 세우기 전에 가짜 서보를 바꾼다 (세울 때 처음 읽는 값을 정한다).
    fn run_loop_reading(
        settings: &Settings,
        prepare: impl FnOnce(&mut FakeServo),
    ) -> RunLoop<Bus<FakePort>> {
        let joint = settings.joint("j3").unwrap();
        let servo = Servo::from_joint(joint).unwrap();
        let mut fake = programmed(FakeServo::with(3, joint.model_number, 2048), &servo);
        prepare(&mut fake);
        let joints = Joints::open(
            open_fake_bus([fake]),
            vec![servo],
            Caps::for_loop(&settings.run_loop),
        )
        .unwrap();
        RunLoop::start(joints, settings.run_loop, None).unwrap()
    }

    const DT: f64 = 0.01;

    /// 박자를 돌리며 순서를 끝까지 진행한다. `hook`는 박자마다 가짜 서보를 바꿀 자리다.
    fn drive(
        run: &mut RunLoop<Bus<FakePort>>,
        runner: &mut Runner,
        keys: &mut Keys,
        limit: usize,
        mut hook: impl FnMut(&mut RunLoop<Bus<FakePort>>, f64),
    ) -> Status {
        for n in 0..limit {
            let now = n as f64 * DT;
            hook(run, now);
            run.tick(DT);
            let status = runner.tick(run, keys, now);
            if status != Status::Running {
                return status;
            }
        }
        Status::Running
    }

    #[test]
    fn a_script_is_checked_against_the_robot_settings() {
        let settings = settings();
        let good = "name = \"t\"\njoints = [\"j3\"]\n[[steps]]\nmove = { j3 = 30.0 }\n[[steps]]\nrelease = true\n";
        let script = Script::parse(good, &settings).unwrap();
        assert_eq!(script.joints, ["j3"]);
        assert_eq!(script.steps.len(), 2);
        assert!(script.describe().contains("j3 30.0°"));
        let bad = "name = \"t\"\njoints = [\"j3\", \"j9\"]\nspeed_percent = 150\n\
                   [[steps]]\nmove = { j3 = 170.0 }\n[[steps]]\nrelease = true\n[[steps]]\nhold_s = 1\n\
                   [[steps]]\npause = \"a\"\nhold_s = 2\n";
        let error = Script::parse(bad, &settings).unwrap_err();
        for expected in [
            "j9",
            "speed_percent",
            "소프트 한계",
            "release는 마지막",
            "하나만",
        ] {
            assert!(error.contains(expected), "{expected}: {error}");
        }
        let uncalibrated = Settings::parse(TEST_J3).unwrap();
        let error = Script::parse(good, &uncalibrated).unwrap_err();
        assert!(error.contains("영점"), "{error}");
        assert!(Script::parse("name = \"t\"\njoints = [\"j3\"]\nwhat = 1\n", &settings).is_err());
    }

    #[test]
    fn a_script_moves_holds_and_releases() {
        let settings = settings();
        let text = "name = \"t\"\njoints = [\"j3\"]\n\
                    [[steps]]\npause = \"추를 걸었으면\"\n\
                    [[steps]]\nmove = { j3 = 20.0 }\n\
                    [[steps]]\nhold_s = 1.0\n\
                    [[steps]]\nmove = { j3 = 0.0 }\n\
                    [[steps]]\nrelease = true\n";
        let script = Script::parse(text, &settings).unwrap();
        let mut run = run_loop(&settings);
        let mut runner = Runner::new(script, &settings.run_loop, DT);
        // 멈춤에서 다섯 박자 뒤에 Enter를 누른다
        let mut keys = Keys {
            presses: vec![false, false, false, false, true],
            ..Default::default()
        };
        let mut peak = 0.0_f64;
        let status = drive(&mut run, &mut runner, &mut keys, 5000, |run, _| {
            peak = peak.max(run.targets()[0]);
        });
        assert_eq!(status, Status::Done);
        assert!((peak - 20f64.to_radians()).abs() < 1e-9, "{peak}");
        assert!(run.targets()[0].abs() < 1e-9);
        // 토크를 껐다
        let torque = run.joints_for_test().wire().servos[0].get(reg::TORQUE_ENABLE);
        assert_eq!(torque, reg::TORQUE_OFF);
        assert!(runner.released());
        assert!(
            keys.said.iter().any(|line| line.contains("토크를 껐다")),
            "{:?}",
            keys.said
        );
    }

    #[test]
    fn long_holds_are_capped_for_the_simulation() {
        let settings = settings();
        let text = "name = \"t\"\njoints = [\"j3\"]\n\
                    [[steps]]\nhold_s = 600\n\
                    [[steps]]\nmove = { j3 = 20.0 }\n\
                    [[steps]]\nhold_s = 2\n";
        let mut script = Script::parse(text, &settings).unwrap();
        assert_eq!(script.cap_holds(5.0), 1);
        let holds: Vec<f64> = script
            .steps
            .iter()
            .filter_map(|step| match step {
                Step::Hold(seconds) => Some(*seconds),
                _ => None,
            })
            .collect();
        assert_eq!(holds, [5.0, 2.0]);
    }

    #[test]
    fn enter_stops_a_move_and_ends_the_script() {
        let settings = settings();
        let text = "name = \"t\"\njoints = [\"j3\"]\n[[steps]]\nmove = { j3 = 60.0 }\n[[steps]]\nhold_s = 5\n";
        let script = Script::parse(text, &settings).unwrap();
        let mut run = run_loop(&settings);
        let mut runner = Runner::new(script, &settings.run_loop, DT);
        let mut presses = vec![false; 50];
        presses.push(true);
        let mut keys = Keys {
            presses,
            ..Default::default()
        };
        let status = drive(&mut run, &mut runner, &mut keys, 5000, |_, _| {});
        assert!(
            matches!(&status, Status::Stopped(text) if text.contains("사람이 멈췄다")),
            "{status:?}"
        );
        // 감속해 선다: 정지 중이고, 목표 60°에 닿기 전이다
        assert_eq!(run.mode(), &Mode::Stopping);
        for _ in 0..500 {
            run.tick(DT);
        }
        assert_eq!(run.mode(), &Mode::Holding);
        assert!(
            run.targets()[0] < 60f64.to_radians() - 0.1,
            "{}",
            run.targets()[0]
        );
    }

    /// 게임패드의 B는 멈춤에서 기다리는 중에도 순서를 끝낸다 (Enter와 A는 멈춤에서 "다음"이다). 팔은 움직이지 않고
    /// 그 자리에서 버틴다.
    #[test]
    fn a_stop_request_ends_the_script_at_a_pause() {
        let settings = settings();
        let text = "name = \"t\"\njoints = [\"j3\"]\n\
                    [[steps]]\npause = \"선을 봤으면\"\n\
                    [[steps]]\nmove = { j3 = 20.0 }\n\
                    [[steps]]\nrelease = true\n";
        let script = Script::parse(text, &settings).unwrap();
        let mut run = run_loop(&settings);
        let mut runner = Runner::new(script, &settings.run_loop, DT);
        // 판단 창이 차는 100박자 뒤에 순서가 시작한다
        let mut keys = Keys {
            stop_at: Some(130),
            ..Default::default()
        };
        let status = drive(&mut run, &mut runner, &mut keys, 5000, |_, _| {});
        assert!(
            matches!(&status, Status::Stopped(text) if text == "[1-pause] 사람이 멈췄다 (게임패드 B)"),
            "{status:?}"
        );
        assert_eq!(
            keys.said[0], "[1-pause] 선을 봤으면",
            "누를 키는 콘솔이 붙인다"
        );
        assert_eq!(run.mode(), &Mode::Holding);
        assert!(run.targets()[0].abs() < 1e-9, "움직이지 않았다");
        assert!(!runner.released(), "토크를 끄지 않았다");
    }

    /// 움직이는 동안의 B는 Enter처럼 감속해 서게 한다.
    #[test]
    fn a_stop_request_stops_a_move() {
        let settings = settings();
        let text = "name = \"t\"\njoints = [\"j3\"]\n[[steps]]\nmove = { j3 = 60.0 }\n[[steps]]\nhold_s = 5\n";
        let script = Script::parse(text, &settings).unwrap();
        let mut run = run_loop(&settings);
        let mut runner = Runner::new(script, &settings.run_loop, DT);
        let mut keys = Keys {
            stop_at: Some(150),
            ..Default::default()
        };
        let status = drive(&mut run, &mut runner, &mut keys, 5000, |_, _| {});
        assert!(
            matches!(&status, Status::Stopped(text) if text.contains("게임패드")),
            "{status:?}"
        );
        assert_eq!(run.mode(), &Mode::Stopping);
        for _ in 0..500 {
            run.tick(DT);
        }
        assert_eq!(run.mode(), &Mode::Holding);
        assert!(run.targets()[0] < 60f64.to_radians() - 0.1);
    }

    #[test]
    fn moves_to_exactly_the_soft_limits_run() {
        // 소프트 한계 -30°~118°는 1706.7칸과 3390.6칸이다. 한계를 안쪽으로 반올림하던 때는 118°(반올림하면
        // 3391칸)로 가는 움직임을 순서는 받고 실행 루프는 움직이기 직전에 거부했다 (2026-11-18 첫 실물 실행)
        let settings = settings();
        let text = "name = \"t\"\njoints = [\"j3\"]\n\
                    [[steps]]\nmove = { j3 = 118.0 }\n\
                    [[steps]]\nmove = { j3 = -30.0 }\n";
        let script = Script::parse(text, &settings).unwrap();
        let mut run = run_loop(&settings);
        let mut runner = Runner::new(script, &settings.run_loop, DT);
        let mut keys = Keys::default();
        let mut peak = 0.0_f64;
        let status = drive(&mut run, &mut runner, &mut keys, 5000, |run, _| {
            peak = peak.max(run.targets()[0]);
        });
        assert_eq!(status, Status::Done, "{:?}", keys.said);
        assert!((peak - 118f64.to_radians()).abs() < 1e-9, "{peak}");
        assert!((run.targets()[0] - (-30f64).to_radians()).abs() < 1e-9);
        // 한계를 넘는 목표는 읽을 때 거른다
        let error = Script::parse(&text.replace("118.0", "118.1"), &settings).unwrap_err();
        assert!(error.contains("소프트 한계"), "{error}");
    }

    #[test]
    fn a_move_cut_short_by_a_hot_servo_ends_the_script() {
        let settings = settings();
        let text = "name = \"t\"\njoints = [\"j3\"]\n\
                    [[steps]]\nmove = { j3 = 60.0 }\n\
                    [[steps]]\nmove = { j3 = 0.0 }\n";
        let script = Script::parse(text, &settings).unwrap();
        let mut run = run_loop(&settings);
        let mut runner = Runner::new(script, &settings.run_loop, DT);
        let mut keys = Keys::default();
        let hot = settings.run_loop.temperature_rest as i32 + 1;
        // 움직이기는 판단 창이 찬 1초 뒤에 시작한다
        let status = drive(&mut run, &mut runner, &mut keys, 5000, |run, now| {
            if now > 1.5 {
                run.joints_for_test().wire().servos[0].set(reg::PRESENT_TEMPERATURE, hot);
            }
        });
        assert!(
            matches!(&status, Status::Stopped(text)
                if text.contains("목표에 닿기 전에") && text.contains("쉬는 기준")),
            "{status:?}"
        );
        // 선 자리에서 버티고, 다음 단계(0°로 움직이기)는 하지 않았다
        assert_eq!(run.mode(), &Mode::Holding);
        let stopped = run.targets()[0];
        assert!(
            stopped > 0.1 && stopped < 60f64.to_radians() - 0.1,
            "{stopped}"
        );
    }

    #[test]
    fn holding_ends_early_when_the_servo_gets_warm() {
        let settings = settings();
        let text = "name = \"t\"\njoints = [\"j3\"]\n[[steps]]\nhold_s = 600\n[[steps]]\nmove = { j3 = 10.0 }\n";
        let script = Script::parse(text, &settings).unwrap();
        let mut run = run_loop(&settings);
        let mut runner = Runner::new(script, &settings.run_loop, DT);
        let mut keys = Keys::default();
        let warm = settings.run_loop.temperature_warn as i32;
        let status = drive(&mut run, &mut runner, &mut keys, 20_000, |run, now| {
            if now > 2.0 {
                run.joints_for_test().wire().servos[0].set(reg::PRESENT_TEMPERATURE, warm);
            }
        });
        assert_eq!(status, Status::Done);
        assert!(
            keys.said
                .iter()
                .any(|line| line.contains("알림 기준") && line.contains("버티기를 끝낸다"))
        );
        // 버티기를 끝내고 다음 단계(움직이기)를 했다
        assert!((run.targets()[0] - 10f64.to_radians()).abs() < 1e-9);
    }

    #[test]
    fn a_warm_servo_skips_to_the_hot_return_step() {
        let settings = settings();
        // 20°에서 버티다 뜨거워지면 40°로 가지 않고, 돌아갈 곳(5°)으로 간 뒤 남은 순서를 한다
        let text = "name = \"t\"\njoints = [\"j3\"]\n\
                    [[steps]]\nmove = { j3 = 20.0 }\n\
                    [[steps]]\nhold_s = 600\n\
                    [[steps]]\nmove = { j3 = 40.0 }\n\
                    [[steps]]\nhold_s = 600\n\
                    [[steps]]\nmove = { j3 = 5.0 }\nhot_return = true\n\
                    [[steps]]\nhold_s = 1\n";
        let script = Script::parse(text, &settings).unwrap();
        assert_eq!(script.hot_return, Some(4));
        assert!(script.describe().contains("뜨거우면 여기로"));
        let mut run = run_loop(&settings);
        let mut runner = Runner::new(script, &settings.run_loop, DT);
        let mut keys = Keys::default();
        let warm = settings.run_loop.temperature_warn as i32;
        let mut peak = 0.0_f64;
        let status = drive(&mut run, &mut runner, &mut keys, 20_000, |run, now| {
            if now > 3.0 {
                run.joints_for_test().wire().servos[0].set(reg::PRESENT_TEMPERATURE, warm);
            }
            peak = peak.max(run.targets()[0]);
        });
        assert_eq!(status, Status::Done);
        assert!(
            keys.said
                .iter()
                .any(|line| line.contains("5번째 단계로 돌아간다")),
            "{:?}",
            keys.said
        );
        assert!(peak < 20f64.to_radians() + 1e-9, "{}", peak.to_degrees());
        assert!((run.targets()[0] - 5f64.to_radians()).abs() < 1e-9);
    }

    #[test]
    fn a_script_with_a_hot_return_does_not_start_hot() {
        let settings = settings();
        let text = "name = \"t\"\njoints = [\"j3\"]\n\
                    [[steps]]\nmove = { j3 = 20.0 }\n\
                    [[steps]]\nhold_s = 600\n\
                    [[steps]]\nmove = { j3 = 5.0 }\nhot_return = true\n";
        let script = Script::parse(text, &settings).unwrap();
        // 실행 루프를 세울 때부터 뜨겁다 (판단 창이 찬 뒤에 보고 시작하지 않는다)
        let warm = settings.run_loop.temperature_warn as i32;
        let mut run = run_loop_reading(&settings, |fake| fake.set(reg::PRESENT_TEMPERATURE, warm));
        let mut runner = Runner::new(script, &settings.run_loop, DT);
        let mut keys = Keys::default();
        let status = drive(&mut run, &mut runner, &mut keys, 2000, |_, _| {});
        assert!(
            matches!(&status, Status::Stopped(text) if text.contains("시작하지 않는다")),
            "{status:?}"
        );
        assert!(run.targets()[0].abs() < 1e-9, "움직이지 않았다");
    }

    /// 토크를 켜며 처음 읽은 온도가 튀어도 순서는 시작한다 (2026-11-28 처음 전원: 처음 읽은 J6이 45°C로 튀어
    /// 어깨 버티기가 시작하지 않았다). 실행 루프가 온도를 판단할 수 있을 때(판단 창이 찬 뒤)까지 움직이지 않는다.
    #[test]
    fn a_jumped_first_reading_does_not_keep_a_script_from_starting() {
        let settings = settings();
        let text = "name = \"t\"\njoints = [\"j3\"]\n\
                    [[steps]]\nmove = { j3 = 20.0 }\n\
                    [[steps]]\nhold_s = 1\n\
                    [[steps]]\nmove = { j3 = 5.0 }\nhot_return = true\n";
        let script = Script::parse(text, &settings).unwrap();
        let jumped = settings.run_loop.temperature_rest as i32 + 20;
        let mut run =
            run_loop_reading(&settings, |fake| fake.set(reg::PRESENT_TEMPERATURE, jumped));
        run.joints_for_test().wire().servos[0].set(reg::PRESENT_TEMPERATURE, 30);
        let mut runner = Runner::new(script, &settings.run_loop, DT);
        let mut keys = Keys::default();
        let mut moved_unsettled = false;
        let status = drive(&mut run, &mut runner, &mut keys, 5000, |run, _| {
            moved_unsettled |= !run.settled() && run.targets()[0].abs() > 1e-9;
        });
        assert_eq!(status, Status::Done, "{:?}", keys.said);
        assert!(!moved_unsettled, "판단 창이 차기 전에 움직였다");
        assert!((run.targets()[0] - 5f64.to_radians()).abs() < 1e-9);
    }

    /// 계산 스레드로 궤적을 계산해도 순서가 같게 돈다: 결과가 올 때까지 팔은 버티고, 온 뒤에 움직인다.
    #[test]
    fn a_script_runs_with_the_planner_thread() {
        let settings = settings();
        let text = "name = \"t\"\njoints = [\"j3\"]\n\
                    [[steps]]\nmove = { j3 = 20.0 }\n\
                    [[steps]]\nmove = { j3 = -10.0 }\n";
        let script = Script::parse(text, &settings).unwrap();
        let mut run = run_loop(&settings);
        let mut runner = Runner::with_planner_thread(script, &settings.run_loop, DT).unwrap();
        let mut keys = Keys::default();
        let mut peak = 0.0_f64;
        // 가짜 박자는 실제 시간보다 훨씬 빨리 돌아, 계산을 기다리는 박자 수가 그때마다 다르다
        let status = drive(&mut run, &mut runner, &mut keys, 1_000_000, |run, _| {
            peak = peak.max(run.targets()[0]);
            std::thread::yield_now();
        });
        assert_eq!(status, Status::Done, "{:?}", keys.said);
        assert!((peak - 20f64.to_radians()).abs() < 1e-9, "{peak}");
        assert!((run.targets()[0] - (-10f64).to_radians()).abs() < 1e-9);
        let moves = keys
            .said
            .iter()
            .filter(|line| line.contains("움직인다"))
            .count();
        assert_eq!(moves, 2, "{:?}", keys.said);
    }

    /// 움직이기마다 빠르기를 줄 수 있다 (0042의 떨림 재기: 한 순서에서 7.5°/s와 15°/s로 오간다). 적지 않은
    /// 움직이기는 순서의 빠르기다.
    #[test]
    fn a_move_step_can_have_its_own_speed() {
        let settings = settings();
        let text = "name = \"t\"\njoints = [\"j3\"]\nspeed_percent = 50\n\
                    [[steps]]\nmove = { j3 = 20.0 }\n\
                    [[steps]]\nmove = { j3 = 0.0 }\nspeed_percent = 25\n";
        let script = Script::parse(text, &settings).unwrap();
        let speeds: Vec<f64> = script
            .steps
            .iter()
            .filter_map(|step| match step {
                Step::Move { speed, .. } => Some(*speed),
                _ => None,
            })
            .collect();
        assert_eq!(speeds, [0.5, 0.25]);
        let description = script.describe();
        assert!(
            description.contains("j3 0.0° (속도 25%)") && !description.contains("(속도 50%)"),
            "{description}"
        );
        let mut run = run_loop(&settings);
        let mut runner = Runner::new(script, &settings.run_loop, DT);
        let mut keys = Keys::default();
        let mut fastest = [0.0_f64; 2];
        let mut last = 0.0;
        let status = drive(&mut run, &mut runner, &mut keys, 5000, |run, _| {
            let now = run.targets()[0];
            let speed = (now - last).abs() / DT;
            fastest[usize::from(now < last)] = fastest[usize::from(now < last)].max(speed);
            last = now;
        });
        assert_eq!(status, Status::Done, "{:?}", keys.said);
        // 오를 때는 한계(시험값 30°/s)의 50%, 내릴 때는 25%
        let limit = settings.run_loop.max_speed;
        assert!((fastest[0] - limit * 0.5).abs() < 1e-6, "{fastest:?}");
        assert!((fastest[1] - limit * 0.25).abs() < 1e-6, "{fastest:?}");
        // move 단계에만, 0보다 크고 100 이하
        let parse = |steps: &str| {
            Script::parse(
                &format!("name = \"t\"\njoints = [\"j3\"]\n{steps}"),
                &settings,
            )
        };
        let error = parse("[[steps]]\nhold_s = 1\nspeed_percent = 50\n").unwrap_err();
        assert!(error.contains("move 단계에만"), "{error}");
        for wrong in ["0", "150"] {
            let error = parse(&format!(
                "[[steps]]\nmove = {{ j3 = 1.0 }}\nspeed_percent = {wrong}\n"
            ))
            .unwrap_err();
            assert!(error.contains("0보다 크고 100 이하"), "{error}");
        }
    }

    #[test]
    fn hot_return_marks_one_move_step() {
        let settings = settings();
        let parse = |steps: &str| {
            Script::parse(
                &format!("name = \"t\"\njoints = [\"j3\"]\n{steps}"),
                &settings,
            )
        };
        let error = parse("[[steps]]\nhold_s = 1\nhot_return = true\n").unwrap_err();
        assert!(error.contains("move 단계에만"), "{error}");
        let error = parse(
            "[[steps]]\nmove = { j3 = 1.0 }\nhot_return = true\n\
             [[steps]]\nmove = { j3 = 2.0 }\nhot_return = true\n",
        )
        .unwrap_err();
        assert!(error.contains("한 단계에만"), "{error}");
        let error = parse("[[steps]]\nmove = { j3 = 1.0 }\nhot_return = false\n").unwrap_err();
        assert!(error.contains("true만"), "{error}");
    }

    #[test]
    fn named_poses_and_the_start_pose_are_read() {
        let settings = full_settings();
        let text = "name = \"t\"\njoints = [\"j3\"]\nstart = \"rest\"\nstart_tolerance_deg = 8\n\
                    [[steps]]\nmove = { j3 = 40.0 }\n[[steps]]\nmove = \"rest\"\n[[steps]]\ngrip = 30\n";
        let script = Script::parse(text, &settings).unwrap();
        let start = script.start.as_ref().unwrap();
        assert!((start.angles[0] - 20f64.to_radians()).abs() < 1e-12);
        assert!((start.tolerance - 8f64.to_radians()).abs() < 1e-12);
        assert_eq!(
            script.steps[1],
            Step::Move {
                targets: vec![(0, 20f64.to_radians())],
                speed: 1.0
            }
        );
        assert_eq!(script.steps[2], Step::Grip(30.0));
        assert!(script.uses_gripper());
        let description = script.describe();
        assert!(
            description.contains("시작 자세: j3 20.0° (±8°)"),
            "{description}"
        );
        assert!(
            description.contains("그리퍼: 열림 폭 30 mm"),
            "{description}"
        );
        // 없는 자세, 표의 빠진 관절, 그리퍼 범위, start 없는 받는 차이
        let bad = "name = \"t\"\njoints = [\"j3\"]\nstart = \"nowhere\"\n\
                   [[steps]]\nmove = \"home\"\n[[steps]]\ngrip = 70\n";
        let error = Script::parse(bad, &settings).unwrap_err();
        for expected in [
            "start: 로봇 설정에 없는 자세다: nowhere",
            "1번째 단계: 로봇 설정에 없는 자세다: home",
            "grip은 열림 폭 0~60 mm다: 70",
        ] {
            assert!(error.contains(expected), "{expected}: {error}");
        }
        let error = Script::parse(
            "name = \"t\"\njoints = [\"j3\"]\nstart_tolerance_deg = 3\n",
            &settings,
        )
        .unwrap_err();
        assert!(error.contains("start가 있을 때만"), "{error}");
        // 그리퍼가 없는 로봇 설정
        let without = Settings::parse(&calibrated_j3()).unwrap();
        let error = Script::parse(
            "name = \"t\"\njoints = [\"j3\"]\n[[steps]]\ngrip = 10\n",
            &without,
        )
        .unwrap_err();
        assert!(error.contains("로봇 설정에 그리퍼가 없다"), "{error}");
    }

    #[test]
    fn a_script_does_not_move_away_from_another_pose_than_its_start() {
        let settings = full_settings();
        // 가짜 서보는 0°에 있다. 시작 자세 rest의 j3는 20°라 받는 차이 5°를 넘는다
        let text =
            "name = \"t\"\njoints = [\"j3\"]\nstart = \"rest\"\n[[steps]]\nmove = { j3 = 40.0 }\n";
        let script = Script::parse(text, &settings).unwrap();
        let mut run = run_loop(&settings);
        let mut runner = Runner::new(script, &settings.run_loop, DT);
        let mut keys = Keys::default();
        let status = drive(&mut run, &mut runner, &mut keys, 300, |_, _| {});
        assert!(
            matches!(&status, Status::Stopped(text) if text.contains("시작 자세") && text.contains("j3 0.0°")),
            "{status:?}"
        );
        assert!(run.targets()[0].abs() < 1e-9, "움직이지 않았다");
        // 받는 차이 안이면 움직인다
        let text = "name = \"t\"\njoints = [\"j3\"]\nstart = { j3 = 3.0 }\n[[steps]]\nmove = { j3 = 10.0 }\n";
        let script = Script::parse(text, &settings).unwrap();
        let mut run = run_loop(&settings);
        let mut runner = Runner::new(script, &settings.run_loop, DT);
        let mut keys = Keys::default();
        let status = drive(&mut run, &mut runner, &mut keys, 5000, |_, _| {});
        assert_eq!(status, Status::Done, "{:?}", keys.said);
        assert!(
            keys.said
                .iter()
                .any(|line| line.contains("시작 자세를 확인했다"))
        );
    }

    #[test]
    fn grip_steps_wait_until_the_gripper_reaches_or_grasps() {
        let settings = full_settings();
        let text = "name = \"t\"\njoints = [\"j3\"]\n[[steps]]\ngrip = 30\n[[steps]]\ngrip = 0\n\
                    [[steps]]\nmove = { j3 = 10.0 }\n";
        let script = Script::parse(text, &settings).unwrap();
        let mut run = run_loop_with_gripper(&settings);
        let mut runner = Runner::new(script, &settings.run_loop, DT);
        let mut keys = Keys::default();
        // 30 mm까지 벌린 뒤(20 mm/s로 1.5 s) 닫는 길에 0.5 s쯤 가서 손가락이 짐에 닿는다
        let mut closing_since = None;
        let mut last_width = 0.0;
        let status = drive(&mut run, &mut runner, &mut keys, 2000, |run, now| {
            if let Some((width, _)) = run.gripper() {
                if width < last_width - 1e-6 && closing_since.is_none() {
                    closing_since = Some(now);
                }
                last_width = width;
            }
            if closing_since.is_some_and(|since| now > since + 0.5) {
                run.joints_for_test().wire().servos[1].held = true;
            }
        });
        assert_eq!(status, Status::Done, "{:?}", keys.said);
        let reached = keys
            .said
            .iter()
            .find(|line| line.contains("닿았다"))
            .unwrap();
        assert!(reached.contains("30."), "{reached}");
        let grasped = keys
            .said
            .iter()
            .find(|line| line.contains("잡았다"))
            .unwrap();
        assert!(
            grasped.contains("열림 폭 1") || grasped.contains("열림 폭 2"),
            "{grasped}"
        );
        // 잡은 뒤 팔이 움직였다
        assert!((run.targets()[0] - 10f64.to_radians()).abs() < 1e-9);
    }

    #[test]
    fn a_fault_ends_the_script() {
        let settings = settings();
        let text = "name = \"t\"\njoints = [\"j3\"]\n[[steps]]\nhold_s = 600\n";
        let script = Script::parse(text, &settings).unwrap();
        let mut run = run_loop(&settings);
        let mut runner = Runner::new(script, &settings.run_loop, DT);
        let mut keys = Keys::default();
        let status = drive(&mut run, &mut runner, &mut keys, 1000, |run, now| {
            if now > 0.5 {
                run.joints_for_test().wire().servos[0].overload();
            }
        });
        assert!(
            matches!(&status, Status::Stopped(text) if text.contains("고장")),
            "{status:?}"
        );
    }
}
