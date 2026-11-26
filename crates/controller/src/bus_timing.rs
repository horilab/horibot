//! 서보 버스의 통신 시간 재기 (docs/decisions/0024-servo-bus.md의 만드는 순서 6, 0009의 실물 전 시험 3): 팔의
//! 서보(그리퍼 포함)를 한 버스에 다 이은 채 짧은 읽기, 상태 읽기, 동기 읽기, 한 박자(동기 쓰기 + 동기 읽기)를
//! 실행 루프처럼 주기마다 되풀이해 걸린 시간을 잰다. 첫 연결 때 Python으로 잰 명령(`horibot_servo timing`)과
//! 같은 동작이고 기록도 같은 꼴이라(`samples.csv`, `meta.json`의 결과 칸) 두 값을 견줄 수 있다. 처음 전원 날
//! 컨트롤러 파이에서 이 값으로 실행 루프의 주기를 정한다.
//!
//! - 서보는 움직이지 않는다: 토크가 켜진 서보가 있으면 재지 않고, 한 박자의 동기 쓰기는 목표 대신 토크 한도에
//!   읽은 값을 그대로 쓴다.
//! - 응답 지연(7번)은 서보에 든 값 그대로 재고, 0이 아닌 서보가 있으면 잠근 채 0으로 바꿔 한 번 더 잰다. 바꾼
//!   값은 되돌리지 않는다 (서보 전원을 껐다 켜면 돌아간다).
//! - 짧은 읽기는 핑 대신이다 (모델 번호 2바이트).
//! - 시간은 부르는 쪽이 준다 ([`Clock`]): 실물은 단조 시계와 정한 시각까지 자기, 시험은 가짜 시계.

use std::collections::BTreeMap;
use std::fmt::Write as _;

use horibot_servo::feetech::registers::{self as reg, Register};
use horibot_servo::feetech::{Error, ServoBus, SyncRead};
use serde_json::{Value, json};

use crate::rt;

/// 7번(응답 지연) 한 칸의 시간 [s]. LeRobot과 rustypot의 해석이고, 두 단계의 차이로 실물에서 확인한다
pub const RESPONSE_DELAY_STEP_S: f64 = 2e-6;
/// 한 박자의 동기 쓰기. 실행 루프는 목표를 쓰지만 여기서는 움직이지 않는 칸에 읽은 값을 그대로 쓴다
pub const TICK_WRITE: Register = reg::TORQUE_LIMIT;
/// 동작마다 되풀이하는 기본 횟수
pub const DEFAULT_COUNT: usize = 300;

/// 재는 동작.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Op {
    ShortRead,
    StateRead,
    SyncRead,
    Tick,
}

/// 재는 차례.
pub const OPS: [Op; 4] = [Op::ShortRead, Op::StateRead, Op::SyncRead, Op::Tick];

impl Op {
    /// 기록의 이름 (Python 기록과 같다).
    pub fn name(self) -> &'static str {
        match self {
            Self::ShortRead => "short_read",
            Self::StateRead => "state_read",
            Self::SyncRead => "sync_read",
            Self::Tick => "tick",
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            Self::ShortRead => "짧은 읽기",
            Self::StateRead => "상태 읽기",
            Self::SyncRead => "동기 읽기",
            Self::Tick => "한 박자",
        }
    }

    /// 서보 하나와의 거래인가 (아니면 서보 전체).
    fn single(self) -> bool {
        matches!(self, Self::ShortRead | Self::StateRead)
    }
}

/// 응답 지연의 단계.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Phase {
    /// 서보에 든 응답 지연 그대로
    AsIs,
    /// 응답 지연 0 (잠근 채 썼다)
    DelayZero,
}

impl Phase {
    pub fn name(self) -> &'static str {
        match self {
            Self::AsIs => "as_is",
            Self::DelayZero => "delay_0",
        }
    }

    pub fn label(self) -> &'static str {
        match self {
            Self::AsIs => "응답 지연 그대로",
            Self::DelayZero => "응답 지연 0",
        }
    }
}

/// 한 번 잰 것.
#[derive(Debug, Clone, PartialEq)]
pub struct Sample {
    pub phase: Phase,
    pub op: Op,
    /// 짧은 읽기와 상태 읽기는 그 서보, 동기는 없다 (전체)
    pub servo_id: Option<u8>,
    /// 잰 것 전체의 시작부터 [s]
    pub t_s: f64,
    pub duration_s: f64,
    /// 비었으면 성공. 동기는 실패한 서보마다 "ID:오류"
    pub error: String,
}

/// 단계와 동작마다의 요약.
#[derive(Debug, Clone, PartialEq)]
pub struct Summary {
    pub phase: Phase,
    pub op: Op,
    pub count: usize,
    pub failures: usize,
    /// 성공한 것 가운데 주기를 넘긴 수
    pub over_period: usize,
    pub min_ms: Option<f64>,
    pub median_ms: Option<f64>,
    pub p99_ms: Option<f64>,
    pub max_ms: Option<f64>,
    /// 선로 위 시간만의 계산 ([`wire_time_s`])
    pub wire_ms: f64,
}

/// 잰 결과.
#[derive(Debug, Clone, PartialEq)]
pub struct TimingRun {
    pub ids: Vec<u8>,
    pub models: BTreeMap<u8, u16>,
    /// 서보마다 처음 응답 지연 (7번 값)
    pub delays_before: BTreeMap<u8, i32>,
    pub count: usize,
    pub period_s: f64,
    pub baud: u32,
    pub samples: Vec<Sample>,
    pub summaries: Vec<Summary>,
    /// 응답 지연을 0으로 바꾸지 못했으면 그 까닭
    pub trial_error: Option<String>,
}

/// 재는 시계.
pub trait Clock {
    /// 시작부터의 시간 [s]
    fn now(&self) -> f64;
    /// 이 시각 [s]까지 잔다. 지났으면 바로 돌아온다
    fn sleep_until(&mut self, t: f64);
}

/// 단조 시계와 정한 시각까지 자기 (실행 루프와 같다).
pub struct RtClock {
    start_ns: u64,
}

impl Default for RtClock {
    fn default() -> Self {
        Self {
            start_ns: rt::now_ns(),
        }
    }
}

impl Clock for RtClock {
    fn now(&self) -> f64 {
        rt::now_ns().saturating_sub(self.start_ns) as f64 / 1e9
    }

    fn sleep_until(&mut self, t: f64) {
        rt::sleep_until_ns(self.start_ns + (t.max(0.0) * 1e9) as u64);
    }
}

fn state_span() -> (u8, usize) {
    let first = reg::STATE.iter().map(|r| r.address).min().unwrap_or(0);
    let end = reg::STATE
        .iter()
        .map(|r| usize::from(r.address) + usize::from(r.size))
        .max()
        .unwrap_or(0);
    (first, end - usize::from(first))
}

/// 선로 위 시간: 주고받는 바이트(한 바이트 10비트)와 서보의 응답 지연만 더한 값. 짧은 읽기와 상태 읽기는 서보
/// 하나(응답 지연은 평균), 동기 읽기와 한 박자는 서보 전체다. USB와 운영체제가 더하는 시간은 들어 있지 않아서,
/// 잰 값과의 차이가 그 몫이다.
pub fn wire_time_s(op: Op, delays: &[i32], baud: u32) -> f64 {
    let byte_s = 10.0 / f64::from(baud);
    let n = delays.len();
    let total_delay: i32 = delays.iter().sum();
    let one_delay_s = if n == 0 {
        0.0
    } else {
        f64::from(total_delay) / n as f64 * RESPONSE_DELAY_STEP_S
    };
    let all_delays_s = f64::from(total_delay) * RESPONSE_DELAY_STEP_S;
    let (_, state_size) = state_span();
    // 읽기 요청 FF FF ID LEN INST 주소 길이 CHK, 응답 FF FF ID LEN ERR 값… CHK
    let sync_read = (8 + n) + n * (6 + state_size);
    match op {
        Op::ShortRead => (8 + 6 + usize::from(reg::MODEL.size)) as f64 * byte_s + one_delay_s,
        Op::StateRead => (8 + 6 + state_size) as f64 * byte_s + one_delay_s,
        Op::SyncRead => sync_read as f64 * byte_s + all_delays_s,
        Op::Tick => {
            let sync_write = 8 + n * (1 + usize::from(TICK_WRITE.size));
            (sync_write + sync_read) as f64 * byte_s + all_delays_s
        }
    }
}

fn round_ms(seconds: f64) -> f64 {
    (seconds * 1e6).round() / 1e3
}

/// 가까운 순위 방식: 값의 q 비율이 이 값 이하다.
fn percentile(ordered: &[f64], q: f64) -> f64 {
    let rank = (q * ordered.len() as f64).ceil() as usize;
    ordered[rank.saturating_sub(1).min(ordered.len() - 1)]
}

fn median(ordered: &[f64]) -> f64 {
    let n = ordered.len();
    if n % 2 == 1 {
        ordered[n / 2]
    } else {
        (ordered[n / 2 - 1] + ordered[n / 2]) / 2.0
    }
}

/// 단계와 동작 하나의 요약.
pub fn summarize(samples: &[Sample], phase: Phase, op: Op, period_s: f64, wire_s: f64) -> Summary {
    let mine: Vec<&Sample> = samples
        .iter()
        .filter(|s| s.phase == phase && s.op == op)
        .collect();
    let mut good: Vec<f64> = mine
        .iter()
        .filter(|s| s.error.is_empty())
        .map(|s| s.duration_s)
        .collect();
    good.sort_by(f64::total_cmp);
    let pick = |f: &dyn Fn(&[f64]) -> f64| (!good.is_empty()).then(|| round_ms(f(&good)));
    Summary {
        phase,
        op,
        count: mine.len(),
        failures: mine.len() - good.len(),
        over_period: good.iter().filter(|&&d| d > period_s).count(),
        min_ms: pick(&|g| g[0]),
        median_ms: pick(&median),
        p99_ms: pick(&|g| percentile(g, 0.99)),
        max_ms: pick(&|g| g[g.len() - 1]),
        wire_ms: round_ms(wire_s),
    }
}

/// 버스 오류의 이름 (Python 기록의 오류 이름과 같다).
fn error_name(error: &Error) -> &'static str {
    match error {
        Error::NoReply { .. } => "NoReply",
        Error::BadReply { .. } => "BadReply",
        Error::Port { .. } => "Port",
        Error::Verify { .. } => "Verify",
        Error::Refused(_) => "Refused",
        Error::Invalid(_) => "Invalid",
    }
}

fn sync_errors(results: &SyncRead) -> String {
    let mut text = String::new();
    for (id, result) in results {
        if let Err(error) = result {
            if !text.is_empty() {
                text.push(' ');
            }
            let _ = write!(text, "{id}:{}", error_name(error));
        }
    }
    text
}

fn attempt<B: ServoBus>(bus: &mut B, op: Op, id: u8, ids: &[u8], limits: &[(u8, i32)]) -> String {
    let result = match op {
        Op::ShortRead => bus.read(id, &[reg::MODEL]).map(|_| String::new()),
        Op::StateRead => bus.read(id, reg::STATE).map(|_| String::new()),
        Op::SyncRead => bus.sync_read(ids, reg::STATE).map(|r| sync_errors(&r)),
        Op::Tick => bus
            .sync_write(TICK_WRITE, limits)
            .and_then(|()| bus.sync_read(ids, reg::STATE))
            .map(|r| sync_errors(&r)),
    };
    result.unwrap_or_else(|error| error_name(&error).to_owned())
}

fn read_one<B: ServoBus>(bus: &mut B, id: u8, register: Register) -> Result<i32, String> {
    bus.read(id, &[register])
        .map_err(|e| e.to_string())?
        .get(register)
        .ok_or_else(|| format!("서보 {id}: {}를 읽지 못했다", register.name))
}

/// 서보마다 모델, 토크 스위치, 토크 한도, 응답 지연을 읽는다 (재기 전, 할 일을 보일 때도 쓴다).
pub fn read_servos<B: ServoBus>(bus: &mut B, ids: &[u8]) -> Result<Vec<ServoState>, String> {
    ids.iter()
        .map(|&id| {
            let model = read_one(bus, id, reg::MODEL)?;
            Ok(ServoState {
                id,
                model: u16::try_from(model).unwrap_or_default(),
                torque_enable: read_one(bus, id, reg::TORQUE_ENABLE)?,
                torque_limit: read_one(bus, id, TICK_WRITE)?,
                response_delay: read_one(bus, id, reg::RESPONSE_DELAY)?,
            })
        })
        .collect()
}

/// 재기 전에 읽은 서보 상태.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ServoState {
    pub id: u8,
    pub model: u16,
    pub torque_enable: i32,
    pub torque_limit: i32,
    pub response_delay: i32,
}

/// 통신 시간을 잰다. 토크가 켜진 서보가 있으면 재지 않는다.
pub fn measure<B: ServoBus>(
    bus: &mut B,
    ids: &[u8],
    count: usize,
    period_s: f64,
    baud: u32,
    clock: &mut impl Clock,
) -> Result<TimingRun, String> {
    if ids.is_empty() {
        return Err("잴 서보가 없다".to_owned());
    }
    // 주기가 수가 아니어도(NaN) 받지 않는다
    if count == 0 || !period_s.is_finite() || period_s <= 0.0 {
        return Err("횟수와 주기는 0보다 커야 한다".to_owned());
    }
    let servos = read_servos(bus, ids)?;
    if let Some(on) = servos.iter().find(|s| s.torque_enable != reg::TORQUE_OFF) {
        return Err(format!("서보 {}의 토크가 켜져 있어서 재지 않는다", on.id));
    }
    let limits: Vec<(u8, i32)> = servos.iter().map(|s| (s.id, s.torque_limit)).collect();
    let mut run = TimingRun {
        ids: ids.to_vec(),
        models: servos.iter().map(|s| (s.id, s.model)).collect(),
        delays_before: servos.iter().map(|s| (s.id, s.response_delay)).collect(),
        count,
        period_s,
        baud,
        samples: Vec::new(),
        summaries: Vec::new(),
        trial_error: None,
    };
    let start = clock.now();
    let before: Vec<i32> = run.delays_before.values().copied().collect();
    run.phase(bus, Phase::AsIs, &before, &limits, clock, start);
    if before.iter().any(|&d| d != 0) {
        for &id in ids {
            if let Err(error) = bus.write_trial(id, &[(reg::RESPONSE_DELAY, 0)]) {
                run.trial_error = Some(error.to_string());
                return Ok(run);
            }
        }
        run.phase(
            bus,
            Phase::DelayZero,
            &vec![0; ids.len()],
            &limits,
            clock,
            start,
        );
    }
    Ok(run)
}

impl TimingRun {
    fn phase<B: ServoBus>(
        &mut self,
        bus: &mut B,
        phase: Phase,
        delays: &[i32],
        limits: &[(u8, i32)],
        clock: &mut impl Clock,
        start: f64,
    ) {
        for op in OPS {
            // 동작 하나를 주기마다 되풀이한다. 늦어지면 기다리지 않고 바로 다음을 한다
            let mut due = clock.now();
            for i in 0..self.count {
                clock.sleep_until(due);
                let id = self.ids[i % self.ids.len()];
                let begin = clock.now();
                let error = attempt(bus, op, id, &self.ids, limits);
                let end = clock.now();
                self.samples.push(Sample {
                    phase,
                    op,
                    servo_id: op.single().then_some(id),
                    t_s: begin - start,
                    duration_s: end - begin,
                    error,
                });
                due = (due + self.period_s).max(end);
            }
            let wire = wire_time_s(op, delays, self.baud);
            self.summaries
                .push(summarize(&self.samples, phase, op, self.period_s, wire));
        }
    }

    /// 기록(meta.json)의 결과 칸 (Python 기록과 같은 이름).
    pub fn to_json(&self) -> Value {
        let summaries: Vec<Value> = self
            .summaries
            .iter()
            .map(|s| {
                json!({
                    "phase": s.phase.name(),
                    "op": s.op.name(),
                    "count": s.count,
                    "failures": s.failures,
                    "over_period": s.over_period,
                    "min_ms": s.min_ms,
                    "median_ms": s.median_ms,
                    "p99_ms": s.p99_ms,
                    "max_ms": s.max_ms,
                    "wire_ms": s.wire_ms,
                })
            })
            .collect();
        json!({
            "ids": self.ids,
            "models": self.models.iter().map(|(id, m)| (id.to_string(), json!(m))).collect::<serde_json::Map<_, _>>(),
            "count": self.count,
            "period_s": self.period_s,
            "response_delay_before": self
                .delays_before
                .iter()
                .map(|(id, d)| (id.to_string(), json!(d)))
                .collect::<serde_json::Map<_, _>>(),
            "response_delay_step_s": RESPONSE_DELAY_STEP_S,
            "tick_write": TICK_WRITE.name,
            "trial_error": self.trial_error,
            "summaries": summaries,
        })
    }

    /// samples.csv의 글 (Python 기록과 같은 열: phase, op, servo_id, t_s, duration_ms, error).
    pub fn samples_csv(&self) -> String {
        let mut text = String::from("phase,op,servo_id,t_s,duration_ms,error\n");
        for s in &self.samples {
            let servo = s.servo_id.map(|id| id.to_string()).unwrap_or_default();
            let _ = writeln!(
                text,
                "{},{},{servo},{:.6},{:.4},{}",
                s.phase.name(),
                s.op.name(),
                s.t_s,
                s.duration_s * 1000.0,
                s.error
            );
        }
        text
    }

    /// 사람이 볼 표.
    pub fn report(&self) -> String {
        let mut text = String::new();
        let period_ms = self.period_s * 1000.0;
        for phase in [Phase::AsIs, Phase::DelayZero] {
            let rows: Vec<&Summary> = self.summaries.iter().filter(|s| s.phase == phase).collect();
            if rows.is_empty() {
                continue;
            }
            let _ = writeln!(text);
            if phase == Phase::DelayZero {
                let _ = writeln!(text, "{}: 잠근 채 0으로 바꿨다", phase.label());
            } else {
                let mut kinds: Vec<i32> = self.delays_before.values().copied().collect();
                kinds.sort_unstable();
                kinds.dedup();
                if let [only] = kinds[..] {
                    let us = f64::from(only) * RESPONSE_DELAY_STEP_S * 1e6;
                    let _ = writeln!(text, "{}: 서보마다 7번 = {only} ({us} µs)", phase.label());
                } else {
                    let each: Vec<String> = self
                        .delays_before
                        .iter()
                        .map(|(id, d)| format!("ID {id} = {d}"))
                        .collect();
                    let _ = writeln!(text, "{}: 7번 {}", phase.label(), each.join(", "));
                }
            }
            let _ = writeln!(
                text,
                "  [ms]      {:>5}{:>5}{:>8}{:>8}{:>8}{:>8}{:>8}  {period_ms} ms 넘음",
                "횟수", "실패", "최소", "중간", "p99", "최대", "선로"
            );
            let show = |v: Option<f64>| v.map_or("-".to_owned(), |v| format!("{v:.2}"));
            for s in rows {
                let _ = writeln!(
                    text,
                    "  {:<10}{:>5}{:>5}{:>8}{:>8}{:>8}{:>8}{:>8.2}  {:>10}",
                    s.op.label(),
                    s.count,
                    s.failures,
                    show(s.min_ms),
                    show(s.median_ms),
                    show(s.p99_ms),
                    show(s.max_ms),
                    s.wire_ms,
                    s.over_period
                );
            }
        }
        let _ = writeln!(text);
        let _ = writeln!(
            text,
            "선로는 바이트와 응답 지연만 더한 계산이다. 잰 값과의 차이가 USB와 운영체제의 몫이다."
        );
        if let Some(error) = &self.trial_error {
            let _ = writeln!(text, "응답 지연을 0으로 바꾸지 못해 한 번만 쟀다: {error}");
            let _ = writeln!(text, "서보 전원(+ 플러그)을 뺐다 꽂아 처음 상태로 돌린다.");
        } else if self.delays_before.values().any(|&d| d != 0) {
            let _ = writeln!(
                text,
                "응답 지연을 0으로 바꿔 두었다 (잠근 채라 서보 전원을 껐다 켜면 처음 값으로 돌아간다)."
            );
        } else {
            let _ = writeln!(text, "모든 서보의 응답 지연이 이미 0이라 한 번만 쟀다.");
        }
        text
    }
}

#[cfg(test)]
mod tests {
    use horibot_servo::feetech::fake::{FakeServo, open_fake_bus};

    use super::*;

    /// 기다리지 않는 시계: 자면 그 시각으로 가고, 잴 때마다 조금씩 간다.
    struct FakeClock {
        t: std::cell::Cell<f64>,
    }

    impl Clock for FakeClock {
        fn now(&self) -> f64 {
            let t = self.t.get() + 0.0005;
            self.t.set(t);
            t
        }

        fn sleep_until(&mut self, t: f64) {
            if t > self.t.get() {
                self.t.set(t);
            }
        }
    }

    fn clock() -> FakeClock {
        FakeClock {
            t: std::cell::Cell::new(0.0),
        }
    }

    const ARM: [(u8, u16); 7] = [
        (1, 777),
        (2, 3080),
        (3, 2825),
        (4, 777),
        (5, 777),
        (6, 777),
        (11, 777),
    ];

    fn arm() -> Vec<FakeServo> {
        ARM.iter()
            .map(|&(id, model)| FakeServo::with(id, model, 2048))
            .collect()
    }

    #[test]
    fn the_tick_on_the_wire_matches_the_first_connection_estimate() {
        // 0024의 첫 연결 결과: 서보 7개, 응답 지연 0이면 한 박자의 선로 시간은 1.91 ms다
        let wire = wire_time_s(Op::Tick, &[0; 7], 1_000_000);
        assert!((wire - 1.91e-3).abs() < 1e-9, "{wire}");
        // 응답 지연 250칸(500 µs)이면 동기 읽기마다 서보 수만큼 더해진다
        let slow = wire_time_s(Op::SyncRead, &[250; 7], 1_000_000);
        let fast = wire_time_s(Op::SyncRead, &[0; 7], 1_000_000);
        assert!((slow - fast - 7.0 * 500e-6).abs() < 1e-9);
        let short = wire_time_s(Op::ShortRead, &[0; 7], 1_000_000);
        assert!((short - 16.0 * 1e-5).abs() < 1e-12);
    }

    #[test]
    fn every_op_is_measured_once_per_beat_without_moving_anything() {
        let mut bus = open_fake_bus(arm());
        let ids: Vec<u8> = ARM.iter().map(|&(id, _)| id).collect();
        let before = read_servos(&mut bus, &ids).unwrap();
        let run = measure(&mut bus, &ids, 20, 0.01, 1_000_000, &mut clock()).unwrap();
        assert_eq!(
            run.samples.len(),
            4 * 20,
            "응답 지연이 0이면 한 단계만 잰다"
        );
        assert!(run.samples.iter().all(|s| s.error.is_empty()));
        assert_eq!(run.summaries.len(), 4);
        let tick = &run.summaries[3];
        assert_eq!((tick.op, tick.count, tick.failures), (Op::Tick, 20, 0));
        assert!((tick.wire_ms - 1.91).abs() < 1e-9);
        // 짧은 읽기는 서보를 돌아가며, 동기는 전체다
        assert_eq!(run.samples[0].servo_id, Some(1));
        assert_eq!(run.samples[1].servo_id, Some(2));
        assert_eq!(run.samples[2 * 20].servo_id, None);
        // 주기마다 한 번: 시작 시각이 주기만큼 벌어진다
        let gap = run.samples[1].t_s - run.samples[0].t_s;
        assert!((gap - 0.01).abs() < 1e-6, "{gap}");
        // 토크도 한도도 그대로다
        assert_eq!(read_servos(&mut bus, &ids).unwrap(), before);
        assert_eq!(run.trial_error, None);
        let csv = run.samples_csv();
        assert!(csv.starts_with("phase,op,servo_id,t_s,duration_ms,error\nas_is,short_read,1,"));
        assert_eq!(csv.lines().count(), 1 + 80);
        let meta = run.to_json();
        assert_eq!(meta["tick_write"], "torque_limit");
        assert_eq!(meta["models"]["2"], 3080);
        assert_eq!(meta["summaries"][3]["op"], "tick");
    }

    #[test]
    fn a_nonzero_response_delay_is_measured_again_at_zero() {
        let mut servos = arm();
        servos[2].memory[usize::from(reg::RESPONSE_DELAY.address)] = 250;
        let mut bus = open_fake_bus(servos);
        let ids: Vec<u8> = ARM.iter().map(|&(id, _)| id).collect();
        let run = measure(&mut bus, &ids, 5, 0.01, 1_000_000, &mut clock()).unwrap();
        assert_eq!(run.delays_before[&3], 250);
        assert_eq!(run.samples.len(), 2 * 4 * 5);
        assert_eq!(run.summaries.len(), 8);
        assert!(
            run.summaries[3].wire_ms > run.summaries[7].wire_ms,
            "응답 지연 0의 선로 시간이 짧다"
        );
        assert_eq!(read_one(&mut bus, 3, reg::RESPONSE_DELAY).unwrap(), 0);
        assert!(run.report().contains("응답 지연을 0으로 바꿔 두었다"));
    }

    #[test]
    fn a_servo_with_torque_on_is_not_measured() {
        let mut bus = open_fake_bus(arm());
        bus.write(4, reg::TORQUE_ENABLE, reg::TORQUE_ON).unwrap();
        let error = measure(&mut bus, &[1, 4], 5, 0.01, 1_000_000, &mut clock()).unwrap_err();
        assert!(error.contains("서보 4"), "{error}");
    }

    #[test]
    fn a_missing_servo_is_counted_as_a_failure() {
        let mut bus = open_fake_bus(arm());
        let ids = [1, 2];
        let mut run = measure(&mut bus, &ids, 4, 0.01, 1_000_000, &mut clock()).unwrap();
        // 서보 하나가 빠진 버스로 다시 재면 동기 읽기마다 그 서보의 실패가 남는다
        let mut bus = open_fake_bus(arm().into_iter().filter(|s| s.id() != 2));
        run.samples.clear();
        run.summaries.clear();
        let limits = [(1, 1000), (2, 1000)];
        run.phase(&mut bus, Phase::AsIs, &[0, 0], &limits, &mut clock(), 0.0);
        let sync = run.summaries.iter().find(|s| s.op == Op::SyncRead).unwrap();
        assert_eq!(sync.failures, 4);
        let failed = run.samples.iter().find(|s| s.op == Op::SyncRead).unwrap();
        assert_eq!(failed.error, "2:NoReply");
        let short = run
            .summaries
            .iter()
            .find(|s| s.op == Op::ShortRead)
            .unwrap();
        assert_eq!(
            (short.count, short.failures),
            (4, 2),
            "서보 2를 읽은 차례만 실패"
        );
    }

    #[test]
    fn percentiles_follow_the_python_reference() {
        let values: Vec<f64> = (1..=100).map(f64::from).collect();
        assert_eq!(percentile(&values, 0.99), 99.0);
        assert_eq!(median(&values), 50.5);
        assert_eq!(median(&values[..3]), 2.0);
        assert_eq!(percentile(&values[..1], 0.99), 1.0);
    }
}
