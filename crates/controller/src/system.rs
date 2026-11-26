//! 컨트롤러가 도는 컴퓨터의 실행 환경 (docs/decisions/0026-pi-runtime-deploy-records.md의 2, 5): 실행 루프
//! 스레드의 실시간 설정과, 컨트롤러 파이의 부팅 인자, 클럭 정책, 시각. 실제로 걸렸는지 스스로 보고 기록에 남긴다.
//!
//! - 실행 루프 스레드는 코어 3에 묶고 SCHED_FIFO 80으로 돈다. 메모리는 스레드를 띄우기 전에 모두 잠근다.
//! - 다른 스레드(기록 쓰기, 터미널)는 코어 3을 피하고 가장 낮은 우선순위로 돈다.
//! - 부팅 인자 `isolcpus=3 irqaffinity=0-2`로 코어 3을 떼어 두고, 클럭 정책은 performance다. 둘 다 파이 처음
//!   설정(`pi/controller/setup.sh`)이 넣는다.

use std::fs;
use std::path::Path;

use serde_json::{Value, json};

use crate::rt;

/// 실행 루프 스레드의 SCHED_FIFO 우선순위 (0026의 2. 99는 커널 감시 스레드 몫이다)
pub const LOOP_PRIORITY: i32 = 80;
/// 실행 루프 스레드를 묶는 코어 (부팅 인자로 떼어 둔다)
pub const LOOP_CPU: usize = 3;
/// 컨트롤러 파이의 부팅 인자
pub const BOOT_ARGS: [&str; 2] = ["isolcpus=3", "irqaffinity=0-2"];
/// 컨트롤러 파이의 클럭 정책
pub const GOVERNOR: &str = "performance";
/// 뒤에서 도는 스레드(기록 쓰기 등)의 nice 값 (가장 낮다)
pub const BACKGROUND_NICE: i32 = 19;
/// 실시간 설정을 걸지 못했을 때 볼 것
pub const LIMITS_HINT: &str =
    "파이 처음 설정(horilab 그룹의 rtprio, memlock 한도)을 했는지, 그 뒤 다시 로그인했는지 본다";

/// 실행 루프 스레드에 실제로 걸린 설정.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct LoopThread {
    pub policy: String,
    pub priority: i32,
    pub affinity: Vec<usize>,
}

impl LoopThread {
    pub fn to_json(&self) -> Value {
        json!({
            "policy": self.policy,
            "priority": self.priority,
            "affinity": self.affinity,
            "memory_locked": true,
        })
    }
}

/// 부른 스레드를 실행 루프 스레드로 만든다: 코어 3에 묶고 SCHED_FIFO 80을 건 뒤 스택을 미리 쓴다. 메모리는
/// 먼저 잠가 둔다 ([`rt::lock_memory`]).
pub fn make_loop_thread() -> Result<LoopThread, String> {
    rt::pin_to_cpus(&[LOOP_CPU])
        .map_err(|e| format!("실행 루프를 코어 {LOOP_CPU}에 묶지 못했다: {e}"))?;
    rt::set_fifo(LOOP_PRIORITY).map_err(|e| {
        format!("실행 루프에 SCHED_FIFO {LOOP_PRIORITY}를 걸지 못했다: {e}. {LIMITS_HINT}")
    })?;
    rt::prefault_stack();
    let (policy, priority) = rt::current_sched();
    Ok(LoopThread {
        policy,
        priority,
        affinity: rt::affinity(),
    })
}

/// 부른 스레드를 뒤에서 도는 스레드로 만든다: 코어 3을 피하고 가장 낮은 우선순위로 둔다. 못 해도 그대로 돈다.
pub fn make_background_thread() {
    let cores = rt::affinity();
    let others: Vec<usize> = cores.iter().copied().filter(|&c| c != LOOP_CPU).collect();
    if others.len() < cores.len() && !others.is_empty() {
        let _ = rt::pin_to_cpus(&others);
    }
    let _ = rt::lower_priority(BACKGROUND_NICE);
}

/// 자가 점검 하나.
#[derive(Debug, Clone, PartialEq)]
pub struct Finding {
    pub name: &'static str,
    pub ok: bool,
    pub detail: String,
}

/// 실행 환경: 부팅 인자, 떼어 둔 코어, 클럭 정책, 시각.
#[derive(Debug, Clone, PartialEq)]
pub struct Environment {
    pub host: String,
    pub kernel: Option<String>,
    /// 부팅 인자 (`/proc/cmdline`). 리눅스가 아니면 없다
    pub cmdline: Option<String>,
    /// 커널이 떼어 둔 코어 (`/sys/devices/system/cpu/isolated`)
    pub isolated: Option<String>,
    /// 클럭 정책 (cpufreq 묶음마다. 파이 4는 네 코어가 한 묶음이다)
    pub governors: Vec<String>,
    pub realtime_kernel: bool,
    /// 시스템 시계가 시각 서버에 맞춰졌나 (못 읽으면 없다)
    pub clock_synced: Option<bool>,
}

fn read_trimmed(path: impl AsRef<Path>) -> Option<String> {
    fs::read_to_string(path).ok().map(|s| s.trim().to_owned())
}

impl Environment {
    /// 이 컴퓨터의 값을 읽는다.
    pub fn read() -> Self {
        let mut governors: Vec<(String, String)> = fs::read_dir("/sys/devices/system/cpu/cpufreq")
            .into_iter()
            .flatten()
            .flatten()
            .filter(|entry| entry.file_name().to_string_lossy().starts_with("policy"))
            .filter_map(|entry| {
                let name = entry.file_name().to_string_lossy().into_owned();
                read_trimmed(entry.path().join("scaling_governor")).map(|g| (name, g))
            })
            .collect();
        governors.sort();
        Self {
            host: crate::record::hostname(),
            kernel: read_trimmed("/proc/version"),
            cmdline: read_trimmed("/proc/cmdline"),
            isolated: read_trimmed("/sys/devices/system/cpu/isolated"),
            governors: governors.into_iter().map(|(_, g)| g).collect(),
            realtime_kernel: Path::new("/sys/kernel/realtime").exists(),
            clock_synced: rt::clock_synchronized(),
        }
    }

    /// 컨트롤러 파이의 설정이 걸렸는지. 시각은 맞지 않아도 돌지만 기록에 표시가 남는다 (0026의 5).
    pub fn findings(&self) -> Vec<Finding> {
        let Some(cmdline) = &self.cmdline else {
            return vec![Finding {
                name: "실행 환경",
                ok: false,
                detail: "리눅스가 아니다 (컨트롤러 파이의 설정을 보지 않는다)".to_owned(),
            }];
        };
        let args: Vec<&str> = cmdline.split_whitespace().collect();
        let missing: Vec<&str> = BOOT_ARGS
            .iter()
            .copied()
            .filter(|arg| !args.contains(arg))
            .collect();
        let isolated = self.isolated.as_deref().unwrap_or("");
        let cores = if !missing.is_empty() {
            Finding {
                name: "코어 3 떼어 두기",
                ok: false,
                detail: format!("부팅 인자에 {}가 없다", missing.join(" ")),
            }
        } else if isolated != LOOP_CPU.to_string() {
            Finding {
                name: "코어 3 떼어 두기",
                ok: false,
                detail: format!("부팅 인자는 있는데 커널이 떼어 둔 코어가 '{isolated}'이다"),
            }
        } else {
            Finding {
                name: "코어 3 떼어 두기",
                ok: true,
                detail: BOOT_ARGS.join(" "),
            }
        };
        let governor = if self.governors.is_empty() {
            Finding {
                name: "클럭 정책",
                ok: false,
                detail: "읽지 못했다".to_owned(),
            }
        } else if self.governors.iter().all(|g| g == GOVERNOR) {
            Finding {
                name: "클럭 정책",
                ok: true,
                detail: GOVERNOR.to_owned(),
            }
        } else {
            Finding {
                name: "클럭 정책",
                ok: false,
                detail: format!("{} ({GOVERNOR}이어야 한다)", self.governors.join(", ")),
            }
        };
        let clock = Finding {
            name: "시각",
            ok: self.clock_synced == Some(true),
            detail: match self.clock_synced {
                Some(true) => "시각 서버에 맞췄다".to_owned(),
                Some(false) => "아직 맞추지 못했다 (기록에 표시가 남는다)".to_owned(),
                None => "맞췄는지 읽지 못했다".to_owned(),
            },
        };
        vec![cores, governor, clock]
    }

    pub fn to_json(&self) -> Value {
        json!({
            "host": self.host,
            "kernel": self.kernel,
            "cmdline": self.cmdline,
            "isolated": self.isolated,
            "governors": self.governors,
            "realtime_kernel": self.realtime_kernel,
            "clock_synced": self.clock_synced,
            "target": format!(
                "{}-{} ({})",
                std::env::consts::ARCH,
                std::env::consts::OS,
                if cfg!(target_env = "musl") { "musl 정적" } else { "동적" }
            ),
        })
    }
}

/// 프로그램 판: 이름, 판 번호, 빌드한 커밋.
pub fn version() -> String {
    let uncommitted = match env!("HORIBOT_UNCOMMITTED") {
        "true" => ", 커밋하지 않은 변경 있음",
        "false" => "",
        _ => ", 변경 여부 모름",
    };
    format!(
        "horibot-controller {} (커밋 {}{uncommitted})",
        env!("CARGO_PKG_VERSION"),
        env!("HORIBOT_COMMIT")
    )
}

/// 서보 어댑터의 고정 경로 (`/dev/serial/by-id/`). 다시 꽂거나 다시 켜도 같은 이름이라 컨트롤러 파이에서는
/// 이 경로를 `--port`로 준다. 리눅스가 아니거나 없으면 비어 있다.
pub fn serial_by_id() -> Vec<String> {
    let mut paths: Vec<String> = fs::read_dir("/dev/serial/by-id")
        .into_iter()
        .flatten()
        .flatten()
        .map(|entry| entry.path().display().to_string())
        .collect();
    paths.sort();
    paths
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pi(cmdline: &str, isolated: &str, governors: &[&str], clock: Option<bool>) -> Environment {
        Environment {
            host: "hori1".to_owned(),
            kernel: None,
            cmdline: Some(cmdline.to_owned()),
            isolated: Some(isolated.to_owned()),
            governors: governors.iter().map(|g| (*g).to_owned()).collect(),
            realtime_kernel: false,
            clock_synced: clock,
        }
    }

    #[test]
    fn a_set_up_pi_passes_every_finding() {
        let env = pi(
            "console=tty1 root=LABEL=writable rootwait fixrtc isolcpus=3 irqaffinity=0-2",
            "3",
            &["performance"],
            Some(true),
        );
        assert!(env.findings().iter().all(|f| f.ok), "{:?}", env.findings());
    }

    #[test]
    fn missing_boot_args_and_governor_are_named() {
        let env = pi("console=tty1 isolcpus=3", "3", &["ondemand"], Some(false));
        let findings = env.findings();
        assert!(!findings[0].ok);
        assert!(
            findings[0].detail.contains("irqaffinity=0-2"),
            "{findings:?}"
        );
        assert!(!findings[1].ok);
        assert!(findings[1].detail.contains("ondemand"));
        assert!(!findings[2].ok);
        // 비슷한 인자(isolcpus=2,3)는 같은 것으로 보지 않는다
        let env = pi(
            "isolcpus=2,3 irqaffinity=0-2",
            "2-3",
            &["performance"],
            None,
        );
        assert!(!env.findings()[0].ok);
    }

    #[test]
    fn args_present_but_not_applied_are_caught() {
        let env = pi(
            "isolcpus=3 irqaffinity=0-2",
            "",
            &["performance"],
            Some(true),
        );
        let findings = env.findings();
        assert!(!findings[0].ok);
        assert!(findings[0].detail.contains("떼어 둔 코어"), "{findings:?}");
    }

    #[test]
    fn off_linux_there_is_one_note() {
        let env = Environment {
            cmdline: None,
            ..pi("", "", &[], None)
        };
        let findings = env.findings();
        assert_eq!(findings.len(), 1);
        assert!(!findings[0].ok);
    }

    #[test]
    fn version_names_the_commit() {
        assert!(version().starts_with("horibot-controller "));
        assert!(version().contains("커밋"));
    }
}
