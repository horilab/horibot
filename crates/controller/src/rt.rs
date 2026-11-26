//! 실행 루프가 쓰는 운영체제 기능 (docs/decisions/0026-pi-runtime-deploy-records.md의 2): 단조 시계, 정한
//! 시각까지 자기, 실시간 우선순위, 코어 고정, 메모리 잠금, 낮은 우선순위, 멈춤 신호, 시각이 맞았는지.
//! 컨트롤러의 실행 명령과 bench의 부하 시험이 같이 쓴다.
//!
//! 리눅스(파이)에서만 진짜로 한다. 다른 OS(개발 PC)에서는 시험이 돌 만큼만 흉내 내고, 실시간 기능은
//! 오류를 낸다.
//!
//! - 실시간 우선순위는 스레드마다 `pthread_setschedparam`으로 건다. musl의 `sched_setscheduler`는
//!   일부러 아무것도 하지 않고 오류를 낸다.
//! - 시계는 CLOCK_MONOTONIC이다. 자기는 `clock_nanosleep`에 깨어날 시각을 주는 방식이라, 늦게 깬
//!   만큼이 다음 박자로 쌓이지 않는다.
//! - root가 아니어도 SCHED_FIFO와 메모리 잠금을 쓰려면 사용자의 한도(rtprio, memlock)가 있어야 한다. 컨트롤러
//!   파이는 처음 설정이 horilab 그룹에 이 한도를 준다 (`pi/controller/setup.sh`).

// libc 시스템 호출을 부른다. 부르는 곳마다 SAFETY에 이유를 적는다.
#![allow(unsafe_code)]

use std::sync::atomic::{AtomicBool, Ordering};

static STOP: AtomicBool = AtomicBool::new(false);

/// SIGINT(Ctrl+C)나 SIGTERM을 받았나. 실행 루프가 박자마다 보고 끝낸다.
pub fn stop_requested() -> bool {
    STOP.load(Ordering::Relaxed)
}

pub use imp::{
    affinity, catch_stop_signals, clock_synchronized, current_sched, lock_memory, lower_priority,
    now_ns, pin_to_cpus, set_fifo, sleep_until_ns,
};

/// 실시간 스레드의 스택을 미리 쓰는 크기. 스레드 스택은 이보다 커야 한다.
pub const PREFAULT_STACK: usize = 256 * 1024;

/// 부른 스레드의 스택을 [`PREFAULT_STACK`]만큼 미리 한 번 써서, 박자 도중에 처음 쓰는 페이지 때문에 늦지 않게
/// 한다. 메모리를 잠근 뒤에 부른다.
pub fn prefault_stack() {
    let mut pages = [0u8; PREFAULT_STACK];
    for i in (0..pages.len()).step_by(4096) {
        pages[i] = 1;
    }
    std::hint::black_box(&pages);
}

#[cfg(target_os = "linux")]
mod imp {
    use std::io;
    use std::mem::size_of;
    use std::sync::atomic::Ordering;

    use super::STOP;

    const NS_PER_S: u64 = 1_000_000_000;

    fn timespec(ns: u64) -> libc::timespec {
        // SAFETY: timespec은 정수 칸뿐이라 0으로 채운 값도 올바르다. 대상마다 붙는 칸이 달라서 이렇게 만든다.
        let mut ts: libc::timespec = unsafe { std::mem::zeroed() };
        ts.tv_sec = (ns / NS_PER_S) as _;
        ts.tv_nsec = (ns % NS_PER_S) as _;
        ts
    }

    /// 단조 시계 [ns].
    pub fn now_ns() -> u64 {
        let mut ts = timespec(0);
        // SAFETY: ts는 이 함수의 지역 변수이고, clock_gettime은 그 자리에 쓰기만 한다.
        let rc = unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut ts) };
        debug_assert_eq!(rc, 0, "단조 시계는 늘 읽힌다");
        ts.tv_sec as u64 * NS_PER_S + ts.tv_nsec as u64
    }

    /// 단조 시계로 `deadline_ns`까지 잔다. 이미 지났으면 바로 돌아온다. 신호로 깨면 다시 잔다.
    pub fn sleep_until_ns(deadline_ns: u64) {
        let ts = timespec(deadline_ns);
        loop {
            // SAFETY: ts는 읽기만 한다. 깨어날 시각을 주는 방식(TIMER_ABSTIME)이라 남은 시간 자리는 쓰지 않아 null이다.
            let rc = unsafe {
                libc::clock_nanosleep(
                    libc::CLOCK_MONOTONIC,
                    libc::TIMER_ABSTIME,
                    &ts,
                    std::ptr::null_mut(),
                )
            };
            if rc != libc::EINTR {
                break;
            }
        }
    }

    /// 부른 스레드를 SCHED_FIFO `priority`로 돌린다. root나 CAP_SYS_NICE가 있어야 한다.
    pub fn set_fifo(priority: i32) -> io::Result<()> {
        // SAFETY: sched_param은 정수 칸뿐이라 0으로 채운 값도 올바르다.
        let mut param: libc::sched_param = unsafe { std::mem::zeroed() };
        param.sched_priority = priority;
        // SAFETY: pthread_self()는 지금 도는 스레드라 살아 있고, param은 읽기만 한다.
        let rc =
            unsafe { libc::pthread_setschedparam(libc::pthread_self(), libc::SCHED_FIFO, &param) };
        if rc == 0 {
            Ok(())
        } else {
            Err(io::Error::from_raw_os_error(rc))
        }
    }

    /// 부른 스레드의 스케줄 정책과 우선순위. 실제로 걸렸는지 기록에 남긴다.
    pub fn current_sched() -> (String, i32) {
        let mut policy = 0;
        // SAFETY: sched_param은 정수 칸뿐이라 0으로 채운 값도 올바르다.
        let mut param: libc::sched_param = unsafe { std::mem::zeroed() };
        // SAFETY: 두 자리 모두 이 함수의 지역 변수이고, 함수가 쓰기만 한다.
        let rc =
            unsafe { libc::pthread_getschedparam(libc::pthread_self(), &mut policy, &mut param) };
        if rc != 0 {
            return (
                format!("읽지 못함 ({})", io::Error::from_raw_os_error(rc)),
                0,
            );
        }
        let name = match policy {
            libc::SCHED_FIFO => "fifo".to_owned(),
            libc::SCHED_RR => "rr".to_owned(),
            libc::SCHED_OTHER => "other".to_owned(),
            other => format!("정책 {other}"),
        };
        (name, param.sched_priority)
    }

    /// 부른 스레드를 `cpus` 코어에서만 돌게 한다.
    pub fn pin_to_cpus(cpus: &[usize]) -> io::Result<()> {
        let limit = libc::CPU_SETSIZE as usize;
        if cpus.is_empty() || cpus.iter().any(|&cpu| cpu >= limit) {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                format!("코어 번호가 이상하다: {cpus:?}"),
            ));
        }
        // SAFETY: cpu_set_t는 비트 묶음이라 0으로 채운 값이 빈 집합이다.
        let mut set: libc::cpu_set_t = unsafe { std::mem::zeroed() };
        for &cpu in cpus {
            // SAFETY: cpu가 집합 크기(CPU_SETSIZE) 안인지 위에서 봤다.
            unsafe { libc::CPU_SET(cpu, &mut set) };
        }
        // SAFETY: pid 0은 부른 스레드이고, set은 읽기만 한다.
        let rc = unsafe { libc::sched_setaffinity(0, size_of::<libc::cpu_set_t>(), &set) };
        if rc == 0 {
            Ok(())
        } else {
            Err(io::Error::last_os_error())
        }
    }

    /// 부른 스레드가 돌 수 있는 코어.
    pub fn affinity() -> Vec<usize> {
        // SAFETY: cpu_set_t는 비트 묶음이라 0으로 채운 값이 빈 집합이다.
        let mut set: libc::cpu_set_t = unsafe { std::mem::zeroed() };
        // SAFETY: pid 0은 부른 스레드이고, set은 이 함수의 지역 변수라 함수가 쓰기만 한다.
        let rc = unsafe { libc::sched_getaffinity(0, size_of::<libc::cpu_set_t>(), &mut set) };
        if rc != 0 {
            return Vec::new();
        }
        (0..libc::CPU_SETSIZE as usize)
            // SAFETY: cpu는 집합 크기(CPU_SETSIZE) 안이다.
            .filter(|&cpu| unsafe { libc::CPU_ISSET(cpu, &set) })
            .collect()
    }

    /// 지금 쓰는 메모리와 앞으로 쓸 메모리를 모두 잠근다. 페이지를 처음 쓰거나 디스크에서 다시 읽느라
    /// 늦어지지 않게 한다. 다른 스레드를 띄우기 전에 부른다.
    pub fn lock_memory() -> io::Result<()> {
        // SAFETY: 인자는 플래그뿐이다.
        let rc = unsafe { libc::mlockall(libc::MCL_CURRENT | libc::MCL_FUTURE) };
        if rc == 0 {
            Ok(())
        } else {
            Err(io::Error::last_os_error())
        }
    }

    /// 부른 스레드의 nice 값을 `nice`로 둔다 (19가 가장 낮다). 디스크 입출력 우선순위도 따라 낮아진다.
    pub fn lower_priority(nice: i32) -> io::Result<()> {
        // SAFETY: gettid 시스템 호출은 인자가 없고 늘 성공한다.
        let tid = unsafe { libc::syscall(libc::SYS_gettid) };
        // SAFETY: 인자는 수뿐이다. 리눅스는 PRIO_PROCESS에 스레드 ID를 주면 그 스레드 하나만 바꾼다.
        let rc = unsafe { libc::setpriority(libc::PRIO_PROCESS, tid as libc::id_t, nice) };
        if rc == 0 {
            Ok(())
        } else {
            Err(io::Error::last_os_error())
        }
    }

    /// 시스템 시계가 시각 서버에 맞춰졌나 (커널의 동기 상태). 읽지 못하면 없다. 파이 4는 시계 배터리가 없어서
    /// 켠 직후에는 맞지 않을 수 있다 (0026의 5).
    pub fn clock_synchronized() -> Option<bool> {
        // SAFETY: timex는 정수 칸뿐이라 0으로 채운 값도 올바르다. modes가 0이면 adjtimex는 읽기만 한다.
        let mut timex: libc::timex = unsafe { std::mem::zeroed() };
        // SAFETY: timex는 이 함수의 지역 변수이고, adjtimex가 그 자리에 쓴다.
        let state = unsafe { libc::adjtimex(&mut timex) };
        if state < 0 {
            return None;
        }
        Some(state != libc::TIME_ERROR && timex.status & libc::STA_UNSYNC == 0)
    }

    extern "C" fn on_stop_signal(_signal: libc::c_int) {
        STOP.store(true, Ordering::Relaxed);
    }

    /// SIGINT와 SIGTERM을 받으면 멈춤 표시만 한다 ([`super::stop_requested`]).
    pub fn catch_stop_signals() -> io::Result<()> {
        let handler =
            on_stop_signal as extern "C" fn(libc::c_int) as *const () as libc::sighandler_t;
        for signal in [libc::SIGINT, libc::SIGTERM] {
            // SAFETY: 처리 함수는 원자 변수에 쓰기만 해서, 신호를 받은 어느 스레드에서 불려도 된다.
            let previous = unsafe { libc::signal(signal, handler) };
            if previous == libc::SIG_ERR {
                return Err(io::Error::last_os_error());
            }
        }
        Ok(())
    }
}

#[cfg(not(target_os = "linux"))]
mod imp {
    use std::io;
    use std::sync::OnceLock;
    use std::time::{Duration, Instant};

    fn epoch() -> Instant {
        static EPOCH: OnceLock<Instant> = OnceLock::new();
        *EPOCH.get_or_init(Instant::now)
    }

    fn unsupported() -> io::Error {
        io::Error::new(io::ErrorKind::Unsupported, "리눅스에서만 된다")
    }

    /// 단조 시계 [ns]. 처음 부른 때가 0이다.
    pub fn now_ns() -> u64 {
        u64::try_from(epoch().elapsed().as_nanos()).unwrap_or(u64::MAX)
    }

    /// `deadline_ns`까지 잔다. 이미 지났으면 바로 돌아온다.
    pub fn sleep_until_ns(deadline_ns: u64) {
        let now = now_ns();
        if deadline_ns > now {
            std::thread::sleep(Duration::from_nanos(deadline_ns - now));
        }
    }

    pub fn set_fifo(_priority: i32) -> io::Result<()> {
        Err(unsupported())
    }

    pub fn current_sched() -> (String, i32) {
        ("other".to_owned(), 0)
    }

    pub fn pin_to_cpus(_cpus: &[usize]) -> io::Result<()> {
        Err(unsupported())
    }

    pub fn affinity() -> Vec<usize> {
        let cores = std::thread::available_parallelism().map_or(1, |n| n.get());
        (0..cores).collect()
    }

    pub fn lock_memory() -> io::Result<()> {
        Err(unsupported())
    }

    pub fn lower_priority(_nice: i32) -> io::Result<()> {
        Err(unsupported())
    }

    pub fn clock_synchronized() -> Option<bool> {
        None
    }

    /// 개발 PC에서는 신호를 잡지 않는다. Ctrl+C로 끄면 기록의 끝 표시가 남지 않는다.
    pub fn catch_stop_signals() -> io::Result<()> {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_clock_moves_forward() {
        let a = now_ns();
        let b = now_ns();
        assert!(b >= a);
    }

    #[test]
    fn sleeping_until_a_time_does_not_wake_early() {
        let deadline = now_ns() + 3_000_000;
        sleep_until_ns(deadline);
        assert!(now_ns() >= deadline);
        // 이미 지난 시각이면 바로 돌아온다
        let before = now_ns();
        sleep_until_ns(before.saturating_sub(1_000_000));
        assert!(now_ns() - before < 1_000_000_000);
    }

    #[test]
    fn this_thread_can_run_somewhere() {
        assert!(!affinity().is_empty());
        let (policy, _) = current_sched();
        assert!(!policy.is_empty());
        assert!(!stop_requested());
    }
}
