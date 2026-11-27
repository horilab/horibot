//! 게임패드 (0023의 8, 0034의 실행 명령): 컨트롤러 파이에 동글을 꽂은 무선 게임패드를 evdev로 읽고 떨게 한다.
//!
//! 실행 명령에서는 사람이 팔 옆에서 쥐는 리모컨이고, 버튼은 둘만 쓴다: A는 다음(시작, 멈춤에서 넘어가기), B는
//! 멈춤(언제든 그 자리에 서서 순서를 끝낸다). 다른 버튼, 트리거, 십자 키, 스틱은 보지 않는다: 게임패드를 쥐다가
//! 눌려도 아무 일도 없다. 게임패드가 사라지면(끔, 배터리, 잠듦, 동글이 빠짐) 읽기가 ENODEV로 끝난다.
//!
//! 번호는 리눅스 입력 번호다. 8BitDo Ultimate 2C를 2.4 GHz 동글로 쓰면 커널의 xpad가 Xbox 360 게임패드로 잡아,
//! A는 BTN_SOUTH(304), B는 BTN_EAST(305)다 (docs/hardware.md). 게임패드를 바꾸면 이 번호를 다시 본다.
//!
//! evdev는 리눅스에서만 돈다. 다른 운영 체제에서는 [`Gamepad::find`]가 오류를 내고, 실행 명령은 Enter만 쓴다.

/// 버튼 하나를 누른 뜻.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Press {
    /// A: 다음 (시작, 멈춤에서 넘어가기)
    Next,
    /// B: 멈춤 (언제든 그 자리에 서서 끝낸다)
    Stop,
}

/// 떨림의 모양.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Buzz {
    /// 짧게 한 번: 게임패드가 붙었다 (시작할 때)
    Ready,
    /// 짧게 두 번: 사람이 A를 누를 차례다 (멈춤)
    Turn,
    /// 길게 한 번: 순서가 끝났다 (마쳤거나 섰다). 화면을 본다
    End,
}

const EV_KEY: u16 = 0x01;
const BTN_SOUTH: u16 = 0x130;
const BTN_EAST: u16 = 0x131;

/// 입력 이벤트 하나(종류, 번호, 값)를 누름으로 바꾼다. 누를 때(값 1)만 센다: 뗄 때(0)와 누르고 있을 때의
/// 되풀이(2)는 버린다.
pub fn press_of(kind: u16, code: u16, value: i32) -> Option<Press> {
    match (kind, code, value) {
        (EV_KEY, BTN_SOUTH, 1) => Some(Press::Next),
        (EV_KEY, BTN_EAST, 1) => Some(Press::Stop),
        _ => None,
    }
}

#[cfg(target_os = "linux")]
mod device {
    use std::io;
    use std::path::{Path, PathBuf};
    use std::thread;

    use evdev::{Device, FFEffect, FFEffectCode, FFEffectData, FFEffectKind, FFReplay, FFTrigger};

    use super::{Buzz, Press, press_of};

    /// 게임패드를 찾는 곳: 이름이 [`SUFFIX`]로 끝나는 링크가 게임패드의 입력 장치다.
    const BY_ID: &str = "/dev/input/by-id";
    const SUFFIX: &str = "-event-joystick";
    const HINT: &str = "게임패드를 켠다 (동글은 컨트롤러 파이에 꽂혀 있어야 한다)";
    const ENODEV: i32 = 19;

    /// 연 게임패드: 읽기는 [`Gamepad::wait_press`]로 하다가 [`Gamepad::spawn`]으로 스레드에 넘기고, 떨림은 이쪽에
    /// 남는다 (떨림 효과는 장치 파일을 따로 복제해 쥔다).
    pub struct Gamepad {
        name: String,
        path: PathBuf,
        device: Option<Device>,
        /// 짧은 떨림 (앞에 쉬는 틈이 있어 두 번 틀면 두 번 떤다)
        pulse: Option<FFEffect>,
        long: Option<FFEffect>,
    }

    impl Gamepad {
        /// `/dev/input/by-id/`에서 게임패드(이름이 `-event-joystick`으로 끝나는 첫 장치)를 찾아 연다.
        pub fn find() -> Result<Self, String> {
            let entries = std::fs::read_dir(BY_ID)
                .map_err(|e| format!("{BY_ID}를 읽지 못했다: {e}. {HINT}"))?;
            let mut paths: Vec<PathBuf> = entries
                .filter_map(Result::ok)
                .map(|entry| entry.path())
                .filter(|path| {
                    path.file_name()
                        .and_then(|name| name.to_str())
                        .is_some_and(|name| name.ends_with(SUFFIX))
                })
                .collect();
            paths.sort();
            let path = paths
                .into_iter()
                .next()
                .ok_or_else(|| format!("게임패드가 없다 ({BY_ID}/*{SUFFIX}). {HINT}"))?;
            Self::open(&path)
        }

        /// 이 경로의 게임패드를 열고, 떨 수 있다고 답하면 떨림 효과를 올린다.
        pub fn open(path: &Path) -> Result<Self, String> {
            let mut device = Device::open(path)
                .map_err(|e| format!("게임패드 {}를 열지 못했다: {e}", path.display()))?;
            let name = device.name().unwrap_or("이름 없음").to_owned();
            let rumbles = device
                .supported_ff()
                .is_some_and(|ff| ff.contains(FFEffectCode::FF_RUMBLE));
            let (pulse, long) = if rumbles {
                (
                    upload(&mut device, 150, 120).ok(),
                    upload(&mut device, 700, 0).ok(),
                )
            } else {
                (None, None)
            };
            Ok(Self {
                name,
                path: path.to_owned(),
                device: Some(device),
                pulse,
                long,
            })
        }

        pub fn name(&self) -> &str {
            &self.name
        }

        pub fn path(&self) -> &Path {
            &self.path
        }

        /// 게임패드가 떨 수 있다고 답해 떨림 효과를 올렸다 (실제로 떨리는지는 사람이 본다).
        pub fn rumbles(&self) -> bool {
            self.pulse.is_some() && self.long.is_some()
        }

        /// A나 B를 누를 때까지 기다린다. 게임패드가 사라지면 오류다.
        pub fn wait_press(&mut self) -> Result<Press, String> {
            let device = self.device.as_mut().ok_or("게임패드 읽기를 이미 넘겼다")?;
            loop {
                let events = device.fetch_events().map_err(gone_text)?;
                for event in events {
                    if let Some(press) = press_of(event.event_type().0, event.code(), event.value())
                    {
                        return Ok(press);
                    }
                }
            }
        }

        /// 읽기를 스레드로 넘긴다: 누를 때마다 `press`를 부르고, 게임패드가 사라지면 `gone`을 부르고 끝난다.
        pub fn spawn(
            &mut self,
            mut press: impl FnMut(Press) + Send + 'static,
            gone: impl FnOnce(String) + Send + 'static,
        ) -> Result<(), String> {
            let mut device = self.device.take().ok_or("게임패드 읽기를 이미 넘겼다")?;
            thread::Builder::new()
                .name("gamepad".to_owned())
                .spawn(move || {
                    crate::system::make_background_thread();
                    loop {
                        match device.fetch_events() {
                            Ok(events) => {
                                for event in events {
                                    if let Some(pressed) =
                                        press_of(event.event_type().0, event.code(), event.value())
                                    {
                                        press(pressed);
                                    }
                                }
                            }
                            Err(error) => {
                                gone(gone_text(error));
                                return;
                            }
                        }
                    }
                })
                .map(|_| ())
                .map_err(|e| format!("게임패드 스레드를 띄우지 못했다: {e}"))
        }

        /// 떤다. 떨지 못하면 그냥 둔다 (떨림이 없거나 게임패드가 사라졌다).
        pub fn buzz(&mut self, buzz: Buzz) {
            let (effect, count) = match buzz {
                Buzz::Ready => (&mut self.pulse, 1),
                Buzz::Turn => (&mut self.pulse, 2),
                Buzz::End => (&mut self.long, 1),
            };
            if let Some(effect) = effect {
                let _ = effect.play(count);
            }
        }
    }

    /// 두 모터를 3/4 세기로 `length_ms` 동안 떠는 효과. `delay_ms`는 틀 때마다 앞에 쉬는 틈이다.
    fn upload(device: &mut Device, length_ms: u16, delay_ms: u16) -> io::Result<FFEffect> {
        device.upload_ff_effect(FFEffectData {
            direction: 0,
            trigger: FFTrigger::default(),
            replay: FFReplay {
                length: length_ms,
                delay: delay_ms,
            },
            kind: FFEffectKind::Rumble {
                strong_magnitude: 0xC000,
                weak_magnitude: 0xC000,
            },
        })
    }

    fn gone_text(error: io::Error) -> String {
        if error.raw_os_error() == Some(ENODEV) {
            "게임패드가 끊겼다 (꺼졌거나 잠들었거나 동글이 빠졌다)".to_owned()
        } else {
            format!("게임패드를 읽지 못했다: {error}")
        }
    }
}

#[cfg(not(target_os = "linux"))]
mod device {
    use std::convert::Infallible;
    use std::path::Path;

    use super::{Buzz, Press};

    /// 리눅스가 아니면 게임패드를 읽지 않는다: 값을 만들 수 없는 꼴이라 아래 메서드는 불리지 않는다.
    pub struct Gamepad {
        never: Infallible,
    }

    impl Gamepad {
        pub fn find() -> Result<Self, String> {
            Err("게임패드는 리눅스(컨트롤러 파이)에서만 읽는다".to_owned())
        }

        pub fn name(&self) -> &str {
            match self.never {}
        }

        pub fn path(&self) -> &Path {
            match self.never {}
        }

        pub fn rumbles(&self) -> bool {
            match self.never {}
        }

        pub fn wait_press(&mut self) -> Result<Press, String> {
            match self.never {}
        }

        pub fn spawn(
            &mut self,
            _press: impl FnMut(Press) + Send + 'static,
            _gone: impl FnOnce(String) + Send + 'static,
        ) -> Result<(), String> {
            match self.never {}
        }

        pub fn buzz(&mut self, _buzz: Buzz) {
            match self.never {}
        }
    }
}

pub use device::Gamepad;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn only_a_and_b_count_and_only_when_pressed() {
        assert_eq!(press_of(EV_KEY, BTN_SOUTH, 1), Some(Press::Next));
        assert_eq!(press_of(EV_KEY, BTN_EAST, 1), Some(Press::Stop));
        // 누르고 있는 동안의 되풀이와 뗄 때는 세지 않는다
        for value in [0, 2] {
            assert_eq!(press_of(EV_KEY, BTN_SOUTH, value), None);
            assert_eq!(press_of(EV_KEY, BTN_EAST, value), None);
        }
        // X(307), Y(308), LB(310), RB(311), 홈(316)은 쓰지 않는다
        for code in [307, 308, 310, 311, 316] {
            assert_eq!(press_of(EV_KEY, code, 1), None);
        }
        // 트리거(축 2, 5), 십자 키(축 16, 17), 스틱(축 0, 1, 3, 4)도 쓰지 않는다
        for code in [2, 5, 16, 17, 0, 1, 3, 4] {
            assert_eq!(press_of(0x03, code, 255), None);
        }
    }
}
