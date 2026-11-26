//! 파이에서 미리 재는 시험 프로그램 (docs/decisions/0026-pi-runtime-deploy-records.md의 2).
//!
//! 실행 파일은 `src/bin/`에 있다.
//!
//! - `smoke`: PC에서 파이용으로 교차 빌드한 실행 파일이 파이에서 도는지 본다.
//! - `runloop`: 가짜 서보 버스로 100 Hz 실행 루프를 흉내 내어, 부하를 건 채 깨어나는 지연과 놓친 박자를
//!   잰다 (미리 재기 2).

pub mod runloop;

/// 운영체제 실시간 기능은 컨트롤러의 것을 같이 쓴다 (실행 명령과 같은 코드로 잰다).
pub use horibot_controller::rt;
