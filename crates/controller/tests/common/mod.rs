//! 통합 시험이 같이 쓰는 것: 시뮬레이션 서버 띄우기 (docs/decisions/0033).

use std::io::{BufRead, BufReader};
use std::path::Path;
use std::process::{Child, Command, Stdio};

/// 띄운 시뮬레이션 서버. 시험이 끝나면 끈다.
pub struct Server(Child);

impl Drop for Server {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}

/// 가상환경의 Python으로 서버를 바로 띄운다. `uv run`을 거치면 시험이 끝나 uv를 꺼도 Python 서버가 남아
/// (Windows는 손자 프로세스를 같이 끄지 않는다) 출력 통로를 붙잡는다. 가상환경은 `uv sync`로 만든다.
pub fn start_server() -> (Server, String) {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    let python = if cfg!(windows) {
        root.join(".venv/Scripts/python.exe")
    } else {
        root.join(".venv/bin/python")
    };
    // 시뮬레이션은 설계를 불러오지 않고 설계가 쓴 MJCF만 읽는다 (0007). 시험마다 다른 파일에 쓴다 (같이 돌 수 있다)
    let model = std::env::temp_dir().join(format!(
        "horibot-sim-{}-{:?}.xml",
        std::process::id(),
        std::thread::current().id()
    ));
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
