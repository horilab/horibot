//! 부딪힘 검사의 시간과 거리 (0041): 자세마다 두 질문을 재고, 원하면 창 안의 짝과 거리를 CSV로 쓴다.
//!
//!     cargo run --release -p horibot-controller --example collision_check -- <자세 json> [짝 csv]
//!
//! 자세 파일은 `uv run python -m horibot_design.studies.collision poses`가 쓴다 (build/collision/poses.json:
//! [{"tag": "rest", "q_deg": [...]}, ...]). 짝 CSV(자세 번호, 부품 a, 부품 b 또는 평면, 조각 거리 [mm])는 같은
//! 검토 코드의 `compare`가 메시 거리와 견준다. 파이에서는 `uv run tools/pi_build.py -p horibot-controller --example
//! collision_check`로 빌드해 `target/aarch64-unknown-linux-musl/release/examples/collision_check`를 data/와 자세
//! 파일과 같이 올려 돌린다 (로봇 설정은 지금 폴더의 data/robot.toml).

use std::fs;
use std::path::Path;
use std::time::Instant;

use horibot_controller::collision::{Contact, Fingers, Model, WINDOW_MM};
use horibot_controller::settings::Settings;
use serde::Deserialize;

/// 짝 CSV의 창 [mm]: 0041에서 메시 거리와 견준 창 (여유 5 mm까지의 판단을 본다)
const PAIR_WINDOW_MM: f32 = 20.0;

#[derive(Deserialize)]
struct PoseFile {
    tag: String,
    q_deg: [f64; 6],
}

fn stats(label: &str, values: &mut [f64]) {
    if values.is_empty() {
        println!("  {label}: 없음");
        return;
    }
    values.sort_by(f64::total_cmp);
    let at = |p: f64| values[((values.len() - 1) as f64 * p).round() as usize];
    println!(
        "  {label} {}개 [ms]: 중간 {:.3}, 99% {:.3}, 최대 {:.3}",
        values.len(),
        at(0.5),
        at(0.99),
        values[values.len() - 1]
    );
}

fn main() -> Result<(), String> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let poses_path = args
        .first()
        .ok_or("자세 json이 없다 (uv run python -m horibot_design.studies.collision poses)")?;
    let text = fs::read_to_string(poses_path).map_err(|e| format!("{poses_path}: {e}"))?;
    let poses: Vec<PoseFile> = serde_json::from_str(&text).map_err(|e| e.to_string())?;
    let settings = Settings::load(Path::new("data/robot.toml"))?;
    let t = Instant::now();
    // 설계 형상(메시)과 견주므로 손가락은 메시와 같은 닫은 자리에 둔다
    let model = Model::load(Path::new("data/collision"), &settings, Fingers::At(0.0))?;
    println!(
        "부품 {}, 읽기 {:.2} s, 자세 {}, 여유 {} mm",
        model.len(),
        t.elapsed().as_secs_f64(),
        poses.len(),
        model.margin()
    );
    let qs: Vec<[f64; 6]> = poses.iter().map(|p| p.q_deg.map(f64::to_radians)).collect();
    // 한 번 데워 둔다 (첫 자세의 캐시 손실을 빼려고)
    for q in qs.iter().take(20) {
        let _ = model.nearest(q, WINDOW_MM);
    }
    let named: Vec<bool> = poses.iter().map(|p| p.tag != "random").collect();
    for (label, run) in [("여유 안에 드나", true), ("30 mm 안 거리", false)] {
        let mut all = Vec::new();
        let mut some = Vec::new();
        let mut blocked = 0;
        for (q, is_named) in qs.iter().zip(&named) {
            let t = Instant::now();
            let hit = if run {
                model.first_within(q, model.margin()).is_some()
            } else {
                model.nearest(q, WINDOW_MM).distance < model.margin()
            };
            let ms = t.elapsed().as_secs_f64() * 1e3;
            all.push(ms);
            if *is_named {
                some.push(ms);
            }
            blocked += usize::from(hit);
        }
        println!("{label} (여유 안에 든 자세 {blocked}):");
        stats("모든 자세", &mut all);
        stats("이름·훑기 자세", &mut some);
    }
    if let Some(out) = args.get(1) {
        let mut rows = String::from("pose,a,b,distance_mm\n");
        for (i, q) in qs.iter().enumerate() {
            for (contact, d) in model.all_within(q, PAIR_WINDOW_MM) {
                let (a, b) = match contact {
                    Contact::Parts(a, b) => (a.to_string(), b.to_string()),
                    Contact::Plane(a, p) => (a.to_string(), format!("plane{p}")),
                };
                rows += &format!("{i},{a},{b},{d:.4}\n");
            }
        }
        fs::write(out, rows).map_err(|e| format!("{out}: {e}"))?;
        println!("짝: {out}");
    }
    Ok(())
}
