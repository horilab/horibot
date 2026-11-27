//! 부딪힘 검사 (docs/decisions/0041-collision-checking.md): 볼록 조각과 parry3d.
//!
//! 형상은 설계가 만든 자료다 (`data/collision/`, `uv run python -m horibot_design.collision build`): 부품마다 볼록
//! 조각의 꼭짓점(부품이 그려진 틀의 좌표, mm)과, 틀마다 붙은 링크와 링크 좌표계에서 본 자리. 자세(관절 각)를 받으면
//! 링크 좌표계의 자리를 정기구학으로 구해 두 질문에 답한다 (0041의 결정 2):
//! - [`Model::first_within`]: 여유 안에 드는 짝이 있나. 하나라도 찾으면 거기서 멈춘다.
//! - [`Model::nearest`]: 창 안에서 가장 가까운 거리. 창 밖은 창으로 본다.
//!
//! 관절 이동(관절 공간의 곧은 길)은 [`Model::check_move`]가 길 위의 자세를 거리를 보며 건너뛰어 검사한다.
//!
//! 보는 것 (0041의 결정 3, 4):
//! - 두 칸 이상 떨어진 링크의 부품끼리. 링크 0은 베이스다 (베이스 몸통, 파이와 덮개, 어댑터, 책상 클램프). 이웃한
//!   링크 사이는 그 관절 각 하나에만 달려서 관절 범위로 지킨다 (설계 시험이 범위 안에서 떨어져 있는지 본다).
//! - 링크 2 이상의 부품과 안전 평면 (로봇 설정의 책상, 벽). 링크 1은 J1 둘레로만 돌아서 평면까지의 거리가 J1 각
//!   하나에만 달린다. 설계 시험이 본다.
//! - 받침에 앉는 짝(로봇 설정의 rest_contact)은 두 링크 사이 관절이 쉬는 자세와 쉬는 자세 위를 잇는 곧은 길 위에
//!   있을 때만 보지 않는다 (앉고 떠나는 길, 0037). 그 밖에서는 다른 짝처럼 본다.
//!
//! 도구: 로봇 설정에 그리퍼가 있으면 그리퍼 부품을 넣는다. 손가락처럼 열림 폭을 따라 움직이는 부품은 읽을 때
//! 고른다 ([`Fingers`]): 실행 명령은 닫은 자리부터 다 연 자리까지 쓸고 간 모양(두 자리 꼭짓점의 볼록 껍질)으로 둔다.
//! 열림 폭과 상관없이 안전하다. 그리퍼 둘레에서 헛경보가 걸리면 지금 열림 폭의 자리로 둔다.
//!
//! 단위: 이 모듈 안은 mm와 f32다. 측정(0041)과 같은 조건이고, parry3d를 f32로 쓴다. 관절 각은 rad, f64로 받는다.

use std::collections::{BTreeMap, HashMap};
use std::fs;
use std::path::Path;
use std::sync::Arc;

use parry3d::bounding_volume::Aabb;
use parry3d::math::{Mat3, Pose, Rotation, Vector};
use parry3d::query::{self, ClosestPoints};
use parry3d::shape::ConvexPolyhedron;
use serde::Deserialize;

use crate::kinematics::{self, Kinematics};
use crate::settings::{CollisionSettings, Settings};

/// 링크 수 (0은 베이스, 1 ~ 6은 관절 1 ~ 6이 움직이는 링크)
pub const LINKS: usize = 7;
/// 이동 검사가 거리를 보는 창 [mm]. 창 안에 아무것도 없으면 창에서 여유를 뺀 만큼 건너뛴다. 0041에서 잰 "30 mm 안
/// 거리"의 창이다 (파이에서 한 자세 중간 0.75 ms, 최대 7 ms, 2026-11-27)
pub const WINDOW_MM: f32 = 30.0;
/// 이동 검사의 가장 작은 걸음 [mm]: 거리가 여유에 가까워도 이만큼은 건너뛴다. 그래서 자세 사이에서는 여유보다 이만큼
/// 더 가까워질 수 있다 (검사한 자세는 여유를 지킨다)
pub const MIN_STEP_MM: f32 = 0.5;
/// 이동 하나에서 검사할 자세 수의 상한 (막히지 않았는데 넘으면 알린다)
const MAX_SAMPLES: usize = 20_000;

/// 부딪힘 검사의 형상과 기준.
pub struct Model {
    parts: Vec<Part>,
    /// 링크마다 부품 번호
    by_link: Vec<Vec<usize>>,
    /// 관절마다 부모 링크 좌표계에서 본 원점 [mm]과 축
    origins: [Vector; 6],
    axes: [Vector; 6],
    planes: Vec<Plane>,
    rest: Option<Rest>,
    /// 관절 k가 돌 때 링크 k 이후의 점이 그 관절 원점에서 떨어진 거리의 상한 [mm]: 관절 각이 1 rad 바뀔 때 움직이는
    /// 거리의 상한이다 (이동 검사의 걸음)
    reach: [f32; 6],
    margin: f32,
}

struct Part {
    name: String,
    group: String,
    /// 설계의 틀 이름 ("j3", "j3_out" …)
    frame: String,
    link: usize,
    /// 링크 좌표계에서 본 부품 틀의 자리
    offset: Pose,
    /// 부품 틀 좌표의 볼록 조각과 부품 전체의 상자
    pieces: Arc<Vec<Piece>>,
    local_box: Aabb,
    /// 부품 틀 좌표의 꼭짓점 (안전 평면까지 거리)
    points: Arc<Vec<Vector>>,
}

/// 볼록 조각 하나와 그 상자 (부품 틀 좌표).
struct Piece {
    shape: ConvexPolyhedron,
    local_box: Aabb,
}

/// 조각마다 꼭짓점 (부품 틀 좌표, mm): 자료의 조각 파일 하나
pub type PiecePoints = Arc<Vec<Vec<Vector>>>;

/// 만든 부품 형상: 조각, 부품의 상자, 조각 껍질의 꼭짓점 (모두 부품 틀 좌표)
#[derive(Clone)]
struct Built {
    pieces: Arc<Vec<Piece>>,
    local_box: Aabb,
    points: Arc<Vec<Vector>>,
}

struct Plane {
    name: String,
    normal: Vector,
    offset: f32,
}

/// 앉고 떠나는 길: 쉬는 자세(`from`)와 쉬는 자세 위(`to`)를 잇는 관절 공간의 곧은 길.
struct Rest {
    from: [f64; 6],
    to: [f64; 6],
    within: f64,
    groups: [String; 2],
}

impl Rest {
    /// 관절 lo+1 ~ hi(링크 lo와 hi 사이)만 볼 때 자세가 길에서 within 안인가 (관절마다 가장 큰 차이).
    fn on_path(&self, q: &[f64; 6], lo: usize, hi: usize) -> bool {
        let joints = lo..hi;
        let d: Vec<f64> = joints.clone().map(|k| self.to[k] - self.from[k]).collect();
        let r: Vec<f64> = joints
            .clone()
            .map(|k| kinematics::wrap(q[k] - self.from[k]))
            .collect();
        let length = d.iter().map(|v| v * v).sum::<f64>();
        let t = if length > 0.0 {
            (r.iter().zip(&d).map(|(a, b)| a * b).sum::<f64>() / length).clamp(0.0, 1.0)
        } else {
            0.0
        };
        r.iter()
            .zip(&d)
            .all(|(a, b)| (a - t * b).abs() <= self.within)
    }
}

/// 가까운 것.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Contact {
    /// 부품 둘 (번호)
    Parts(usize, usize),
    /// 부품과 안전 평면 (부품 번호, 평면 번호)
    Plane(usize, usize),
}

/// 검사의 답: 가장 가까운 거리와 그 짝.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Nearest {
    /// [mm]. 겹치면 0, 창 안에 아무것도 없으면 창
    pub distance: f32,
    pub contact: Option<Contact>,
}

/// 이동 검사의 결과.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MoveReport {
    /// 검사한 자세 수
    pub samples: usize,
    /// 길 위에서 가장 가까웠던 것
    pub nearest: Nearest,
}

/// 이동 검사에서 막힌 자리.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Blocked {
    /// 길의 몫 (0은 시작, 1은 끝)
    pub fraction: f64,
    /// 그 자세의 관절 각 [rad]
    pub q: [f64; 6],
    pub nearest: Nearest,
}

/// 설계가 만든 부품 하나 (모델을 만드는 재료).
pub struct PartSpec {
    pub name: String,
    pub group: String,
    /// 설계의 틀 이름 ("j3", "j3_out" …)
    pub frame: String,
    pub link: usize,
    /// 링크 좌표계에서 본 부품 틀: 회전(행이 먼저)과 옮김 [mm]
    pub rotation: [[f64; 3]; 3],
    pub translation_mm: [f64; 3],
    /// 조각마다 꼭짓점 (부품 틀 좌표, mm)
    pub pieces: PiecePoints,
    /// 다 연 자리까지 옮겨 가는 양 [mm] (그리퍼 손가락)
    pub sweep: Option<Vector>,
}

/// 그리퍼 손가락처럼 열림 폭을 따라 움직이는 부품을 어디에 둘지.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Fingers {
    /// 닫은 자리부터 다 연 자리까지 쓸고 간 모양. 열림 폭을 몰라도, 움직이는 중에도 안전하다 (실행 명령)
    Swept,
    /// 이 열림 폭 [mm]의 자리 (설계 형상과 견주는 예제는 닫은 자리 0에 둔다)
    At(f64),
}

impl Model {
    /// 로봇 설정 옆의 `collision/`을 읽는다. 로봇 설정에 `[collision]`이 있어야 한다. 그리퍼는 로봇 설정에 있을 때만
    /// 넣고, 손가락은 `fingers`대로 둔다.
    pub fn load(dir: &Path, settings: &Settings, fingers: Fingers) -> Result<Self, String> {
        let collision = settings
            .collision
            .as_ref()
            .ok_or("로봇 설정에 [collision]이 없다 (0041)")?;
        let manifest_path = dir.join("parts.json");
        let text = fs::read_to_string(&manifest_path)
            .map_err(|e| format!("{}을 읽지 못했다: {e}", manifest_path.display()))?;
        let manifest: ManifestFile =
            serde_json::from_str(&text).map_err(|e| format!("{}: {e}", manifest_path.display()))?;
        let rest = match &collision.rest_contact {
            None => None,
            Some(contact) => Some((
                settings.pose("rest").ok_or("쉬는 자세(rest)가 없다")?,
                settings
                    .pose(&contact.to)
                    .ok_or_else(|| format!("자세 {}가 없다", contact.to))?,
            )),
        };
        let max_width = settings.gripper.as_ref().map(|gripper| gripper.max_width);
        let mut cache: HashMap<String, PiecePoints> = HashMap::new();
        let mut specs = Vec::new();
        for part in &manifest.parts {
            // 도구는 달린 것만 넣는다 (지금 도구는 그리퍼 하나다)
            let opening = match part.tool.as_str() {
                "" => None,
                "gripper" => match max_width {
                    None => continue,
                    Some(width) => Some(width),
                },
                other => return Err(format!("{}: 모르는 도구다: {other}", part.name)),
            };
            let frame = manifest
                .frames
                .get(&part.frame)
                .ok_or_else(|| format!("{}: 틀 {}이 없다", part.name, part.frame))?;
            let pieces = match cache.get(&part.key) {
                Some(pieces) => pieces.clone(),
                None => {
                    let pieces = Arc::new(read_pieces(dir, &part.key)?);
                    cache.insert(part.key.clone(), pieces.clone());
                    pieces
                }
            };
            let t = &frame.t_link_frame;
            let rotation = [
                [t[0][0], t[0][1], t[0][2]],
                [t[1][0], t[1][1], t[1][2]],
                [t[2][0], t[2][1], t[2][2]],
            ];
            let mut translation_mm = [t[0][3], t[1][3], t[2][3]];
            let mut sweep = None;
            if let (Some(v), Some(max)) = (part.slide, opening) {
                match fingers {
                    // 닫은 자리 조각과 다 연 자리 조각을 하나로 감싼다
                    Fingers::Swept => {
                        sweep = Some(Vector::new(
                            (v[0] * max) as f32,
                            (v[1] * max) as f32,
                            (v[2] * max) as f32,
                        ));
                    }
                    // 틀 좌표의 옮김을 링크 좌표로 돌려 더한다
                    Fingers::At(width) => {
                        let width = width.clamp(0.0, max);
                        for (k, row) in rotation.iter().enumerate() {
                            translation_mm[k] +=
                                width * (row[0] * v[0] + row[1] * v[1] + row[2] * v[2]);
                        }
                    }
                }
            }
            specs.push(PartSpec {
                name: part.name.clone(),
                group: part.group.clone(),
                frame: part.frame.clone(),
                link: frame.link,
                rotation,
                translation_mm,
                pieces,
                sweep,
            });
        }
        Self::new(&settings.kinematics, specs, collision, rest)
    }

    /// 부품과 기준으로 만든다. `rest`는 앉고 떠나는 길(쉬는 자세, 쉬는 자세 위)이고, 로봇 설정에 rest_contact가
    /// 있을 때 쓴다.
    pub fn new(
        kinematics: &Kinematics,
        specs: Vec<PartSpec>,
        settings: &CollisionSettings,
        rest: Option<(&[f64; 6], &[f64; 6])>,
    ) -> Result<Self, String> {
        let mut parts = Vec::with_capacity(specs.len());
        let mut by_link = vec![Vec::new(); LINKS];
        // 같은 조각을 쓰는 부품(관절마다 같은 서보, 나사)은 형상을 한 번만 만든다. 열쇠는 조각 목록의 자리이고,
        // 값에 그 목록을 같이 쥐어 자리가 다른 목록에 다시 쓰이지 않게 한다
        let mut shapes: HashMap<usize, (PiecePoints, Built)> = HashMap::new();
        for spec in specs {
            if spec.link >= LINKS {
                return Err(format!("{}: 링크 {}는 없다", spec.name, spec.link));
            }
            let shared = spec
                .sweep
                .is_none()
                .then_some(Arc::as_ptr(&spec.pieces) as usize);
            let built = match shared.and_then(|key| shapes.get(&key)) {
                Some((_, built)) => built.clone(),
                None => {
                    let built = build_shape(&spec)?;
                    if let Some(key) = shared {
                        shapes.insert(key, (spec.pieces.clone(), built.clone()));
                    }
                    built
                }
            };
            by_link[spec.link].push(parts.len());
            parts.push(Part {
                offset: pose_of(&spec.rotation, spec.translation_mm),
                name: spec.name,
                group: spec.group,
                frame: spec.frame,
                link: spec.link,
                pieces: built.pieces,
                local_box: built.local_box,
                points: built.points,
            });
        }
        let origins = kinematics.origins().map(|o| {
            Vector::new(
                (o[0] * 1e3) as f32,
                (o[1] * 1e3) as f32,
                (o[2] * 1e3) as f32,
            )
        });
        let axes = kinematics
            .axes()
            .map(|a| Vector::new(a[0] as f32, a[1] as f32, a[2] as f32));
        // 링크마다 원점에서 가장 먼 점 [mm]
        let mut radius = [0.0f32; LINKS];
        for part in &parts {
            let far = part
                .points
                .iter()
                .map(|p| (part.offset * *p).length())
                .fold(0.0f32, f32::max);
            radius[part.link] = radius[part.link].max(far);
        }
        let mut reach = [0.0f32; 6];
        for (k, slot) in reach.iter_mut().enumerate() {
            // 관절 k+1의 원점(링크 k+1의 원점)에서 링크 l의 점까지: 사이 관절 원점의 길이 합 + 링크 l의 반지름
            let mut chain = 0.0f32;
            let mut best = 0.0f32;
            for l in (k + 1)..LINKS {
                if l > k + 1 {
                    chain += origins[l - 1].length();
                }
                best = best.max(chain + radius[l]);
            }
            *slot = best;
        }
        let planes = settings
            .planes
            .iter()
            .map(|p| Plane {
                name: p.name.clone(),
                normal: Vector::new(p.normal[0] as f32, p.normal[1] as f32, p.normal[2] as f32),
                offset: p.offset_mm as f32,
            })
            .collect();
        let rest = match (&settings.rest_contact, rest) {
            (Some(contact), Some((from, to))) => {
                for group in &contact.groups {
                    if !parts.iter().any(|part| &part.group == group) {
                        return Err(format!(
                            "collision.rest_contact: 형상 자료에 묶음 {group}이 없다"
                        ));
                    }
                }
                Some(Rest {
                    from: *from,
                    to: *to,
                    within: contact.within,
                    groups: contact.groups.clone(),
                })
            }
            (Some(_), None) => {
                return Err("collision.rest_contact: 쉬는 자세(rest)가 없다".to_owned());
            }
            (None, _) => None,
        };
        Ok(Self {
            parts,
            by_link,
            origins,
            axes,
            planes,
            rest,
            reach,
            margin: settings.margin_mm as f32,
        })
    }

    /// 여유 [mm].
    pub fn margin(&self) -> f32 {
        self.margin
    }

    /// 부품 수.
    pub fn len(&self) -> usize {
        self.parts.len()
    }

    pub fn is_empty(&self) -> bool {
        self.parts.is_empty()
    }

    /// 부품 이름.
    pub fn part_name(&self, index: usize) -> &str {
        &self.parts[index].name
    }

    /// 사람이 볼 짝: "위팔/위팔 ↔ 책상 (2.1 mm)".
    pub fn describe(&self, nearest: &Nearest) -> String {
        let what = match nearest.contact {
            None => return format!("{} mm 안에 없다", nearest.distance),
            Some(Contact::Parts(a, b)) => {
                format!("{} ↔ {}", self.parts[a].name, self.parts[b].name)
            }
            Some(Contact::Plane(a, p)) => {
                format!("{} ↔ 안전 평면 {}", self.parts[a].name, self.planes[p].name)
            }
        };
        if nearest.distance <= 0.0 {
            format!("{what} (겹친다)")
        } else {
            format!("{what} ({:.1} mm)", nearest.distance)
        }
    }

    /// 여유 안에 드는 것이 있으면 그 하나 (찾는 대로 멈춘다).
    pub fn first_within(&self, q: &[f64; 6], margin: f32) -> Option<Nearest> {
        let placed = self.place(q);
        let mut found = None;
        self.visit(&placed, margin, |distance, contact| {
            found = Some(Nearest {
                distance,
                contact: Some(contact),
            });
            Visit::Stop
        });
        found
    }

    /// 창 [mm] 안에서 가장 가까운 것. 창 안에 아무것도 없으면 거리는 창이다.
    pub fn nearest(&self, q: &[f64; 6], window: f32) -> Nearest {
        let placed = self.place(q);
        let mut best = Nearest {
            distance: window,
            contact: None,
        };
        self.visit(&placed, window, |distance, contact| {
            if distance < best.distance {
                best = Nearest {
                    distance,
                    contact: Some(contact),
                };
            }
            if distance <= 0.0 {
                Visit::Stop
            } else {
                Visit::Shrink(best.distance)
            }
        });
        best
    }

    /// 창 [mm] 안에 든 것을 모두: (무엇, 거리). 검사하는 짝과 평면만이다 (0041의 숫자를 다시 만드는 예제가 메시
    /// 거리와 견준다).
    pub fn all_within(&self, q: &[f64; 6], window: f32) -> Vec<(Contact, f32)> {
        let placed = self.place(q);
        let mut found = Vec::new();
        self.visit(&placed, window, |distance, contact| {
            found.push((contact, distance));
            Visit::Shrink(window)
        });
        found
    }

    /// 관절 하나(1 ~ 6)를 사이에 둔 이웃 링크의 부품끼리 창 안에서 가장 가까운 것. 실행 중에는 보지 않는 짝이다:
    /// 둘 사이 거리는 그 관절 각 하나에만 달려서, 관절 범위 안에서 떨어져 있는지를 시험이 이것으로 본다 (0041의
    /// 결정 3). 관절 속 부품끼리(같은 묶음에서 그 관절의 서보 좌표에 그린 부품: 서보, 혼, 출력 원판)는 뺀다.
    pub fn neighbours(&self, q: &[f64; 6], joint: usize, window: f32) -> Nearest {
        let placed = self.place(q);
        let inside = [format!("j{joint}"), format!("j{joint}_out")];
        let mut best = Nearest {
            distance: window,
            contact: None,
        };
        for &i in &self.by_link[joint - 1] {
            for &j in &self.by_link[joint] {
                let (a, b) = (&self.parts[i], &self.parts[j]);
                if a.group == b.group && inside.contains(&a.frame) && inside.contains(&b.frame) {
                    continue;
                }
                if gap(&placed.boxes[i], &placed.boxes[j]) >= best.distance {
                    continue;
                }
                let Some(distance) = self.part_distance(i, j, &placed, best.distance) else {
                    continue;
                };
                if distance < best.distance {
                    best = Nearest {
                        distance,
                        contact: Some(Contact::Parts(i, j)),
                    };
                }
            }
        }
        best
    }

    /// 관절 공간의 곧은 길(관절 이동, [`crate::plan::move_joints`])을 검사한다. 길 위의 자세를 거리를 보며
    /// 건너뛴다: 거리가 d이면 어느 점도 d - 여유(적어도 [`MIN_STEP_MM`])보다 더 움직이지 않는 만큼 간다. 관절이
    /// 1 rad 돌 때 점이 움직이는 거리의 상한(링크의 길이와 반지름)으로 걸음을 정해서, 두 자세 사이에서 여유보다
    /// [`MIN_STEP_MM`] 넘게 가까워지는 일이 없다.
    pub fn check_move(&self, from: &[f64; 6], to: &[f64; 6]) -> Result<MoveReport, Blocked> {
        let rate: f32 = (0..6)
            .map(|k| self.reach[k] * (to[k] - from[k]).abs() as f32)
            .sum();
        let mut s = 0.0f64;
        let mut samples = 0;
        let mut closest = Nearest {
            distance: WINDOW_MM,
            contact: None,
        };
        loop {
            let q: [f64; 6] = std::array::from_fn(|k| from[k] + (to[k] - from[k]) * s);
            let nearest = self.nearest(&q, WINDOW_MM);
            samples += 1;
            if nearest.distance < closest.distance {
                closest = nearest;
            }
            if nearest.distance < self.margin {
                return Err(Blocked {
                    fraction: s,
                    q,
                    nearest,
                });
            }
            if s >= 1.0 || rate <= 0.0 {
                break;
            }
            if samples >= MAX_SAMPLES {
                return Err(Blocked {
                    fraction: s,
                    q,
                    nearest: Nearest {
                        distance: nearest.distance,
                        contact: None,
                    },
                });
            }
            let step = (nearest.distance - self.margin).max(MIN_STEP_MM);
            s = (s + f64::from(step / rate)).min(1.0);
        }
        Ok(MoveReport {
            samples,
            nearest: closest,
        })
    }

    /// 자세의 부품 자리와 상자.
    fn place(&self, q: &[f64; 6]) -> Placed {
        let frames = links_mm(q, &self.origins, &self.axes);
        let world: Vec<Pose> = self
            .parts
            .iter()
            .map(|part| frames[part.link] * part.offset)
            .collect();
        let boxes: Vec<Aabb> = self
            .parts
            .iter()
            .zip(&world)
            .map(|(part, pose)| part.local_box.transform_by(pose))
            .collect();
        let link_boxes = self
            .by_link
            .iter()
            .map(|ids| {
                ids.iter()
                    .fold(Aabb::new_invalid(), |acc, &i| merge(&acc, &boxes[i]))
            })
            .collect();
        Placed {
            world,
            boxes,
            link_boxes,
            q: *q,
        }
    }

    /// 받침에 앉는 짝이고, 두 링크 사이 관절이 앉고 떠나는 길 위에 있다.
    fn allowed(&self, a: usize, b: usize, placed: &Placed) -> bool {
        let Some(rest) = &self.rest else {
            return false;
        };
        let (pa, pb) = (&self.parts[a], &self.parts[b]);
        let groups = (pa.group == rest.groups[0] && pb.group == rest.groups[1])
            || (pa.group == rest.groups[1] && pb.group == rest.groups[0]);
        let (lo, hi) = (pa.link.min(pb.link), pa.link.max(pb.link));
        groups && rest.on_path(&placed.q, lo, hi)
    }

    /// 두 부품 사이의 거리 [mm]가 창보다 작으면 그 거리. 볼록 조각 짝마다 묻는다: 조각의 상자 틈이 지금까지 찾은
    /// 거리보다 먼 짝은 건너뛰고, 볼록 조각끼리만 parry3d에 묻는다. 조각을 묶음(compound)으로 한 번에 물으면 거의
    /// 닿은 자리에서 가장 가까운 조각을 놓쳐 거리를 크게 낸 일이 있었다 (parry3d 0.28, 2026-11-27: 메시 0.44 mm인
    /// 짝을 1.77 mm로 냈다. 같은 조각 짝을 하나씩 물으면 맞았다).
    fn part_distance(&self, i: usize, j: usize, placed: &Placed, window: f32) -> Option<f32> {
        let (pi, pj) = (&self.parts[i], &self.parts[j]);
        let (wi, wj) = (&placed.world[i], &placed.world[j]);
        let boxes_j: Vec<Aabb> = pj
            .pieces
            .iter()
            .map(|p| p.local_box.transform_by(wj))
            .collect();
        let mut best = window;
        let mut found = false;
        for a in pi.pieces.iter() {
            let box_a = a.local_box.transform_by(wi);
            if gap(&box_a, &placed.boxes[j]) >= best {
                continue;
            }
            for (b, box_b) in pj.pieces.iter().zip(&boxes_j) {
                if gap(&box_a, box_b) >= best {
                    continue;
                }
                let distance = match query::closest_points(wi, &a.shape, wj, &b.shape, best) {
                    // 볼록 다면체끼리는 늘 된다. 안 되면 겹친 것으로 본다 (안전한 쪽)
                    Ok(ClosestPoints::Intersecting) | Err(_) => return Some(0.0),
                    Ok(ClosestPoints::WithinMargin(p1, p2)) => (p1 - p2).length(),
                    Ok(ClosestPoints::Disjoint) => continue,
                };
                if distance < best {
                    best = distance;
                    found = true;
                }
            }
        }
        found.then_some(best)
    }

    /// 창 안의 짝과 평면마다 거리를 재 알린다. 알림이 Shrink를 돌려주면 그 뒤로는 그보다 가까운 것만 찾는다.
    fn visit(&self, placed: &Placed, window: f32, mut report: impl FnMut(f32, Contact) -> Visit) {
        let mut window = window;
        for la in 0..LINKS {
            for lb in (la + 2)..LINKS {
                if gap(&placed.link_boxes[la], &placed.link_boxes[lb]) >= window {
                    continue;
                }
                for &i in &self.by_link[la] {
                    if gap(&placed.boxes[i], &placed.link_boxes[lb]) >= window {
                        continue;
                    }
                    for &j in &self.by_link[lb] {
                        if gap(&placed.boxes[i], &placed.boxes[j]) >= window
                            || self.allowed(i, j, placed)
                        {
                            continue;
                        }
                        let Some(distance) = self.part_distance(i, j, placed, window) else {
                            continue;
                        };
                        if distance < window {
                            match report(distance, Contact::Parts(i, j)) {
                                Visit::Stop => return,
                                Visit::Shrink(w) => window = w,
                            }
                        }
                    }
                }
            }
        }
        for (k, plane) in self.planes.iter().enumerate() {
            for link in 2..LINKS {
                if box_above(&placed.link_boxes[link], plane) >= window {
                    continue;
                }
                for &i in &self.by_link[link] {
                    if box_above(&placed.boxes[i], plane) >= window {
                        continue;
                    }
                    let distance = self.plane_distance(i, plane, placed);
                    if distance < window {
                        match report(distance.max(0.0), Contact::Plane(i, k)) {
                            Visit::Stop => return,
                            Visit::Shrink(w) => window = w,
                        }
                    }
                }
            }
        }
    }

    /// 부품이 평면에서 팔 쪽으로 떨어진 거리 [mm] (넘었으면 0 이하). 평면을 부품 틀로 옮겨 꼭짓점마다 잰다:
    /// n·(R p + t) - d = (Rᵀn)·p + n·t - d. 볼록 조각의 꼭짓점이라 조각까지의 거리와 같다.
    fn plane_distance(&self, i: usize, plane: &Plane, placed: &Placed) -> f32 {
        let pose = &placed.world[i];
        let local = pose.rotation.inverse() * plane.normal;
        let shift = plane.normal.dot(pose.translation) - plane.offset;
        self.parts[i]
            .points
            .iter()
            .map(|p| local.dot(*p))
            .fold(f32::INFINITY, f32::min)
            + shift
    }

    /// 링크 하나의 부품이 안전 평면에 가장 가까운 것. 실행 중에는 링크 2 이상만 본다: 링크 1(J1 몸통과 선반)이 J1
    /// 범위 안에서 평면에서 떨어져 있는지를 시험이 이것으로 본다.
    pub fn plane_clearance(&self, q: &[f64; 6], link: usize) -> Nearest {
        let placed = self.place(q);
        let mut best = Nearest {
            distance: f32::INFINITY,
            contact: None,
        };
        for (k, plane) in self.planes.iter().enumerate() {
            for &i in &self.by_link[link] {
                let distance = self.plane_distance(i, plane, &placed).max(0.0);
                if distance < best.distance {
                    best = Nearest {
                        distance,
                        contact: Some(Contact::Plane(i, k)),
                    };
                }
            }
        }
        best
    }
}

enum Visit {
    Stop,
    Shrink(f32),
}

struct Placed {
    world: Vec<Pose>,
    boxes: Vec<Aabb>,
    link_boxes: Vec<Aabb>,
    /// 관절 각 [rad]
    q: [f64; 6],
}

/// 부품 하나의 형상: 조각마다 볼록 껍질과 상자 (쓸고 가는 부품은 두 자리 꼭짓점의 껍질), 부품의 상자, 꼭짓점.
fn build_shape(spec: &PartSpec) -> Result<Built, String> {
    let mut pieces = Vec::new();
    let mut points = Vec::new();
    for piece in spec.pieces.iter() {
        let mut hull_points = piece.clone();
        if let Some(sweep) = spec.sweep {
            hull_points.extend(piece.iter().map(|p| *p + sweep));
        }
        let hull = ConvexPolyhedron::from_convex_hull(&hull_points)
            .ok_or_else(|| format!("{}: 조각의 볼록 껍질을 만들지 못했다", spec.name))?;
        let local_box = bounds(hull.points());
        points.extend(hull.points().iter().copied());
        pieces.push(Piece {
            shape: hull,
            local_box,
        });
    }
    if pieces.is_empty() {
        return Err(format!("{}: 조각이 없다", spec.name));
    }
    Ok(Built {
        pieces: Arc::new(pieces),
        local_box: bounds(&points),
        points: Arc::new(points),
    })
}

fn bounds(points: &[Vector]) -> Aabb {
    points.iter().fold(Aabb::new_invalid(), |acc, p| {
        Aabb::new(acc.mins.min(*p), acc.maxs.max(*p))
    })
}

/// 회전(행이 먼저)과 옮김 [mm]의 자리.
fn pose_of(rotation: &[[f64; 3]; 3], translation_mm: [f64; 3]) -> Pose {
    let col = |j: usize| {
        Vector::new(
            rotation[0][j] as f32,
            rotation[1][j] as f32,
            rotation[2][j] as f32,
        )
    };
    Pose::from_parts(
        Vector::new(
            translation_mm[0] as f32,
            translation_mm[1] as f32,
            translation_mm[2] as f32,
        ),
        Rotation::from_mat3(&Mat3::from_cols(col(0), col(1), col(2))),
    )
}

/// 관절 각에서 링크 0 ~ 6의 자리 [mm]. 정기구학([`Kinematics::link_frames`])과 같은 셈을 f32로 한다.
fn links_mm(q: &[f64; 6], origins: &[Vector; 6], axes: &[Vector; 6]) -> [Pose; LINKS] {
    let mut out = [Pose::IDENTITY; LINKS];
    let mut pose = Pose::IDENTITY;
    for k in 0..6 {
        pose *= Pose::from_parts(origins[k], Rotation::from_axis_angle(axes[k], q[k] as f32));
        out[k + 1] = pose;
    }
    out
}

fn merge(a: &Aabb, b: &Aabb) -> Aabb {
    Aabb::new(a.mins.min(b.mins), a.maxs.max(b.maxs))
}

/// 두 상자 사이의 틈 [mm] (겹치면 0).
fn gap(a: &Aabb, b: &Aabb) -> f32 {
    let d = (a.mins - b.maxs).max(b.mins - a.maxs).max(Vector::ZERO);
    d.length()
}

/// 상자가 평면에서 팔 쪽으로 떨어진 거리의 하한 [mm] (넘었으면 0 이하).
fn box_above(b: &Aabb, plane: &Plane) -> f32 {
    let n = plane.normal;
    // 법선 쪽으로 가장 낮은 꼭짓점
    let corner = Vector::new(
        if n.x >= 0.0 { b.mins.x } else { b.maxs.x },
        if n.y >= 0.0 { b.mins.y } else { b.maxs.y },
        if n.z >= 0.0 { b.mins.z } else { b.maxs.z },
    );
    n.dot(corner) - plane.offset
}

#[derive(Deserialize)]
struct ManifestFile {
    frames: BTreeMap<String, FrameFile>,
    parts: Vec<PartFile>,
}

#[derive(Deserialize)]
struct FrameFile {
    link: usize,
    #[serde(rename = "T_link_frame")]
    t_link_frame: [[f64; 4]; 4],
}

#[derive(Deserialize)]
struct PartFile {
    name: String,
    frame: String,
    group: String,
    tool: String,
    slide: Option<[f64; 3]>,
    key: String,
}

#[derive(Deserialize)]
struct PiecesFile {
    pieces: Vec<Vec<f32>>,
}

/// 조각 파일 하나: 조각마다 꼭짓점 [mm].
fn read_pieces(dir: &Path, key: &str) -> Result<Vec<Vec<Vector>>, String> {
    let path = dir.join("pieces").join(format!("{key}.json"));
    let text =
        fs::read_to_string(&path).map_err(|e| format!("{}을 읽지 못했다: {e}", path.display()))?;
    let file: PiecesFile =
        serde_json::from_str(&text).map_err(|e| format!("{}: {e}", path.display()))?;
    file.pieces
        .iter()
        .map(|flat| {
            if flat.len() < 12 || flat.len() % 3 != 0 {
                return Err(format!("{}: 조각의 꼭짓점이 모자라다", path.display()));
            }
            Ok(flat
                .as_chunks::<3>()
                .0
                .iter()
                .map(|c| Vector::new(c[0], c[1], c[2]))
                .collect())
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::settings::{RestContact, SafetyPlane};
    use std::f64::consts::PI;
    use std::sync::Arc;

    /// 지금 팔과 같은 꼴의 기구학 (로봇 설정의 [kinematics])
    fn arm() -> Kinematics {
        let mm = |x: f64, y: f64, z: f64| [x * 1e-3, y * 1e-3, z * 1e-3];
        Kinematics::new(
            [
                mm(0.0, 0.0, 0.0),
                mm(0.0, 0.0, 160.0),
                mm(0.0, 0.0, 220.0),
                mm(0.0, 0.0, 180.0),
                mm(0.0, 79.21, 0.0),
                mm(0.0, 0.0, 70.31),
            ],
            [
                [0.0, 0.0, 1.0],
                [0.0, 1.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
                [0.0, 1.0, 0.0],
            ],
            [[0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]],
            mm(0.0, 32.6, 0.0),
        )
        .expect("팔")
    }

    /// 가운데가 center [mm]이고 반폭이 half인 상자 하나.
    fn cube(name: &str, group: &str, link: usize, center: [f32; 3], half: f32) -> PartSpec {
        let mut points = Vec::new();
        for x in [-half, half] {
            for y in [-half, half] {
                for z in [-half, half] {
                    points.push(Vector::new(center[0] + x, center[1] + y, center[2] + z));
                }
            }
        }
        PartSpec {
            name: name.to_owned(),
            group: group.to_owned(),
            frame: String::new(),
            link,
            rotation: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            translation_mm: [0.0; 3],
            pieces: Arc::new(vec![points]),
            sweep: None,
        }
    }

    fn settings(planes: Vec<SafetyPlane>) -> CollisionSettings {
        CollisionSettings {
            margin_mm: 3.0,
            planes,
            rest_contact: None,
        }
    }

    fn desk() -> SafetyPlane {
        SafetyPlane {
            name: "책상".to_owned(),
            normal: [0.0, 0.0, 1.0],
            offset_mm: 0.0,
        }
    }

    #[test]
    fn links_follow_the_kinematics() {
        let k = arm();
        let origins = k.origins().map(|o| {
            Vector::new(
                (o[0] * 1e3) as f32,
                (o[1] * 1e3) as f32,
                (o[2] * 1e3) as f32,
            )
        });
        let axes = k
            .axes()
            .map(|a| Vector::new(a[0] as f32, a[1] as f32, a[2] as f32));
        let q = [0.3, -0.7, 1.1, 2.9, -0.4, 0.8];
        let frames = links_mm(&q, &origins, &axes);
        for (f, want) in frames[1..].iter().zip(k.link_frames(&q)) {
            for i in 0..3 {
                assert!((f.translation[i] - (want.position[i] * 1e3) as f32).abs() < 1e-3);
            }
            let m = Mat3::from_quat(f.rotation);
            for (i, row) in want.rotation.iter().enumerate() {
                for (j, v) in row.iter().enumerate() {
                    assert!((m.col(j)[i] - *v as f32).abs() < 1e-5);
                }
            }
        }
    }

    #[test]
    fn far_links_are_checked_and_neighbours_are_not() {
        // 베이스(링크 0)의 상자와 위팔(링크 2)의 상자: 영점에서 위팔 상자는 어깨 위 J2 축 높이(160 mm)에 있다
        let base = cube("베이스/몸통", "베이스", 0, [60.0, 0.0, 100.0], 10.0);
        let shoulder = cube("어깨/집", "어깨", 1, [60.0, 0.0, 100.0], 10.0); // 베이스와 겹치지만 이웃이다
        let upper = cube("위팔/위팔", "위팔", 2, [0.0, 0.0, 50.0], 10.0);
        let model =
            Model::new(&arm(), vec![base, shoulder, upper], &settings(vec![]), None).expect("모델");
        // 영점: 위팔 상자 가운데 (0, 0, 210), 베이스 상자 가운데 (60, 0, 100) → 이웃 아닌 짝은 멀다
        let n = model.nearest(&[0.0; 6], WINDOW_MM);
        assert_eq!(n.contact, None);
        // J2를 앞으로 숙이면 위팔 상자가 (50 sin, 0, 160 + 50 cos)로 내려온다. 90°에서는 (50, 0, 160):
        // x는 겹치고 z 틈은 150 - 110 = 40 > 창
        let q = [0.0, 90f64.to_radians(), 0.0, 0.0, 0.0, 0.0];
        assert_eq!(model.nearest(&q, WINDOW_MM).contact, None);
        // 120°에서는 가운데가 (43.3, 0, 135)이고 상자도 120° 돌아서, 아래 모서리가 베이스 상자 윗면(z 110)에서
        // 15 mm(돌지 않은 상자)보다 가깝다
        let q = [0.0, 120f64.to_radians(), 0.0, 0.0, 0.0, 0.0];
        let n = model.nearest(&q, WINDOW_MM);
        assert_eq!(n.contact, Some(Contact::Parts(0, 2)));
        assert!(n.distance > 5.0 && n.distance < 15.0, "{n:?}");
        assert!(model.first_within(&q, 3.0).is_none());
        assert!(model.first_within(&q, n.distance + 0.5).is_some());
    }

    #[test]
    fn desk_plane_sees_links_two_and_beyond() {
        // 링크 1의 상자는 책상 밑으로 내려가 있어도 보지 않는다 (J1 둘레로만 돈다)
        let low = cube("어깨/발", "어깨", 1, [0.0, 0.0, -5.0], 2.0);
        let upper = cube("위팔/끝", "위팔", 2, [0.0, 0.0, 200.0], 5.0);
        let model =
            Model::new(&arm(), vec![low, upper], &settings(vec![desk()]), None).expect("모델");
        assert_eq!(model.nearest(&[0.0; 6], WINDOW_MM).contact, None);
        // 위팔을 앞으로 눕혀 끝이 책상 아래로: z = 160 + 200 cos(100°) = 125, 숙임이 더 크면 내려간다
        let q = [0.0, 150f64.to_radians(), 0.0, 0.0, 0.0, 0.0];
        let n = model.nearest(&q, WINDOW_MM);
        assert_eq!(n.contact, Some(Contact::Plane(1, 0)));
        assert_eq!(n.distance, 0.0);
        assert!(model.describe(&n).contains("안전 평면 책상"));
    }

    #[test]
    fn rest_contact_is_ignored_only_on_the_path_off_the_shelf() {
        // 아래팔 상자는 링크 3의 원점(J3)에 있다: J2 = θ에서 (220 sin θ, 0, 160 + 220 cos θ). 선반 상자를 J2 = 100°의
        // J3 바로 밑(20 mm 아래)에 두면 그 둘레에서 닿는다. J2를 줄이면 J3이 올라간다 (떠나는 길)
        let t = 100f32.to_radians();
        let under = [220.0 * t.sin(), 0.0, 160.0 + 220.0 * t.cos() - 20.0];
        let shelf = || cube("선반/선반", "선반", 1, under, 10.0);
        let fore = || cube("아래팔/아래팔", "아래팔", 3, [0.0, 0.0, 0.0], 10.0);
        let mut s = settings(vec![]);
        s.rest_contact = Some(RestContact {
            groups: ["아래팔".to_owned(), "선반".to_owned()],
            to: "rest_above".to_owned(),
            within: 2f64.to_radians(),
        });
        let model0 =
            Model::new(&arm(), vec![shelf(), fore()], &settings(vec![]), None).expect("모델");
        // 닿는 J2 각을 찾아 쉬는 자세로 둔다
        let mut rest = [0.0; 6];
        let mut found = None;
        for tenth in 800..1200 {
            rest[1] = (f64::from(tenth) / 10.0).to_radians();
            if model0.first_within(&rest, 0.0001).is_some() {
                found = Some(rest);
                break;
            }
        }
        let rest = found.expect("닿는 각이 있다");
        let mut above = rest;
        above[1] -= 15f64.to_radians();
        let (shelf, fore) = (shelf(), fore());
        let model = Model::new(&arm(), vec![shelf, fore], &s, Some((&rest, &above))).expect("모델");
        assert!(
            model.first_within(&rest, 3.0).is_none(),
            "쉬는 자세는 봐준다"
        );
        assert!(
            model.first_within(&above, 3.0).is_none(),
            "쉬는 자세 위는 떨어져 있다"
        );
        // 떠나는 길과 읽은 각의 어긋남(폭 2° 안)은 봐준다
        let mut off = rest;
        off[1] -= 0.4f64.to_radians();
        off[2] += 1.5f64.to_radians();
        assert!(model.first_within(&off, 3.0).is_none());
        assert!(model.check_move(&rest, &above).is_ok());
        // 길에서 벗어나면 (J3을 3° 돌림) 다시 본다: 아래팔 상자는 J3 원점에 있어 닿은 채다
        let mut away = rest;
        away[2] += 3f64.to_radians();
        assert!(model.first_within(&away, 3.0).is_some());
        assert!(model.check_move(&rest, &away).is_err());
        // J1은 두 링크를 같이 돌려서 상관없다: 둘 사이 관절(J2, J3)만 본다
        let mut turned = rest;
        turned[0] += 90f64.to_radians();
        assert!(model.first_within(&turned, 3.0).is_none());
    }

    #[test]
    fn a_move_through_an_obstacle_is_blocked_between_clear_ends() {
        // 베이스 앞에 기둥, 위팔 끝 상자가 J1을 돌며 그 기둥을 지나간다: 시작과 끝은 비어 있다
        let post = cube("베이스/기둥", "베이스", 0, [0.0, 250.0, 300.0], 15.0);
        let tip = cube("위팔/끝", "위팔", 2, [0.0, 0.0, 200.0], 10.0);
        let model = Model::new(&arm(), vec![post, tip], &settings(vec![]), None).expect("모델");
        // J2 = 60°: 끝은 J1 축에서 200 sin 60 = 173 mm, 높이 160 + 100 = 260 mm
        let q = |j1: f64| [j1.to_radians(), 60f64.to_radians(), 0.0, 0.0, 0.0, 0.0];
        // 기둥을 위팔 끝 원 위(반지름 173, 높이 260)의 J1 = 90° 자리로 옮긴 모델을 다시 만든다
        let post = cube("베이스/기둥", "베이스", 0, [0.0, 173.2, 260.0], 15.0);
        let tip = cube("위팔/끝", "위팔", 2, [0.0, 0.0, 200.0], 10.0);
        let model2 = Model::new(&arm(), vec![post, tip], &settings(vec![]), None).expect("모델");
        let _ = model;
        assert!(model2.first_within(&q(0.0), 3.0).is_none());
        assert!(model2.first_within(&q(180.0), 3.0).is_none());
        let blocked = model2
            .check_move(&q(0.0), &q(180.0))
            .expect_err("가운데에서 막힌다");
        assert!(
            blocked.fraction > 0.2 && blocked.fraction < 0.8,
            "{blocked:?}"
        );
        // 기둥을 피하는 길(반대로 돌기)은 지난다
        let report = model2.check_move(&q(0.0), &q(-150.0)).expect("지난다");
        assert!(report.samples > 2);
        assert!(report.nearest.distance >= 3.0);
    }

    #[test]
    fn sweep_covers_the_whole_finger_travel() {
        // 손가락(링크 6)이 열리며 +x로 30 mm 간다: 쓸고 간 모양이라 다 연 자리의 장애물에도 걸린다
        let mut finger = cube("그리퍼/손가락", "그리퍼", 6, [0.0, 0.0, 0.0], 2.0);
        finger.sweep = Some(Vector::new(30.0, 0.0, 0.0));
        let model = Model::new(&arm(), vec![finger], &settings(vec![]), None).expect("모델");
        let part = &model.parts[0];
        assert!((part.local_box.maxs.x - 32.0).abs() < 1e-4);
        assert!((part.local_box.mins.x + 2.0).abs() < 1e-4);
    }

    #[test]
    fn reach_bounds_how_far_points_move() {
        let tip = cube("손목 2/끝", "손목 2", 6, [0.0, 0.0, 100.0], 1.0);
        let model = Model::new(&arm(), vec![tip], &settings(vec![]), None).expect("모델");
        // 임의의 작은 관절 걸음에서 점이 움직인 거리 ≤ Σ reach |Δq|
        let origins = model.origins;
        let axes = model.axes;
        let p = Vector::new(0.0, 0.0, 100.0);
        let a = [0.2, -0.4, 0.9, 2.5, 0.3, -1.0];
        for k in 0..6 {
            let mut b = a;
            b[k] += 0.01;
            let pa = links_mm(&a, &origins, &axes)[6] * p;
            let pb = links_mm(&b, &origins, &axes)[6] * p;
            let moved = (pa - pb).length();
            assert!(
                moved <= model.reach[k] * 0.01 + 1e-3,
                "J{} {moved} > {}",
                k + 1,
                model.reach[k] * 0.01
            );
        }
        let _ = PI;
    }
}
