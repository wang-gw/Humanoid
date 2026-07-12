# 126. Flat-Foot Penalty — Ankle Twist Fix (E2E Walking v2)

## 목적

doc 124의 E2E walking 정책(v1) 영상에서 **발목이 틀어져 보이는** 문제를 해결한다.
mesh 정렬(doc 125)은 정상이므로 원인은 학습된 정책의 비대칭 보행에 있다.

## 진단 (직접 이미지 캡처 + 정량 측정)

발바닥 기울기 = 발 body의 up축과 world +Z 사이 각도 (`xmat[2,2]` → arccos). 0° = 평평.

**1. Mesh는 정상**: 모든 관절을 0(중립)으로 두면 양발이 완전히 평평하게 collision box와 정렬됨. → 시각 아티팩트/mesh 버그 아님.

**2. 걷기 중 발목 각도 (v1 정책)**:

| step | L pitch | R pitch | R roll | L 기울기 | R 기울기 |
|---|---|---|---|---|---|
| 100 | 19° | -8° | -12° | 0.4° ✓ | 20.7° ✗ |
| 200 | 17° | -8° | -13° | 0.2° ✓ | 20.0° ✗ |
| 400 | 19° | -9° | -13° | 0.3° ✓ | 21.2° ✗ |

**결론**: 왼발은 접지 시 평평(0~4°)한데, **오른발은 접지 중에도 `right_ankle_roll ≈ -13°`로 고정되어 발 안쪽 모서리로 보행**(발바닥 20~24° 기울기 지속). doc 124가 언급한 좌우 비대칭과 동일한 로컬 옵티멈.

**근본 원인**:
1. seed 서있는 자세부터 좌우 발목 target 비대칭 (dynamic pose search 산출물)
2. 보상 함수에 flat-foot / ankle-roll 페널티 없음 → 모서리 보행을 막을 유인 부재
3. 좌우 대칭 유도 부재

## 수정: Flat-Foot Penalty

**파일**: `envs/urdf_f_env.py`

```python
def _foot_sole_tilt(self, body_id):
    cos_up = float(self.data.xmat[body_id].reshape(3, 3)[2, 2])
    return float(np.arccos(np.clip(cos_up, -1.0, 1.0)))  # 0 = flat

def _flat_foot_penalty(self, left_in_contact, right_in_contact):
    # 접지(contact)한 발이 deadzone(5°) 초과로 기울면 페널티
    pen = 0.0
    if left_in_contact:
        pen += clip((tilt_L - 0.087) / 0.30, 0, 1)
    if right_in_contact:
        pen += clip((tilt_R - 0.087) / 0.30, 0, 1)
    return self.flat_foot_penalty_weight * pen
```

**핵심 설계 결정 — contact 게이트 (clearance 아님)**:
발이 모서리로 기울면 collision box 중심 높이가 올라가 `clearance`(중심−half height) 지표가
"공중"으로 오판한다. 즉 clearance 게이트는 정작 잡아야 할 모서리 보행을 놓친다.
따라서 **실제 접촉 수(`stats["contacts"] > 0`)** 로 게이트한다.

walking task 분기에만 적용, 기본 weight=0 → 다른 task/기존 policy 무영향.

**파라미터** (`__init__`):

| 파라미터 | 기본값 | 의미 |
|---|---:|---|
| `flat_foot_penalty_weight` | 0.0 | 페널티 계수 (v2 학습 시 1.5) |
| `flat_foot_deadzone_rad` | 0.087 (5°) | 이 각도까지는 페널티 없음 |
| `flat_foot_scale_rad` | 0.30 (~17°) | 페널티 포화 스케일 |

## 학습: e2e_walk_v2 (v1 warm-start)

`scripts/train_e2e_walking.py`에 `--flat-foot-weight`, `--init-from` 추가.
v1 자산을 보존하며 v2로 warm-start (policy+vecnorm 로드, timestep 0부터).

```bash
python3 scripts/train_e2e_walking.py \
  --run-name e2e_walk_v2 --init-from e2e_walk_v1 \
  --flat-foot-weight 1.5 --total-timesteps 1500000 --n-envs 8
```

## 결과

| 지표 | v1 (before) | v2 (flat-foot) |
|---|---:|---:|
| 오른발 기울기 평균 | 21.8° | **7.9°** |
| 오른발 기울기 최대 | 25.0° | **13.0°** |
| 왼발 기울기 평균 | 1.4° | 1.5° |
| 몸체 순전진 (2000 steps) | +13.4 cm | **+21.9 cm** |

발이 평평해지면서 전진 거리도 증가 (모서리 보행 → 정상 지지로 추진력 개선).
시각적으로 v2는 양발이 바닥에 평평하게 놓임 (iso 프레임 비교 확인).

## 산출물

| 파일 | 설명 |
|---|---|
| `envs/urdf_f_env.py` | `_foot_sole_tilt`, `_flat_foot_penalty` + walking 보상 연동 |
| `scripts/train_e2e_walking.py` | `--flat-foot-weight`, `--init-from` 추가 |
| `scripts/render_e2e_walk.py` | `--run`, `--out` 인자 추가 |
| `outputs/train/urdf_f/e2e_walk_v2/` | v2 policy + vecnorm |
| `outputs/e2e_walk_v2_tracking.mp4` | v2 보행 영상 |

## 다음 조치

- 오른발 기울기 8°로 감소했으나 완전 평평(0°)은 아님 → weight 상향(2.0) 또는 추가 학습으로 개선 여지
- 전후 보폭 좌우 비대칭 (foot_y 순전진 L +3.3 / R +14.9cm)은 여전 → mirror-symmetry 학습 검토
