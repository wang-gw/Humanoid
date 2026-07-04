# 116. Left Sequence Environment Preparation

## 목적

이번 단계의 목적은 왼발 sequence를 위한 RL 환경과 pose search 도구를 준비하는 것이다.

핵심 질문:

1. 왼발 unload, clearance, return을 RL task로 정의하고 환경에 연결할 수 있는가?
2. 오른발 task와 대칭적으로 observation에 왼발 clearance를 포함시킬 수 있는가?
3. 새 task들이 기존 task들을 깨뜨리지 않는가?

## 시작점

이전 단계 `115_left_mirror_sequence_precheck.md`의 결론은 다음과 같았다.

- 오른발 step sequence는 단일 동작 수준에서 1차 성공했다.
- 단순 pose mirror (left/right joint swap + roll 부호 반전)는 모두 실패했다.
- 실패 원인: 현재 MJCF 모델의 좌우 joint axis/sign, contact geometry, COM offset이 수학적 mirror로 동작하지 않는다.
- 결론: 왼발 sequence는 오른쪽 pose를 숫자만 교환하는 방식이 아니라 왼쪽 전용 pose search와 왼쪽 전용 RL task가 필요하다.

이번 단계는 그 왼쪽 전용 환경과 도구를 만드는 작업이다.

## 코드 변경

### `envs/urdf_f_env.py`

**새 메서드: `_left_foot_clearance()`**

역할:

- `_right_foot_clearance()`와 동일한 방식으로 왼발 접촉 geom의 최저 Z 좌표를 반환
- `foot_geom_to_side`에서 `"left"` side geom들을 순회

**새 파라미터: `left_contact_penalty_weight`**

역할:

- `right_contact_penalty_weight`와 대칭
- left 발 접촉 수에 penalty를 줄 때 사용
- 기본값 `0.0`

**새 task 4개:**

| task | 목적 | reward 핵심 |
|---|---|---|
| `weight_shift_right` | 왼발 들기 전 무게를 오른쪽으로 이동 | `right_force_ratio → 0.65` 추종 |
| `left_unload` | 왼발 하중 감소 | `left_force < 30 N` |
| `left_clearance` | 왼발 들어올리기 | `left_clearance ≥ clearance_target` |
| `left_return` | 왼발 착지 복귀 | `left_clearance → 0`, 왼발 contact/force 회복 |

**observation 확장:**

기존 observation의 마지막 부분을 다음과 같이 변경했다.

| 변경 전 | 변경 후 |
|---|---|
| `[right_clearance, base_z_delta]` (크기 2) | `[left_clearance, right_clearance, base_z_delta]` (크기 3) |

결과적으로 obs_dim이 `44`에서 `45`로 늘었다.

주의: 기존 right foot task로 학습한 checkpoint들은 obs_dim이 달라 이 환경에서 직접 로드되지 않는다. right task 학습은 이미 완료됐으므로 새 학습 실험에는 영향 없다.

**`_info()` 확장:**

`left_clearance` 항목 추가.

### `scripts/train_urdf_f_ppo.py`

역할:

- `--task` choices에 4개 left task 추가
- `--left-contact-penalty-weight` 인자 추가

### `scripts/search_weight_shift_right_pose.py` (신규)

역할:

- `search_weight_shift_pose.py`의 대칭 버전
- CEM (Cross-Entropy Method) 기반 pose search
- `target_left_ratio` 대신 `target_right_ratio`를 score로 사용
- 오른쪽 무게 이동 pose 후보를 탐색해 JSON으로 저장

## Observation 정의

현재 observation shape는 `45`다.

| 항목 | 크기 | 의미 |
|---|---:|---|
| base roll/pitch/yaw | 3 | 상체 자세 |
| base angular velocity | 3 | 상체 회전 속도 |
| joint position delta | 10 | 기준 관절각 대비 현재 관절각 |
| joint velocity | 10 | 관절 속도 |
| left/right force ratio | 2 | 좌우 하중 비율 |
| contact/force 요약 | 4 | 좌우 접촉 수와 힘 |
| left/right clearance, base height delta | 3 | 좌우 스윙 발 높이와 base 높이 변화 |
| previous action | 10 | action 변화 억제용 정보 |

## 실행 결과

### 1. 새 task smoke test

명령:

```bash
python3 -c "
import sys; sys.path.insert(0, '.')
from envs.urdf_f_env import UrdfFEnv
for task in ['weight_shift_right', 'left_unload', 'left_clearance', 'left_return']:
    env = UrdfFEnv(task=task)
    obs, _ = env.reset()
    action = env.action_space.sample()
    obs2, rew, term, trunc, info = env.step(action)
    print(f'{task}: obs_shape={obs.shape}  reward={rew:.3f}  left_clearance={info[\"left_clearance\"]:.4f}')
    env.close()
"
```

결과:

| task | obs shape | reward | left_clearance |
|---|---|---:|---:|
| weight_shift_right | (45,) | 0.680 | 0.0013 m |
| left_unload | (45,) | 0.494 | 0.0014 m |
| left_clearance | (45,) | 2.567 | 0.0016 m |
| left_return | (45,) | 4.281 | 0.0015 m |

판단:

- 새 task 4개 모두 정상 실행됐다.
- obs_dim 45가 일관되게 반환됐다.

### 2. 기존 task 회귀 확인

명령:

```bash
python3 -c "
import sys; sys.path.insert(0, '.')
from envs.urdf_f_env import UrdfFEnv
for task in ['standing', 'weight_shift_left', 'right_unload', 'right_clearance', 'right_return']:
    env = UrdfFEnv(task=task)
    obs, _ = env.reset()
    action = env.action_space.sample()
    obs2, rew, term, trunc, info = env.step(action)
    print(f'{task}: obs_shape={obs.shape}  reward={rew:.3f}')
    env.close()
"
```

결과:

| task | obs shape | reward |
|---|---|---:|
| standing | (45,) | 1.995 |
| weight_shift_left | (45,) | 1.722 |
| right_unload | (45,) | 2.919 |
| right_clearance | (45,) | 2.995 |
| right_return | (45,) | 2.326 |

판단:

- 기존 5개 task 모두 정상 동작한다.
- obs_dim 변경이 기존 reward/termination 계산에 영향을 주지 않았다.

## 현재 결론

성공한 것:

- `_left_foot_clearance()` 메서드 추가 및 동작 확인
- obs에 왼발 clearance 추가 (obs_dim 44 → 45)
- `weight_shift_right`, `left_unload`, `left_clearance`, `left_return` task reward 구현
- `left_contact_penalty_weight` 파라미터 추가
- `search_weight_shift_right_pose.py` 스크립트 생성
- 기존 task 9개 전부 회귀 이상 없음

아직 부족한 것:

- `weight_shift_right` pose가 실제로 PD 제어에서 안정적으로 달성 가능한지 미확인
- 새 task들의 실제 RL 학습 실행 전
- 왼발 기준 pose 미확정

## 다음 조치

### 선택 A: weight_shift_right pose search 먼저

이유:

- 오른발 sequence에서도 항상 PD probe로 가능성을 먼저 확인하고 RL에 진입했다.
- 단순 mirror가 실패했으므로, RL 시작 pose를 새로 탐색해야 한다.

명령:

```bash
cd scripts
python search_weight_shift_right_pose.py \
  --model ../envs/robots/urdf_f_link/URDF_F_link_virtual_stl_footprint_contact.xml \
  --seed-pose ../configs/dynamic_standing_pose_actuator_dynamics_contact_5s.json \
  --out ../configs/weight_shift_right065_pose.json \
  --out-dir ../outputs/analysis/weight_shift_right_pose_search \
  --target-right-ratio 0.65 \
  --samples 360 \
  --iterations 3
```

산출물:

- `outputs/analysis/weight_shift_right_pose_search/weight_shift_right_pose_candidates.csv`
- `outputs/analysis/weight_shift_right_pose_search/best_weight_shift_right_pose_timeline.csv`
- `outputs/analysis/weight_shift_right_pose_search/weight_shift_right_pose_search_summary.json`
- `configs/weight_shift_right065_pose.json`

통과 기준:

| 항목 | 기준 |
|---|---:|
| final right_force_ratio | ≥ 0.60 |
| final abs(roll) | ≤ 0.10 rad |
| final abs(pitch) | ≤ 0.10 rad |
| contact_fraction | ≥ 0.95 |
| saturation_fraction | ≤ 0.05 |

### 선택 B: RL 직접 진입

이유:

- 오른발 학습에서 RL이 스스로 pose를 찾는 능력을 보여줬다.
- standing pose에서 바로 `weight_shift_right` task로 RL을 시작할 수 있다.

주의:

- pose search 없이 바로 RL에 진입하면 초기 성능이 낮고 수렴이 느릴 수 있다.
- 학습 실패 시 원인 분리가 어렵다.

현재 권장:

> 선택 A로 진행한다. `weight_shift_right` pose search를 먼저 실행하고, `right_force_ratio ≥ 0.60` 달성 여부를 확인한 뒤 RL 학습으로 넘어간다. pose search 결과에 따라 다음 단계 문서에서 결과를 기록한다.
