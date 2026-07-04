# 123. Bilateral Alternating Walking

## 목적

doc 122에서 좌발 step sequence (470/470)가 완성됐다. 이 단계에서는 오른발 step + 왼발 step을 1 episode 내에서 교번으로 연결해 N-cycle 양발 걷기를 달성한다.

## 핵심 문제: right_return distribution mismatch

### 근본 원인 3단계 분석

**1. Reset bounce 의존성**

right_return policy는 초기 reset 시 qvel=0에서 중력이 작용해 1 step에 body가 ~2mm 낙하하는 "bounce"를 학습에 이용했다. Isolated eval에서:

| step | Rclr | 설명 |
|---:|---:|---|
| 0 (init) | +1.634mm | reset 직후 |
| 1 | -2.117mm | bounce (body 낙하, 발이 바닥 압축) |
| 8 | +0.685mm | 반동 |
| 10+ | +0.44mm | 안정화 |

Sequence context에서는 weight_shift_left policy가 robot을 안정적으로 지지하므로 bounce가 발생하지 않는다. 발이 계속 위로 올라가 roll_limit 도달.

**2. Base quaternion 누락**

`weight_shift_left_rcp_ppo_pose.json` 캡처 시 `base_quat`를 저장하지 않아 reset 시 quaternion이 identity로 강제됐다. 실제 sequence 상태는 roll=-0.046 (roll≠0), 이를 무시하면 발 위치가 완전히 달라진다 (Rclr -2.888mm vs +1.634mm).

**3. VecNormalize 통계 불일치**

policy 학습 시 bounce가 있는 dynamics에서 running stats 계산. Sequence에서 bounce 없는 obs를 정규화하면 normalized obs가 학습 분포 밖 (예: norm_obs[12]: 1.0463 vs 학습 시 평균 -3.8 근방).

### 해결책

**1. base_quat 지원 추가 (UrdfFEnv)**

```python
# _load_pose()에서 base_quat 읽기
self._init_base_quat = np.array(payload["base_quat"])

# _init_state()에서 복원
if self._init_base_quat is not None:
    self.data.qpos[3:7] = self._init_base_quat
else:
    self.data.qpos[3:7] = [1, 0, 0, 0]
```

**2. nominal_pose_path 분리 (UrdfFEnv)**

```python
UrdfFEnv(
    pose_path="weight_shift_left_rcp_ppo_pose.json",    # init: 공중 1.63mm
    nominal_pose_path="symmetric_standing_pose.json",    # 목표: 양발 착지
    ...
)
```

`nominal_q` (PD 목표)를 `init_q` (초기 위치)와 분리한다. `nominal_q=symmetric_standing`이면 PD 컨트롤러가 자연스럽게 발을 지면 방향으로 당긴다. **Bounce 없이 자연 착지** 달성.

**3. right_return 재학습 (right_return_from_wsl_rcp_nomstd_150k)**

```bash
python3 scripts/train_urdf_f_ppo.py \
  --pose-json configs/weight_shift_left_rcp_ppo_pose.json \
  --nominal-pose-json configs/symmetric_standing_pose.json \
  --task right_return \
  --total-timesteps 150000 --n-envs 4 --max-episode-steps 250 \
  --action-scale 0.12 --upright-penalty-weight 36.0 \
  --action-penalty-weight 0.08 --action-delta-penalty-weight 0.03 \
  --fall-penalty 20.0 \
  --run-name right_return_from_wsl_rcp_nomstd_150k --device cpu
```

학습 시 VecNormalize 통계가 no-bounce 분포에서 계산되어 sequence context와 일치.

Isolated eval 결과:

| step | Rclr | rc |
|---:|---:|---:|
| 0 | +1.633mm | 1 |
| 1 | +1.622mm | 1 |
| 5 | +0.996mm | 1 |
| 10 | +0.499mm | 4 |
| 25 | +0.195mm | 3 |
| 50+ | -0.090mm | 6-12 |

250/250 완주, bounce 없이 1.63mm → -0.09mm 자연 착지.

## 최종 Walking 구성

### evaluate_urdf_f_walking.py make_stages_simple()

| Stage | Policy | Steps |
|---|---|---:|
| R{N}_weight_shift_left | weight_shift_left_rcp_150k | 100 |
| R{N}_right_return | **right_return_from_wsl_rcp_nomstd_150k** | 170 |
| L{N}_weight_shift_right | weight_shift_right_sym_100k | 100 |
| L{N}_left_clearance | left_clearance_from_wsr_150k | 100 |
| L{N}_left_return | left_return_sym_30k | 170 |
| settle | zero | 100 |

R step = 370 steps/cycle, L step = 370 steps/cycle → 740 steps/cycle

### 코드 변경 사항

- `envs/urdf_f_env.py`: `nominal_pose_path` 파라미터, `base_quat` 지원, `init_q`/`nominal_q` 분리
- `scripts/train_urdf_f_ppo.py`: `--nominal-pose-json` 인자 추가
- `scripts/evaluate_urdf_f_walking.py`: `reset_prev_action` Stage 필드 추가
- `configs/weight_shift_left_rcp_ppo_pose.json`: `base_quat` 필드 추가

## 결과

### 1-cycle (740 steps, 14.8s)

```bash
python3 scripts/evaluate_urdf_f_walking.py \
  --pose-json configs/symmetric_standing_pose.json \
  --cycles 1 \
  --out-dir outputs/eval/urdf_f_walking_1cycle_v9 \
  --seed 1
```

| 항목 | 값 |
|---|---:|
| 완료 step | **740 / 740** |
| 종료 이유 | 없음 |
| sim time | **14.8 s** |
| final roll | **+0.025 rad** |
| final Rclr | -0.089 mm |
| final Lclr | -0.175 mm |
| final rc | 11 |
| final lc | 11 |

단계별 결과:

| Stage | Steps | max Rclr | max Lclr | end roll | end lr |
|---|---:|---:|---:|---:|---:|
| R1_weight_shift_left | 100 | +5.18 mm | — | -0.046 | 0.652 |
| R1_right_return | 170 | +1.61 mm | — | +0.006 | 0.610 |
| L1_weight_shift_right | 100 | — | +0.45 mm | +0.009 | 0.517 |
| L1_left_clearance | 100 | — | +0.47 mm | +0.017 | 0.613 |
| L1_left_return | 170 | — | +0.08 mm | +0.028 | 0.711 |
| settle | 100 | — | — | +0.025 | 0.670 |

5개 seed 전부 740/740 성공.

### 2-cycle (1380 steps, 27.6s)

```bash
python3 scripts/evaluate_urdf_f_walking.py \
  --pose-json configs/symmetric_standing_pose.json \
  --cycles 2 \
  --out-dir outputs/eval/urdf_f_walking_2cycle_v1 \
  --seed 1
```

| 항목 | 값 |
|---|---:|
| 완료 step | **1380 / 1380** |
| 종료 이유 | 없음 |
| final roll | **+0.032 rad** |

| Stage | Steps | max Rclr | max Lclr | end roll |
|---|---:|---:|---:|---:|
| R1_weight_shift_left | 100 | +5.18 mm | — | -0.046 |
| R1_right_return | 170 | +1.61 mm | — | +0.006 |
| L1_weight_shift_right | 100 | — | +0.45 mm | +0.009 |
| L1_left_clearance | 100 | — | +0.47 mm | +0.017 |
| L1_left_return | 170 | — | +0.08 mm | +0.028 |
| R2_weight_shift_left | 100 | +1.62 mm | — | -0.037 |
| R2_right_return | 170 | +1.16 mm | — | +0.012 |
| L2_weight_shift_right | 100 | — | +0.30 mm | +0.012 |
| L2_left_clearance | 100 | — | +0.49 mm | -0.075 |
| L2_left_return | 170 | — | +0.49 mm | +0.038 |
| settle | 100 | — | — | +0.032 |

### 3-cycle (2020 steps, 40.4s)

```bash
python3 scripts/evaluate_urdf_f_walking.py \
  --pose-json configs/symmetric_standing_pose.json \
  --cycles 3 \
  --out-dir outputs/eval/urdf_f_walking_3cycle_v1 \
  --seed 1
```

| 항목 | 값 |
|---|---:|
| 완료 step | **2020 / 2020** |
| 종료 이유 | 없음 |
| final roll | **+0.035 rad** |

## 산출물

| 파일 | 설명 |
|---|---|
| `outputs/eval/urdf_f_walking_1cycle_v9/` | 1-cycle walking 평가 |
| `outputs/eval/urdf_f_walking_2cycle_v1/` | 2-cycle walking 평가 |
| `outputs/eval/urdf_f_walking_3cycle_v1/` | 3-cycle walking 평가 |
| `outputs/train/urdf_f/right_return_from_wsl_rcp_nomstd_150k/` | 최종 right_return 학습 |

## 제한 사항

1. **왼발 clearance 마진**: max 0.47-0.49mm (doc 122와 동일한 hardware 제약). 오른발은 최대 5.18mm이나 왼발 지지 기반의 접촉 수가 적어 왼발이 높이 들리지 않는다.

2. **2nd cycle right swing 감소**: R2에서 Rclr max=1.62mm (R1: 5.18mm). 첫 번째 cycle 이후 robot이 약간 비대칭 상태에서 시작하기 때문.

3. **연속 보행 미구현**: 이 구현은 fixed-location step (앞으로 나아가지 않음). 실제 walking progress는 별도 구현 필요.

## 현재 결론

curriculum 상태:

| task | 상태 |
|---|---|
| standing | 완료 |
| weight_shift_left | 완료 |
| right clearance | 완료 |
| right_return | 완료 (no-bounce nomstd) |
| right step sequence | 완료 ✓ |
| weight_shift_right | 완료 |
| left clearance | 완료 |
| left_return | 완료 |
| left step sequence | 완료 ✓ |
| **bilateral alternating walking** | **완료 ✓ (3 cycles, 2020 steps, 40.4s)** |

## 다음 조치

1. **왼발 clearance 개선**: right foot contact pad 수 증가 or support polygon 개선
2. **연속 보행**: forward motion을 포함한 true walking 구현
3. **no-bounce 재학습 통합**: 다른 policy들도 base_quat 지원 + nominal_pose 분리 적용
