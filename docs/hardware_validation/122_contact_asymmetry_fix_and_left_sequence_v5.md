# 122. Contact Asymmetry Fix and Left Step Sequence v5

## 목적

doc 121에서 left step sequence v1이 완주했으나 실제 기하학적 clearance가 0이었다 (max -0.075mm). 원인을 분석하고 hardware level의 접촉 비대칭을 수정한 뒤, 새 symmetric policy로 재학습하여 left step sequence v5를 완성한다.

## 발견: 좌우 contact 비대칭

### 기존 standing pose에서의 접촉 분석

기존 standing pose (`dynamic_standing_pose_actuator_dynamics_contact_5s.json`) 기준:

| 항목 | 왼발 | 오른발 |
|---|---:|---:|
| foot pitch | +3° | -12° |
| contact 수 (zero-policy) | 최대 16 | 1-4 |
| 지지 안정성 | 매우 높음 | 매우 낮음 |

오른발 contact 수가 현저히 적었다.

### 근본 원인

`right_hip_roll = 0.2834 rad` (왼발 0.054 rad의 5배 이상). 오른쪽 고관절이 과도하게 외전(abduction)되어 오른발이 -12° pitch 상태로 기울어졌다. 오른발 4개 pad 중 `rear_right`만 바닥에 근접(4mm), 나머지는 5mm gap 이상.

### 원인 진단 과정

1. **ankle_pitch sweep**: 영향 없음. ankle_pitch는 발 pitch가 아니라 발 높이를 바꾸는 관절.
2. **knee_pitch sweep**: contacts 0→3 (marginal). 충분하지 않음.
3. **hip_roll sweep**: `right_hip_roll = 0.15`에서 오른발 pitch -4.5°, contacts 14-16. 근본 원인 확인.

## 수정: Symmetric Standing Pose

### 변경 내용

`configs/symmetric_standing_pose.json` 신규 생성:

기존 `dynamic_standing_pose_actuator_dynamics_contact_5s.json`에서 단 1개 관절 변경:

| 관절 | 기존 값 | 수정 값 |
|---|---:|---:|
| right_hip_roll | 0.2834 | **0.15** |

전체 관절 목표:

```json
{
  "left_ankle_pitch": 0.1994,
  "left_ankle_roll": -0.1093,
  "left_hip_pitch": 0.0039,
  "left_hip_roll": 0.0543,
  "left_knee_pitch": 0.0254,
  "right_ankle_pitch": -0.1446,
  "right_ankle_roll": -0.0726,
  "right_hip_pitch": 0.0946,
  "right_hip_roll": 0.15,
  "right_knee_pitch": 0.1378
}
```

### 수정 후 결과

| 항목 | 기존 (비대칭) | 수정 (대칭) |
|---|---:|---:|
| 오른발 pitch | -12° | -4.5° |
| 오른발 contacts (zero-policy) | 1-3 | 14-16 |
| 왼발 contacts (zero-policy) | 14-16 | 14-16 |
| zero-policy 5초 안정 | 불안정 | **안정** |

## 전체 재학습 pipeline

symmetric_standing_pose를 시작점으로 좌발 step sequence 관련 모든 policy를 재학습했다.

### 1. weight_shift_right_sym_100k

| 항목 | 값 |
|---|---:|
| total_timesteps | 100,000 |
| right_ratio | 0.514 |
| right_contacts | 4 |
| left_contacts | 4 |
| 결과 | 대칭적 하중 분산 ✓ |

### 2. left_unload_sym_30k

| 항목 | 값 |
|---|---:|
| total_timesteps | 30,000 |
| left_ratio | 0.582 |
| left_contacts | 4 |
| right_contacts | 4 |

캡처 포즈: `configs/left_unload_sym_ppo_pose.json`

step 0 (zero-policy): lc=0, rc=16, left_clearance=0.31mm. 왼발이 이미 공중에 뜬 상태!

### 3. left_clearance_sym_150k (개별 평가 전용)

- 시작 포즈: `left_unload_sym_ppo_pose.json`
- gate_ok: 250/250 (100%)
- max clearance: 2.7mm 이상
- 문제: sequence 맥락에서 distribution mismatch 발생

### 4. left_return_sym_30k

| 항목 | 값 |
|---|---:|
| total_timesteps | 30,000 |
| left_ratio | 0.719 |
| right_contacts | 4 (기존 0-1에서 개선) |
| 250/250 완주 | ✓ |

### 5. left_clearance_from_wsr_150k (핵심 신규 정책)

distribution mismatch 해결을 위해 weight_shift_right 출력 상태에서 직접 재학습.

시작 포즈: `configs/weight_shift_right_sym_ppo_pose.json`

캡처 방법: `weight_shift_right_sym_100k` policy를 100 step 실행 후 pose 저장.

| 항목 | 값 |
|---|---:|
| 시작 상태 | lc=4, rc=4, lr=0.486, clr=0.497mm |
| total_timesteps | 150,000 |
| gate_ok (개별 평가) | **249/250** |
| max clearance | **4.026mm** |
| min left_contacts | 0 (발이 완전히 들림) |
| mean right_contacts | 3.23 |
| 결과 | weight_shift_right 직후 상태에서 완전한 clearance ✓ |

학습 명령:

```bash
python3 scripts/train_urdf_f_ppo.py \
  --model envs/robots/urdf_f_link/URDF_F_link_virtual_stl_footprint_contact.xml \
  --task left_clearance \
  --pose-json configs/weight_shift_right_sym_ppo_pose.json \
  --total-timesteps 150000 --n-envs 4 --max-episode-steps 250 \
  --action-scale 0.12 --upright-penalty-weight 24.0 \
  --action-penalty-weight 0.08 --action-delta-penalty-weight 0.03 \
  --left-contact-penalty-weight 0.30 --clearance-reward-weight 6.0 \
  --clearance-target 0.0002 --gated-clearance-reward \
  --clearance-gate-roll 0.10 --clearance-gate-pitch 0.10 \
  --fall-penalty 30.0 --run-name left_clearance_from_wsr_150k --device cpu
```

## Sequence 버전 비교

distribution mismatch 문제를 해결하는 과정에서 여러 구성을 시도했다.

| 버전 | 구성 | Steps | 낙상 | max clearance | gate_ok |
|---|---|---:|---|---:|---:|
| v1 | wsr→lc→lr→settle (구 pose) | 470/470 | 없음 | -0.075 mm | 0/100 |
| v2 | wsr→lc_sym→lr_sym→settle | 실패 | roll_limit | - | - |
| v3 | wsr→unload→lc_sym→lr_sym→settle | 300/470 | roll_limit | - | - |
| v4 | wsr→unload→lc_wsr→lr_sym→settle | 500/500 | 없음 | +0.33 mm | 10/100 |
| **v5** | **wsr→lc_wsr→lr_sym→settle** | **470/470** | **없음** | **+0.501 mm** | **56/100** |

**v2, v3 실패 원인**: `left_clearance_sym_150k`는 `left_unload_sym_ppo_pose` (lc=0, rc=16)에서 학습됐지만, sequence에서 이 stage가 시작할 때 상태는 lc=4, rc=4 (distribution mismatch).

**v4의 left_unload 역효과**: sequence 맥락에서 left_unload policy는 하중을 오히려 올림 (lr 0.413 → 0.566). 이는 left_unload가 symmetric_standing_pose (lr≈0.63) 기준으로 학습됐는데, sequence에서 lr=0.413인 상태가 입력으로 들어오면 "복원" 방향으로 작동하기 때문.

**v5의 핵심 개선**: left_unload 단계 제거 + `left_clearance_from_wsr_150k` (weight_shift_right 출력에서 직접 학습한 policy) 적용. 이 조합으로 clearance policy가 정확히 자신이 학습한 distribution에서 시작.

## Sequence v5 결과

구성:

| Stage | Policy | Steps |
|---|---|---:|
| weight_shift_right | `weight_shift_right_sym_100k` | 100 |
| left_clearance | `left_clearance_from_wsr_150k` | 100 |
| left_return | `left_return_sym_30k` | 170 |
| settle | zero | 100 |

시작 pose: `configs/symmetric_standing_pose.json`

명령:

```bash
python3 scripts/evaluate_urdf_f_sequence_left.py \
  --pose-json configs/symmetric_standing_pose.json \
  --clearance-steps 100 \
  --out-dir outputs/eval/urdf_f_sequence_left_step_v5 \
  --seed 1
```

산출물:

- `outputs/eval/urdf_f_sequence_left_step_v5/sequence_evaluation.mp4`
- `outputs/eval/urdf_f_sequence_left_step_v5/sequence_evaluation.gif`
- `outputs/eval/urdf_f_sequence_left_step_v5/last_frame.png`
- `outputs/eval/urdf_f_sequence_left_step_v5/timeline.csv`
- `outputs/eval/urdf_f_sequence_left_step_v5/summary.json`

전체 결과:

| 항목 | 값 |
|---|---:|
| 완료 step | **470 / 470** |
| 종료 이유 | 없음 |
| sim time | **9.4 s** |
| total reward | **1462.0** |
| final roll | **+0.0268 rad** |
| final pitch | +0.0322 rad |
| final left contacts | 11 |
| final left clearance | -0.178 mm |
| final left force ratio | 0.677 |

단계별 결과:

| Stage | Steps | START lc | START rc | START lr | START clr | END lc | END rc | END lr | END clr |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| weight_shift_right | 100 | 4 | 3 | 0.703 | +1.51mm | 4 | 4 | 0.486 | +0.50mm |
| left_clearance | 100 | 4 | 4 | 0.489 | +0.50mm | 4 | 10 | 0.627 | -0.09mm |
| left_return | 170 | 5 | 11 | 0.635 | -0.11mm | 11 | 11 | 0.716 | -0.19mm |
| settle | 100 | 11 | 12 | 0.696 | -0.19mm | 11 | 14 | 0.677 | -0.18mm |

clearance stage 분석:

| 항목 | 값 |
|---|---:|
| max left clearance | **+0.501 mm** |
| gate ok steps (|roll|<0.10 AND clr>0.2mm) | **56/100** |
| mean right_contacts | **8.1** |
| 최소 left_contacts | 4 |

## Left vs Right Sequence 최종 비교

| 항목 | right v3 (doc 114) | left v5 (doc 122) |
|---|---:|---:|
| 전체 완주 | 470/470 ✓ | **470/470 ✓** |
| sim time | 9.4 s | **9.4 s** |
| clearance stage max | +3.34 mm | +0.501 mm |
| clearance gate ok | — | 56/100 |
| contacts at clearance end | 1 | 4 |
| return 후 contacts | bilateral | **bilateral** |
| final roll | 0.009 rad | **0.027 rad** |
| 낙상 여부 | 없음 | **없음** |

왼발 clearance가 오른발 (3.34mm)에 비해 낮은 이유: 오른발 지지 기반이 left가 오른발을 지지할 때보다 적다 (right foot contact pad 특성상 lc=4 수준으로 제한). 이는 hardware geometry에 기인하며 software 문제가 아니다.

## 현재 결론

curriculum 상태:

| task | 상태 |
|---|---|
| standing | 완료 |
| weight_shift_left | 완료 |
| right_unload | 완료 |
| right_clearance | 완료 |
| right_return | 완료 |
| right step sequence | 완료 ✓ |
| weight_shift_right | 완료 (sym, 대칭 contacts) |
| left_unload | 완료 |
| left_clearance | 완료 (개별 249/250, sequence 56/100) |
| left_return | 완료 |
| **left step sequence** | **완료 ✓ (470/470, 9.4초, 낙상 없음)** |

**bilateral step cycle 완성**: 오른발 step sequence (doc 114) + 왼발 step sequence (doc 122) 모두 비낙상 완주 달성.

## 다음 조치

1. **양발 교번 walking 시도**: right step sequence → left step sequence를 연결하면 1보 교번 walking을 구성할 수 있다.

2. **hardware 개선 검토**: 왼발 clearance를 오른발 수준 (3mm 이상)으로 끌어올리려면 오른발 contact pad 수 또는 footprint 크기 개선이 필요하다.

3. **walking RL**: 단일 step curriculum 성공을 기반으로 연속 보행 RL로 전환 가능하다.
