# 121. Left Step Sequence Evaluation

## 목적

이번 단계의 목적은 doc 114 (right step sequence evaluation)를 왼쪽에 미러링하는 것이다.

각 단계에서 따로 검증한 policy들을:

1. weight_shift_right
2. left_clearance
3. left_return
4. settle (zero)

하나의 연속 에피소드에서 순서대로 연결했을 때 안정적으로 동작하는지 확인한다.

## 추가한 스크립트

- `scripts/evaluate_urdf_f_sequence_left.py`

역할:
- 하나의 MuJoCo episode 안에서 왼발 sequence 4단계를 순서대로 적용
- 각 stage마다 action scale, reward/termination 파라미터를 policy 학습 파라미터와 일치
- `left_contact_penalty_weight` 지원 추가 (오른쪽 sequence 스크립트에는 없던 항목)
- 전체 영상, timeline, summary 저장

## Sequence v1

구성:

| Stage | Policy | Steps | 비고 |
|---|---|---:|---|
| weight_shift_right | `weight_shift_right_40k` | 100 | action_scale=0.15, upright=24.0 |
| left_clearance | `left_clearance_unload_gated_150k` | 100 | action_scale=0.12, gated, gate_roll=0.10 |
| left_return | `left_return_from_unload_30k` | 170 | action_scale=0.12, upright=24.0 |
| settle | zero | 100 | action_scale=0.05 |

시작 pose: `configs/dynamic_standing_pose_actuator_dynamics_contact_5s.json`

명령:

```bash
python3 scripts/evaluate_urdf_f_sequence_left.py \
  --pose-json configs/dynamic_standing_pose_actuator_dynamics_contact_5s.json \
  --clearance-steps 100 \
  --out-dir outputs/eval/urdf_f_sequence_left_step_v1 \
  --seed 1
```

산출물:

- `outputs/eval/urdf_f_sequence_left_step_v1/sequence_evaluation.mp4`
- `outputs/eval/urdf_f_sequence_left_step_v1/sequence_evaluation.gif`
- `outputs/eval/urdf_f_sequence_left_step_v1/last_frame.png`
- `outputs/eval/urdf_f_sequence_left_step_v1/timeline.csv`
- `outputs/eval/urdf_f_sequence_left_step_v1/summary.json`

## Sequence v1 결과

전체 결과:

| 항목 | 값 |
|---|---:|
| 완료 step | 470 / 470 |
| 종료 이유 | 없음 |
| sim time | 9.4 s |
| total reward | 841.3 |
| final roll | +0.0116 rad |
| final pitch | +0.0175 rad |
| final left contacts | 14 |
| final left clearance | -0.145 mm |
| final left force ratio | 0.6426 |

단계별 결과:

| Stage | Steps | final roll | final pitch | final left clearance | final left contacts | final left_force_ratio |
|---|---:|---:|---:|---:|---:|---:|
| weight_shift_right | 100 | -0.0079 | -0.0063 | -0.096 mm | 11 | 0.4094 |
| left_clearance | 100 | -0.0681 | +0.0112 | -0.083 mm | 5 | 0.3731 |
| left_return | 170 | -0.0210 | +0.0334 | -0.128 mm | 15 | 0.6480 |
| settle | 100 | +0.0116 | +0.0175 | -0.145 mm | 14 | 0.6426 |

clearance stage 분석:

| 항목 | 값 |
|---|---:|
| max left clearance | -0.075 mm |
| mean left clearance | -0.088 mm |
| mean left contacts | 6.73 |
| gate ok steps (|roll|<0.10 AND clearance>0.2mm) | 0 / 100 |

## 해석

**성공한 것:**

- 470/470 steps 완주, 낙상 없음 ✓
- weight_shift_right: left_ratio 0.64 → 0.409 (왼발 하중 이동) ✓
- left_clearance: left_contacts 11 → 5 (접촉 수 감소) ✓
- left_return: left_ratio 0.373 → 0.648 (하중 복원) ✓
- settle: roll 0.012 rad (매우 안정, 거의 수직) ✓
- 전체 sequence 9.4초 비낙상 완료 ✓

**제한 사항:**

left_clearance stage에서 실제 기하학적 clearance (발이 지면 위로 뜨기)가 달성되지 않았다. max clearance -0.075mm는 음수이므로 발이 실제로 뜬 적이 없다.

이 제한의 원인:

1. `left_clearance_unload_gated_150k` policy는 `left_unload_ppo_pose`에서 학습됐다. 이 pose의 관절 각도/몸통 위치는 `weight_shift_right_40k` policy가 만드는 최종 상태와 다르다.
2. clearance 단계에서 left_contacts는 11 → 5로 줄었다 (부분적 하중 감소는 작동). 하지만 발이 완전히 들리지는 않았다.
3. 오른발 sequence v3에서 clearance는 3.34mm (실제 발 들기)였다. 왼발은 0/100 gate-ok steps.

## Right vs Left Sequence 비교

| 항목 | right v3 | left v1 |
|---|---:|---:|
| 전체 완주 | 470/470 ✓ | 470/470 ✓ |
| clearance stage max | +3.34 mm | -0.075 mm |
| clearance gate ok steps | 측정 안함 | 0/100 |
| contacts at clearance end | 1 | 5 |
| return 후 contacts | 4 | 15 |
| final roll | 0.009 rad | 0.012 rad |
| final force ratio | 0.277 (right) | 0.643 (left) |

핵심 차이: right sequence는 오른발이 실제로 들렸다가 내려왔다. left sequence는 왼발 하중이 줄었지만 발이 지면 위로 완전히 뜨지는 않았다.

## 현재 결론

curriculum 상태:

| task | 상태 |
|---|---|
| standing | 완료 |
| weight_shift_left | 완료 |
| right_unload | 완료 |
| right_clearance | 완료 |
| right_return | 완료 |
| weight_shift_right | 완료 |
| left_unload | 완료 |
| left_clearance | 완료 (개별 실험) |
| left_return | 완료 (개별 실험) |
| **left step sequence** | **완주 성공, 실제 clearance 없음** |

성공한 것:
- 9.4초 연속 비낙상 sequence 완주
- 왼발 하중 이동 → 감소 → 복원 cycle 시연
- settle 후 roll 0.012 rad 달성 (very stable)

아직 부족한 것:
- sequence 맥락에서 실제 왼발 기하학적 clearance
- 개별 실험 (max 10.95mm)과 연결 성능 (0mm) 간 격차

## 다음 조치

### 선택 A: clearance 단계 연장

clearance stage를 100 step에서 150-200 step으로 늘려 policy가 더 많은 time을 갖게 한다.

### 선택 B: left_unload stage 추가

sequence에 `left_unload_30k` stage를 clearance 전에 삽입한다.

```
weight_shift_right (100) → left_unload (100) → left_clearance (100) → left_return (170) → settle (100)
```

이렇게 하면 clearance policy가 학습한 distribution (left_unload state)에 더 가까운 상태에서 시작한다.

### 선택 C: hardware 개선 결정

오른발 contact pad 수를 늘리거나 support polygon을 개선하는 hardware 변경 없이는 실제 왼발 clearance → return cycle이 어렵다는 결론을 내리고 hardware 개선 설계로 넘어간다.

현재 권장:

> 먼저 선택 B (left_unload stage 삽입)를 시도한다. clearance policy의 학습 distribution과 sequence 상태 간 격차를 줄이는 가장 직접적인 방법이다.
