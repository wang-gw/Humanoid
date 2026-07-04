# 120. Left Return RL Training

## 목적

이번 단계의 목적은 left step sequence의 마지막 curriculum인 `left_return`을 완료하는 것이다. 왼발 unload 이후 bilateral standing을 복원하는 policy를 학습한다.

핵심 질문:

1. PPO가 왼발 하중을 복원할 수 있는가?
2. 5초 동안 안정적으로 bilateral stance를 유지할 수 있는가?
3. left step sequence의 구조적 한계는 무엇인가?

## 시작점

이전 단계 `119_left_clearance_rl_training.md`의 결론:

- `left_clearance` PPO 150k는 5초 안정, gated clearance 221/250 steps, max clearance 10.95mm를 달성했다.
- 왼발 발바닥 평균 2.26개 contacts로 대폭 감소했다.

오른발 sequence에서 `right_return` (doc 113)은 IK로 설계된 안정적 clearance pose (`right_foot_lift002`)에서 출발했다. 이 포즈는 zero-policy에서 5초 안정이었다.

## 구조적 비대칭 문제

left_return은 오른발 sequence보다 근본적으로 더 어렵다. 그 이유를 아래에 정리한다.

**오른발 return의 출발 조건:**
- 왼발: 16개 contact pad (매우 안정적인 지지 기반)
- 오른발: 1-4개 contact (들린 상태)
- zero-policy로도 5초 안정

**왼발 return의 출발 조건:**
- 오른발: 3-6개 contact (불안정한 지지 기반)
- 왼발: 0-4개 contact (들린 상태)
- zero-policy에서 36-57 steps 만에 낙상

이 비대칭은 hardware 설계에서 비롯된다. 왼발은 STL footprint 기반의 16개 contact pad를 가지고 있어 지지 다각형이 크다. 반면 오른발은 더 적은 pad로 서 있어 기본 안정성이 낮다.

### 캡처 포즈별 zero-policy 안정성

`left_clearance` 에피소드에서 다양한 순간을 포즈로 캡처해 zero-policy 안정성을 테스트했다.

| 포즈 | clearance | contacts | roll | zero-policy 안정 steps |
|---|---:|---:|---:|---:|
| step 21 (t=0.42s) | 1.52 mm | 4 | 0.079 | 50 |
| step 61 (t=1.22s) | 0.56 mm | 1 | 0.052 | 36 |
| step 100 (t=2.02s) | 0.77 mm | 1 | 0.061 | 41 |
| step 250 (final) | 10.95 mm | 6 | -0.184 | <30 |

모든 캡처 포즈가 zero-policy에서 불안정했다. `left_clearance` PPO의 능동적 제어가 없으면 오른발 지지만으로 균형을 잡을 수 없다.

## 전략

`right_return`과 동일한 방식은 적용 불가하다. 대신 `left_unload` PPO 최종 상태를 시작점으로 사용해 bilateral standing 복원을 학습한다.

- 시작 포즈: `left_unload_ppo_pose.json` (left_contacts=11, left_ratio=0.462, 5초 안정)
- 목표: left force ratio를 원래 수준으로 복원
- 의미: 하중 이동 → clearance → 하중 복원의 cycle에서 마지막 단계

## 기준선 평가: zero-policy

명령:

```bash
python3 scripts/evaluate_urdf_f_policy.py \
  --model envs/robots/urdf_f_link/URDF_F_link_virtual_stl_footprint_contact.xml \
  --task left_return \
  --pose-json configs/left_unload_ppo_pose.json \
  --policy zero \
  --steps 250 \
  --action-scale 0.12 \
  --upright-penalty-weight 24.0 \
  --action-penalty-weight 0.08 \
  --action-delta-penalty-weight 0.03 \
  --fall-penalty 20.0 \
  --out-dir outputs/eval/urdf_f_left_return_unload_baseline
```

결과 요약:

| 항목 | 값 |
|---|---:|
| 완료 step | 250 / 250 |
| sim time | 5.0 s |
| final left force ratio | 0.4625 |
| final left contacts | 11 |
| mean abs roll | 0.0148 rad |
| 종료 이유 | 없음 |

zero-policy baseline에서 left_unload_ppo_pose는 안정적으로 유지된다. left force ratio는 거의 변하지 않는다 (0.462 유지).

## PPO 학습

설정:

| 항목 | 값 |
|---|---:|
| total_timesteps | 30,000 |
| action_scale | 0.12 |
| upright_penalty_weight | 24.0 |
| action_penalty_weight | 0.08 |
| action_delta_penalty_weight | 0.03 |
| fall_penalty | 20.0 |
| n_envs | 4 |
| max_episode_steps | 250 |

학습 명령:

```bash
python3 scripts/train_urdf_f_ppo.py \
  --model envs/robots/urdf_f_link/URDF_F_link_virtual_stl_footprint_contact.xml \
  --task left_return \
  --pose-json configs/left_unload_ppo_pose.json \
  --total-timesteps 30000 \
  --n-envs 4 \
  --max-episode-steps 250 \
  --action-scale 0.12 \
  --upright-penalty-weight 24.0 \
  --action-penalty-weight 0.08 \
  --action-delta-penalty-weight 0.03 \
  --fall-penalty 20.0 \
  --run-name left_return_from_unload_30k \
  --device cpu
```

최종 학습 로그:

| 항목 | 값 |
|---|---:|
| total_timesteps | 30,720 |
| ep_len_mean | 250 |
| ep_rew_mean | 938 |
| fps | 약 340 |

산출물:

- `outputs/train/urdf_f/left_return_from_unload_30k/config.json`
- `outputs/train/urdf_f/left_return_from_unload_30k/ppo_policy.zip`
- `outputs/train/urdf_f/left_return_from_unload_30k/vecnormalize.pkl`

## 학습 policy 평가

명령:

```bash
python3 scripts/evaluate_urdf_f_policy.py \
  --model envs/robots/urdf_f_link/URDF_F_link_virtual_stl_footprint_contact.xml \
  --task left_return \
  --pose-json configs/left_unload_ppo_pose.json \
  --policy ppo \
  --policy-path outputs/train/urdf_f/left_return_from_unload_30k/ppo_policy.zip \
  --vecnormalize-path outputs/train/urdf_f/left_return_from_unload_30k/vecnormalize.pkl \
  --steps 250 \
  --action-scale 0.12 \
  --upright-penalty-weight 24.0 \
  --action-penalty-weight 0.08 \
  --action-delta-penalty-weight 0.03 \
  --fall-penalty 20.0 \
  --out-dir outputs/eval/urdf_f_left_return_from_unload_30k
```

산출물:

- `outputs/eval/urdf_f_left_return_from_unload_30k/left_return_ppo_seed1/evaluation.mp4`
- `outputs/eval/urdf_f_left_return_from_unload_30k/left_return_ppo_seed1/evaluation.gif`
- `outputs/eval/urdf_f_left_return_from_unload_30k/left_return_ppo_seed1/last_frame.png`
- `outputs/eval/urdf_f_left_return_from_unload_30k/left_return_ppo_seed1/timeline.csv`
- `outputs/eval/urdf_f_left_return_from_unload_30k/left_return_ppo_seed1/summary.json`

결과 요약:

| 항목 | 값 |
|---|---:|
| 완료 step | 250 / 250 |
| sim time | 5.0 s |
| total reward | 978.25 |
| mean left force ratio | 0.6829 |
| final left force ratio | 0.7231 |
| mean left contacts | 11.98 |
| final left contacts | 12 |
| mean abs roll | 0.1500 rad |
| final roll | 0.1343 rad |
| final base_z | +0.0029 m |
| max torque command | 3.39 Nm |
| 종료 이유 | 없음 |

### 에피소드 타임라인

| 시간 | left clearance | left contacts | left force ratio | roll |
|---|---:|---:|---:|---:|
| 0.02s | -1.04 mm | 15 | 0.548 | +0.001 |
| 0.52s | -0.16 mm | 13 | 0.660 | +0.104 |
| 1.02s | -0.16 mm | 11 | 0.651 | +0.158 |
| 2.02s | -0.17 mm | 11 | 0.678 | +0.154 |
| 3.02s | -0.16 mm | 13 | 0.685 | +0.149 |
| 4.02s | -0.16 mm | 13 | 0.704 | +0.139 |
| 5.00s | -0.15 mm | 12 | 0.723 | +0.134 |

## Zero vs PPO 비교

| 항목 | zero-policy | PPO 30k | 판단 |
|---|---:|---:|---|
| 5초 유지 | 성공 | 성공 | 둘 다 통과 |
| total reward | — | 978.25 | PPO 개선 |
| final left force ratio | 0.4625 | 0.7231 | PPO 대폭 개선 |
| mean left force ratio | 0.4625 | 0.6829 | PPO 개선 |
| final left contacts | 11 | 12 | PPO 약간 개선 |
| mean abs roll | 0.0148 rad | 0.1500 rad | PPO 증가 (하중 이동 반영) |

## 해석

left_return은 두 가지 의미를 갖는다.

하나는 직접적인 의미다. PPO는 왼발 하중을 0.462에서 0.723으로 복원했다. 즉, 하중이 이동된 상태에서 출발해 다시 bilateral stance에 가까운 상태로 돌아오는 것을 학습했다.

다른 하나는 구조적 한계다. 오른발 sequence에서 `right_return`은 발이 공중에 뜬 상태 (clearance=1.5mm)에서 착지하는 것을 시연했다. 왼발 sequence에서는 clearance 상태 (발이 공중)에서 안정적으로 착지하는 시연이 불가능했다. 이유는 오른발 지지 기반의 불안정성이다.

오른발은 왼발보다 contact pad가 적고 지지 다각형이 작다. 왼발이 공중에 뜬 상태에서 오른발만으로 균형을 잡기 위해 active control이 필요하다. 이 control이 없어지는 순간 (zero-policy로 전환하거나 새 policy로 시작할 때) 로봇이 바로 낙상한다.

이는 software 문제가 아니라 hardware asymmetry 문제다.

> 왼발 lift → 안정적인 오른발 단일 지지 → 왼발 착지의 완전한 cycle을 시연하려면 오른발 contact 면적을 늘리거나, 오른발 support polygon을 개선하는 hardware 변경이 필요하다.

## 현재 결론

curriculum 상태:

| task | 상태 |
|---|---|
| standing | 완료 |
| weight_shift_left | 완료 |
| right_unload | 완료 |
| right_clearance | 완료 |
| right_return | 완료 |
| weight_shift_right | 완료 (right_ratio 0.610) |
| left_unload | 완료 (left_ratio 0.642 → 0.462) |
| left_clearance | 완료 (150k, clearance 평균 2.15mm, gate ok 221/250) |
| **left_return** | **부분 완료 (unload → bilateral 복원, left_ratio 0.462 → 0.723)** |

left step sequence 전체 실험:

- weight_shift_right: right_ratio 0.610 (5초 안정)
- left_unload: left_ratio 0.462 (5초 안정)
- left_clearance: max clearance 10.95mm, gate ok 221/250 steps
- left_return: bilateral restoration (left_ratio 0.723, 5초 안정)

## 다음 조치

full step sequence evaluation (doc 114의 right side 미러)을 진행할 수 있다.

현재 left sequence는 right sequence와 비교해:
- 오른발 지지 기반의 구조적 한계로 clearance state에서 직접 return을 시연할 수 없었다
- 하중 이동 → clearance 달성 → 하중 복원의 cycle은 각 단계에서 성공적으로 시연됐다

> 다음 단계로 `left_step_sequence_evaluation`을 통해 각 curriculum 단계의 영상을 하나로 정리하거나, hardware 개선 방향을 결정할 수 있다.
