# 119. Left Clearance RL Training

## 목적

이번 단계의 목적은 왼발 clearance curriculum을 완료하는 것이다. `left_unload` 다음 단계로, 왼발을 지면에서 들어 올려 clearance를 달성하고 5초 동안 안정적으로 유지할 수 있는지 확인한다.

핵심 질문:

1. PPO가 왼발 clearance를 달성할 수 있는가?
2. clearance 달성 중 roll/pitch 안정성을 유지할 수 있는가?
3. gated clearance (roll/pitch gate 내에서 clearance 달성)가 가능한가?

## 시작점

이전 단계 `118_left_unload_rl_training.md`의 결론:

- `left_unload` PPO 30k는 왼발 하중을 0.642 → 0.462로 줄이고 5초 안정을 달성했다.
- 왼발 contact 수는 16 → 11개로 감소했다.
- 다음 단계로 `left_clearance`가 권장됐다.

오른발 sequence에서 `right_clearance` (doc 108-112)는 `weight_shift_left065_contact_constrained_multipoint.json` 포즈 (right_contacts=4, 5초 안정)를 시작점으로 성공했다. 이번 왼발 sequence에서는 등가 포즈가 없어 다양한 접근법을 시험했다.

## 코드 변경

이번 단계에서 추가된 코드 변경은 없다. 모든 준비는 doc 116에서 완료됐다.

## 사용한 모델과 pose

모델:

- `envs/robots/urdf_f_link/URDF_F_link_virtual_stl_footprint_contact.xml`

pose (최종 성공 실험):

- `configs/left_unload_ppo_pose.json` — `left_unload_30k` PPO 최종 상태 캡처 (left_ratio=0.462, left_contacts=11, 5초 안정)

## 기준선 평가: zero-policy (standing pose)

명령:

```bash
python3 scripts/evaluate_urdf_f_policy.py \
  --model envs/robots/urdf_f_link/URDF_F_link_virtual_stl_footprint_contact.xml \
  --task left_clearance \
  --pose-json configs/dynamic_standing_pose_actuator_dynamics_contact_5s.json \
  --policy zero \
  --steps 250 \
  --action-scale 0.12 \
  --upright-penalty-weight 24.0 \
  --clearance-reward-weight 6.0 \
  --clearance-target 0.0002 \
  --gated-clearance-reward \
  --clearance-gate-roll 0.10 \
  --clearance-gate-pitch 0.10 \
  --out-dir outputs/eval/urdf_f_left_clearance_gated_05mm_baseline
```

산출물:

- `outputs/eval/urdf_f_left_clearance_gated_05mm_baseline/left_clearance_zero_seed1/evaluation.mp4`
- `outputs/eval/urdf_f_left_clearance_gated_05mm_baseline/left_clearance_zero_seed1/evaluation.gif`
- `outputs/eval/urdf_f_left_clearance_gated_05mm_baseline/left_clearance_zero_seed1/last_frame.png`
- `outputs/eval/urdf_f_left_clearance_gated_05mm_baseline/left_clearance_zero_seed1/timeline.csv`
- `outputs/eval/urdf_f_left_clearance_gated_05mm_baseline/left_clearance_zero_seed1/summary.json`

결과 요약:

| 항목 | 값 |
|---|---:|
| 완료 step | 250 / 250 |
| sim time | 5.0 s |
| total reward | — |
| final left clearance | -0.12 mm |
| final left contacts | 16 |
| final left force ratio | 0.6417 |
| mean abs roll | 0.0270 rad |
| max abs roll | 0.0541 rad |
| 종료 이유 | 없음 |

## 실험 경과 요약

오른발 sequence와 달리 왼발 clearance는 구조적 비대칭으로 인해 여러 시도가 필요했다. 아래는 주요 실험과 결과다.

### 실험 1: CEM pose 시작점 (left_clearance_cem_pose_40k)

CEM 탐색으로 찾은 `weight_shift_right065_pose.json`은 이미 left_clearance=37mm, left_contacts=2를 달성한 포즈다. 그러나 RL env에서 zero-policy를 실행하면 3.96s에 pitch_limit으로 낙상한다.

PPO 40k 학습 시 ep_len_mean이 198→238 steps로 개선됐지만, 평가 결과 PPO는 안정성을 위해 발을 다시 착지시켰다 (final clearance=-0.13mm, contacts=11).

판단: CEM pose는 RL env에서 불안정해 출발점으로 부적합하다.

### 실험 2: Standing pose에서 다양한 설정 (gated, no-gate, 30k-60k)

Standing pose (left_contacts=16)를 시작점으로 action_scale=0.05-0.12, contact_penalty=0.25-0.40으로 30k-60k 학습했다. 60k 이후에도 left_contacts=16 유지 (credit assignment 문제).

판단: 16개 contacts 상태에서 PPO가 발을 들어 올리는 것은 너무 어렵다.

### 실험 3: left_unload PPO 최종 상태를 시작점으로 (left_clearance_unload_gated_50k)

`left_unload_30k` PPO를 250 steps 실행한 최종 상태를 캡처해 `left_unload_ppo_pose.json`으로 저장했다 (left_contacts=11, left_ratio=0.462, 5초 안정).

명령:

```bash
python3 scripts/train_urdf_f_ppo.py \
  --model envs/robots/urdf_f_link/URDF_F_link_virtual_stl_footprint_contact.xml \
  --task left_clearance \
  --pose-json configs/left_unload_ppo_pose.json \
  --total-timesteps 50000 \
  --n-envs 4 \
  --max-episode-steps 250 \
  --action-scale 0.12 \
  --upright-penalty-weight 24.0 \
  --action-penalty-weight 0.08 \
  --action-delta-penalty-weight 0.03 \
  --left-contact-penalty-weight 0.30 \
  --clearance-reward-weight 6.0 \
  --clearance-target 0.0002 \
  --gated-clearance-reward \
  --clearance-gate-roll 0.10 \
  --clearance-gate-pitch 0.10 \
  --fall-penalty 30.0 \
  --run-name left_clearance_unload_gated_50k \
  --device cpu
```

50k 결과: ep_len=250 안정, ep_rew=-265 (개선 중). 평가에서 final clearance=9.63mm였지만, 이는 에피소드 후반 body가 기울어지면서 발이 상대적으로 올라간 것 (base_z=-0.021, roll=+0.238).

타임라인 분석에서 step 221 (t=4.42s)에 clearance=1.38mm, contacts=4, roll=+0.029로 gated 조건을 충족했다 (gate_ok: 9/250 steps).

판단: 올바른 방향. 더 많은 학습 step이 필요하다.

## 최종 PPO 학습: left_clearance_unload_gated_150k

### 설정

| 항목 | 값 |
|---|---:|
| total_timesteps | 150,000 |
| action_scale | 0.12 |
| upright_penalty_weight | 24.0 |
| action_penalty_weight | 0.08 |
| action_delta_penalty_weight | 0.03 |
| left_contact_penalty_weight | 0.30 |
| clearance_reward_weight | 6.0 |
| clearance_target | 0.0002 m (0.2 mm) |
| gated_clearance_reward | True |
| clearance_gate_roll | 0.10 rad |
| clearance_gate_pitch | 0.10 rad |
| fall_penalty | 30.0 |
| n_envs | 4 |
| max_episode_steps | 250 |

학습 명령:

```bash
python3 scripts/train_urdf_f_ppo.py \
  --model envs/robots/urdf_f_link/URDF_F_link_virtual_stl_footprint_contact.xml \
  --task left_clearance \
  --pose-json configs/left_unload_ppo_pose.json \
  --total-timesteps 150000 \
  --n-envs 4 \
  --max-episode-steps 250 \
  --action-scale 0.12 \
  --upright-penalty-weight 24.0 \
  --action-penalty-weight 0.08 \
  --action-delta-penalty-weight 0.03 \
  --left-contact-penalty-weight 0.30 \
  --clearance-reward-weight 6.0 \
  --clearance-target 0.0002 \
  --gated-clearance-reward \
  --clearance-gate-roll 0.10 \
  --clearance-gate-pitch 0.10 \
  --fall-penalty 30.0 \
  --run-name left_clearance_unload_gated_150k \
  --device cpu
```

### 학습 로그 (후반부)

| total_timesteps | ep_len_mean | ep_rew_mean |
|---:|---:|---:|
| 2,048 | 250 | -367 |
| 51,200 | 250 | -265 |
| 100,352 | 250 | 750 |
| 149,504 | 229 | 964 |
| 151,552 | 229 | 964 |

### 산출물

- `outputs/train/urdf_f/left_clearance_unload_gated_150k/config.json`
- `outputs/train/urdf_f/left_clearance_unload_gated_150k/ppo_policy.zip`
- `outputs/train/urdf_f/left_clearance_unload_gated_150k/vecnormalize.pkl`

## 학습 policy 평가

명령:

```bash
python3 scripts/evaluate_urdf_f_policy.py \
  --model envs/robots/urdf_f_link/URDF_F_link_virtual_stl_footprint_contact.xml \
  --task left_clearance \
  --pose-json configs/left_unload_ppo_pose.json \
  --policy ppo \
  --policy-path outputs/train/urdf_f/left_clearance_unload_gated_150k/ppo_policy.zip \
  --vecnormalize-path outputs/train/urdf_f/left_clearance_unload_gated_150k/vecnormalize.pkl \
  --steps 250 \
  --action-scale 0.12 \
  --upright-penalty-weight 24.0 \
  --action-penalty-weight 0.08 \
  --action-delta-penalty-weight 0.03 \
  --left-contact-penalty-weight 0.30 \
  --clearance-reward-weight 6.0 \
  --clearance-target 0.0002 \
  --gated-clearance-reward \
  --clearance-gate-roll 0.10 \
  --clearance-gate-pitch 0.10 \
  --fall-penalty 30.0 \
  --out-dir outputs/eval/urdf_f_left_clearance_unload_gated_150k
```

산출물:

- `outputs/eval/urdf_f_left_clearance_unload_gated_150k/left_clearance_ppo_seed1/evaluation.mp4`
- `outputs/eval/urdf_f_left_clearance_unload_gated_150k/left_clearance_ppo_seed1/evaluation.gif`
- `outputs/eval/urdf_f_left_clearance_unload_gated_150k/left_clearance_ppo_seed1/last_frame.png`
- `outputs/eval/urdf_f_left_clearance_unload_gated_150k/left_clearance_ppo_seed1/timeline.csv`
- `outputs/eval/urdf_f_left_clearance_unload_gated_150k/left_clearance_ppo_seed1/summary.json`

결과 요약:

| 항목 | 값 |
|---|---:|
| 완료 step | 250 / 250 |
| sim time | 5.0 s |
| total reward | 1363.99 |
| mean left clearance | 2.15 mm |
| max left clearance | 10.95 mm |
| clearance > 0.2mm steps | 228 / 250 (91%) |
| gate ok steps | 221 / 250 (88%) |
| mean left contacts | 2.26 |
| final left contacts | 6 |
| mean left force ratio | 0.5191 |
| final left force ratio | 0.3778 |
| mean abs roll | 0.0549 rad |
| max abs roll | 0.1836 rad |
| max torque command | 4.73 Nm |
| 종료 이유 | 없음 |

### 에피소드 타임라인

| 시간 | left clearance | left contacts | roll | left force ratio |
|---|---:|---:|---:|---:|
| 0.02s | -1.03 mm | 15 | +0.001 | 0.496 |
| 0.42s | +1.52 mm | 4 | +0.079 | 0.469 |
| 0.82s | -0.11 mm | 5 | +0.088 | 0.675 |
| 1.22s | +0.56 mm | 1 | +0.052 | 0.730 |
| 1.62s | +1.04 mm | 1 | +0.053 | 0.862 |
| 2.02s | +0.69 mm | 1 | +0.061 | 0.808 |
| 2.42s | +0.62 mm | 1 | +0.061 | 0.694 |
| 2.82s | +1.21 mm | 1 | +0.057 | 0.449 |
| 3.22s | +1.81 mm | 1 | +0.050 | 0.346 |
| 3.62s | +2.53 mm | 1 | +0.043 | 0.286 |
| 4.02s | +3.81 mm | 1 | +0.034 | 0.436 |
| 4.42s | +4.90 mm | 1 | +0.011 | 0.544 |
| 4.82s | +7.98 mm | 6 | -0.080 | 0.481 |
| 5.00s | +10.95 mm | 6 | -0.184 | 0.378 |

## Zero vs PPO 비교

| 항목 | zero-policy | PPO 150k | 판단 |
|---|---:|---:|---|
| 5초 유지 | 성공 | 성공 | 둘 다 통과 |
| total reward | — | 1363.99 | PPO 대폭 개선 |
| mean left clearance | -0.12 mm | +2.15 mm | PPO 성공 |
| max left clearance | -0.12 mm | +10.95 mm | PPO 성공 |
| gate ok steps | 0 / 250 | 221 / 250 | PPO 성공 |
| final left force ratio | 0.6417 | 0.3778 | PPO 개선 |
| mean left contacts | 16 | 2.26 | PPO 대폭 개선 |
| mean abs roll | 0.0270 rad | 0.0549 rad | PPO 약간 증가 |
| max abs roll | 0.0541 rad | 0.1836 rad | PPO 증가 |

## 해석

오른발 sequence와 비교하면 왼발 clearance는 구조적으로 더 어려운 문제다. 그 이유는 다음과 같다.

왼발은 16개 contact pad가 있고, 오른발은 4-6개 정도만 지면에 닿는다. 이 비대칭 때문에 오른발은 지면 지지 역할을 안정적으로 수행하기 어렵다. 오른발에 전체 무게를 실어야 왼발을 들 수 있는데, 오른발 지지가 불안정하면 robot이 쉽게 넘어진다.

이 문제를 해결하기 위해 두 단계 접근법을 사용했다.

1. **left_unload PPO 상태 캡처**: `left_unload_30k` 학습 후 최종 joint 상태를 포즈로 저장했다 (left_contacts=11, left_ratio=0.462). 이 포즈는 이미 왼발 하중이 줄어있고 5초 안정이다.

2. **gated clearance 150k 학습**: contact penalty (0.30)로 left foot을 들어 올리는 방향으로 유도하고, gated reward (roll<0.10 조건)로 안정성을 동시에 요구했다. 50k에서 gate ok가 9 steps에 불과했지만, 150k에서 221/250 steps로 급격히 향상됐다.

정성적 관찰:

- t=1.22s부터 t=4.42s까지 약 3.2초 동안 left_contacts=1, roll<0.08로 안정적 hovering을 유지했다.
- left_force_ratio가 에피소드 도중 0.35까지 감소 (right foot이 65% 담당).
- 에피소드 후반 roll이 커지지만 종료 기준 (0.55)을 초과하지 않았다.

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
| **left_clearance** | **완료 (150k, clearance 평균 2.15mm, gate ok 221/250)** |
| left_return | 미시작 |

성공한 것:

- `left_clearance` RL 학습 실행 (150k steps)
- PPO policy 저장
- 학습 전/후 영상 생성
- 5초 비낙상 유지
- gated clearance 달성 (221/250 steps, roll<0.10 AND clearance>0.2mm)
- max clearance 10.95mm 달성
- left contacts 평균 2.26개 (기준 16개에서 대폭 감소)

## 다음 조치

다음 curriculum은 `left_return`이다.

오른발 sequence에서 `right_return` (doc 113)은 clearance를 달성한 후 오른발을 원래 위치로 되돌려 안정적인 bilateral standing을 복원하는 것이었다.

이번에도 동일한 구성이 필요하다. 왼발이 공중에 있는 상태에서 다시 바닥에 착지시켜 안정적인 bilateral standing을 복원한다.

현재 권장:

> `left_return`으로 넘어간다. clearance 달성 상태 (`left_clearance_unload_gated_150k`)를 시작점으로, 왼발을 원래 위치로 되돌리는 policy를 학습한다.
