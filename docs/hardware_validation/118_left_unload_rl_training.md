# 118. Left Unload RL Training

## 목적

이번 단계의 목적은 `weight_shift_right` 다음 curriculum인 `left_unload` task에서 RL이 왼발 하중을 더 줄일 수 있는지 확인하는 것이다.

핵심 질문:

1. 왼발 하중 비율을 줄일 수 있는가?
2. 5초 동안 넘어지지 않고 유지되는가?
3. 왼발 하중 감소가 실제 발 들기, 즉 접촉 해제로 이어지는가?

## 시작점

이전 단계 `117_weight_shift_right_rl_training.md`의 결론:

- `weight_shift_right` PPO 40k는 5초 안정 유지와 right_force_ratio `0.358 → 0.610` 개선을 달성했다.
- 다음 조치로 선택 B (`left_unload`로 직접 이동)를 권장했다.
- standing pose가 RL 환경에서 5초 안정인 것을 이미 확인했다.

오른발 sequence (`right_unload`, doc 107)에서 `weight_shift_left065` pose를 시작점으로 쓴 것처럼, 이번에는 CEM이 탐색한 `weight_shift_right065_pose.json`이 자연스러운 대응이다. 그러나 doc 117에서 해당 pose가 RL env에서 pitch_limit으로 낙상하는 것을 확인했다. 따라서 이번에도 안정성이 검증된 standing pose를 시작점으로 사용했다.

## 코드 변경

이번 단계에서 추가된 코드 변경은 없다. 모든 준비는 doc 116에서 완료됐다.

## 사용한 모델과 pose

모델:

- `envs/robots/urdf_f_link/URDF_F_link_virtual_stl_footprint_contact.xml`

pose:

- `configs/dynamic_standing_pose_actuator_dynamics_contact_5s.json`

task:

- `left_unload`

목표:

- left force `< 30 N` (왼발 하중 거의 제거)
- 오른발 contact 유지

## 기준선 평가: zero-policy

명령:

```bash
python3 scripts/evaluate_urdf_f_policy.py \
  --model envs/robots/urdf_f_link/URDF_F_link_virtual_stl_footprint_contact.xml \
  --task left_unload \
  --pose-json configs/dynamic_standing_pose_actuator_dynamics_contact_5s.json \
  --policy zero \
  --steps 250 \
  --action-scale 0.12 \
  --upright-penalty-weight 18.0 \
  --action-penalty-weight 0.08 \
  --action-delta-penalty-weight 0.03 \
  --out-dir outputs/eval/urdf_f_left_unload_baseline
```

산출물:

- `outputs/eval/urdf_f_left_unload_baseline/left_unload_zero_seed1/evaluation.mp4`
- `outputs/eval/urdf_f_left_unload_baseline/left_unload_zero_seed1/evaluation.gif`
- `outputs/eval/urdf_f_left_unload_baseline/left_unload_zero_seed1/last_frame.png`
- `outputs/eval/urdf_f_left_unload_baseline/left_unload_zero_seed1/timeline.csv`
- `outputs/eval/urdf_f_left_unload_baseline/left_unload_zero_seed1/summary.json`

결과 요약:

| 항목 | 값 |
|---|---:|
| 완료 step | 250 / 250 |
| sim time | 5.0 s |
| total reward | 172.57 |
| mean left force ratio | 0.6208 |
| final left force ratio | 0.6417 |
| mean left contacts | 14.50 |
| final left contacts | 16 |
| mean abs roll | 0.0260 rad |
| max abs roll | 0.0541 rad |
| max torque command | 3.18 Nm |
| 종료 이유 | 없음 |

판단:

- 기준 controller는 5초 동안 안정적으로 유지된다.
- 하지만 standing pose에서 왼발 하중이 자연스럽게 약 64%를 차지한다.
- 왼발 contact pad 수도 16개로 높다.
- RL이 개선해야 할 여지가 크다.

## PPO 학습

설정:

| 항목 | 값 |
|---|---:|
| total_timesteps | 30,000 |
| action_scale | 0.12 |
| upright_penalty_weight | 18.0 |
| action_penalty_weight | 0.08 |
| action_delta_penalty_weight | 0.03 |
| n_envs | 4 |
| max_episode_steps | 250 |

학습 명령:

```bash
python3 scripts/train_urdf_f_ppo.py \
  --model envs/robots/urdf_f_link/URDF_F_link_virtual_stl_footprint_contact.xml \
  --task left_unload \
  --pose-json configs/dynamic_standing_pose_actuator_dynamics_contact_5s.json \
  --total-timesteps 30000 \
  --n-envs 4 \
  --max-episode-steps 250 \
  --action-scale 0.12 \
  --upright-penalty-weight 18.0 \
  --action-penalty-weight 0.08 \
  --action-delta-penalty-weight 0.03 \
  --run-name left_unload_from_standing_30k \
  --device cpu
```

실제 PPO rollout 단위 때문에 최종 학습 step은 `30,720` step이었다.

최종 학습 로그 요약:

| 항목 | 값 |
|---|---:|
| total_timesteps | 30,720 |
| ep_len_mean | 250 |
| ep_rew_mean | 221 |
| fps | 326 |

산출물:

- `outputs/train/urdf_f/left_unload_from_standing_30k/config.json`
- `outputs/train/urdf_f/left_unload_from_standing_30k/ppo_policy.zip`
- `outputs/train/urdf_f/left_unload_from_standing_30k/vecnormalize.pkl`

판단:

- 학습 중 episode는 끝까지 유지됐다.
- ep_rew_mean이 초반 대비 상승하여 policy update가 작동했다.

## 학습 policy 평가

명령:

```bash
python3 scripts/evaluate_urdf_f_policy.py \
  --model envs/robots/urdf_f_link/URDF_F_link_virtual_stl_footprint_contact.xml \
  --task left_unload \
  --pose-json configs/dynamic_standing_pose_actuator_dynamics_contact_5s.json \
  --policy ppo \
  --policy-path outputs/train/urdf_f/left_unload_from_standing_30k/ppo_policy.zip \
  --vecnormalize-path outputs/train/urdf_f/left_unload_from_standing_30k/vecnormalize.pkl \
  --steps 250 \
  --action-scale 0.12 \
  --upright-penalty-weight 18.0 \
  --action-penalty-weight 0.08 \
  --action-delta-penalty-weight 0.03 \
  --out-dir outputs/eval/urdf_f_left_unload_30k
```

산출물:

- `outputs/eval/urdf_f_left_unload_30k/left_unload_ppo_seed1/evaluation.mp4`
- `outputs/eval/urdf_f_left_unload_30k/left_unload_ppo_seed1/evaluation.gif`
- `outputs/eval/urdf_f_left_unload_30k/left_unload_ppo_seed1/last_frame.png`
- `outputs/eval/urdf_f_left_unload_30k/left_unload_ppo_seed1/timeline.csv`
- `outputs/eval/urdf_f_left_unload_30k/left_unload_ppo_seed1/summary.json`

결과 요약:

| 항목 | 값 |
|---|---:|
| 완료 step | 250 / 250 |
| sim time | 5.0 s |
| total reward | 344.47 |
| mean left force ratio | 0.4695 |
| final left force ratio | 0.4625 |
| mean left contacts | 10.71 |
| final left contacts | 11 |
| final left clearance | ~0 m |
| mean abs roll | 0.0148 rad |
| max abs roll | 0.0376 rad |
| max torque command | 3.01 Nm |
| 종료 이유 | 없음 |

판단:

- PPO policy도 5초 동안 안정적으로 유지됐다.
- 왼발 하중 비율이 줄었다.
- roll이 기준선보다 오히려 개선됐다.
- 하지만 왼발 contact 수는 11개로 여전히 높다.

## Zero vs PPO 비교

| 항목 | zero-policy | PPO 30k | 판단 |
|---|---:|---:|---|
| 5초 유지 | 성공 | 성공 | 둘 다 통과 |
| total reward | 172.57 | 344.47 | PPO 개선 |
| mean left force ratio | 0.6208 | 0.4695 | PPO 개선 |
| final left force ratio | 0.6417 | 0.4625 | PPO 개선 |
| mean left contacts | 14.50 | 10.71 | PPO 개선 |
| final left contacts | 16 | 11 | PPO 개선 |
| mean abs roll | 0.0260 rad | 0.0148 rad | PPO 개선 |
| max abs roll | 0.0541 rad | 0.0376 rad | PPO 개선 |
| mean abs pitch | 0.0028 rad | 0.0041 rad | PPO 약간 악화 |
| max torque command | 3.18 Nm | 3.01 Nm | 비슷함 |

## 해석

고등학생도 이해할 수 있게 말하면 다음과 같다.

이 로봇은 원래 왼쪽에 더 많은 무게가 실린다. 기준 controller만 쓰면 왼발에 전체 체중의 약 64%가 실리고, 접촉 패드도 16개가 전부 닿아 있다.

PPO는 이 문제를 상당히 개선했다. 왼발 하중이 약 46%로 줄었고, 접촉 패드 수도 11개로 감소했다. 그리고 몸이 더 안정적으로 유지됐다 (roll이 더 작아졌다).

오른발 sequence의 `right_unload`와 비교하면, 이번 결과는 오히려 더 좋다. `right_unload`에서는 하중이 줄었지만 roll이 커졌는데, 이번에는 하중이 줄면서 roll도 함께 개선됐다.

하지만 아직 부족한 것이 있다. 왼발이 실제로 바닥에서 완전히 떨어진 것은 아니다. 접촉 수는 11개로 줄었지만, 여전히 지면에 닿아 있다.

이번 결과의 정확한 의미는 다음이다.

> RL은 왼발 하중을 줄이는 방향으로 무게를 이동시키는 것을 학습했다. 다만 왼발이 바닥에서 완전히 떨어지는 clearance 단계는 아직 아니다.

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
| **left_unload** | **30k 완료, left_ratio 0.642 → 0.462** |
| left_clearance | 미시작 |
| left_return | 미시작 |

성공한 것:

- `left_unload` RL 학습 실행
- PPO policy 저장
- 학습 전/후 영상 생성
- 5초 비낙상 유지
- left force ratio 0.642 → 0.462 개선
- roll/pitch 안정성 개선

아직 부족한 것:

- 왼발 contact 완전 해제 (현재 11개 접촉 중)
- 왼발 clearance (지면에서 완전히 떨어지기)
- left_unload 조건 완전 충족 (left_force < 30 N)

## 다음 조치

다음 curriculum은 `left_clearance`다.

오른발 sequence에서 `right_clearance` (doc 108)로 넘어갈 때 reward에 다음 항목을 명시적으로 추가했다.

- clearance reward (왼발 높이)
- left contact penalty (왼발 접촉 수 감소)
- 강한 roll/pitch penalty

이번에도 동일한 구성이 필요하다.

현재 권장:

> `left_clearance`로 넘어간다. clearance reward와 left contact penalty를 추가하고, 안정 조건부 clearance (gated reward)를 적용한다. 시작 pose는 standing pose로 유지한다.
