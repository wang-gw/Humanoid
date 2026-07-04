# 107. Right Unload RL Training

## 목적

이번 단계의 목적은 `weight_shift_left` 다음 curriculum인 `right_unload` task에서 RL이 오른발 하중을 더 줄일 수 있는지 확인하는 것이다.

핵심 질문:

1. 오른발 하중 비율을 줄일 수 있는가?
2. 5초 동안 넘어지지 않고 유지되는가?
3. 오른발 하중 감소가 실제 발 들기, 즉 접촉 해제로 이어지는가?

## 시작점

이전 단계 `106`의 결론:

- PPO는 left force ratio를 목표 `0.65`에 더 가깝게 만들었다.
- 하지만 roll 흔들림이 커졌다.
- 따라서 다음 단계에서는 roll/pitch 안정성 penalty를 강화해야 한다.

## 코드 변경

이번 단계에서 upright penalty를 옵션화했다.

파일:

- `envs/urdf_f_env.py`
- `scripts/train_urdf_f_ppo.py`
- `scripts/evaluate_urdf_f_policy.py`

추가 옵션:

| 옵션 | 의미 |
|---|---|
| `upright_penalty_weight` | roll/pitch penalty 가중치 |

이번 right-unload 실험 설정:

| 항목 | 값 |
|---|---:|
| action_scale | `0.12 rad` |
| upright_penalty_weight | `18.0` |
| action_penalty_weight | `0.08` |
| action_delta_penalty_weight | `0.03` |

이유:

- weight-shift PPO가 roll을 키우며 하중 비율을 맞추는 경향이 있었다.
- right-unload에서는 오른발 하중을 줄이되 roll/pitch collapse를 막아야 한다.

## 시작 pose 확인

처음에는 기존 unload pose를 기준선으로 확인했다.

### `right_unload_from_left065_contact_constrained_multipoint.json`

결과:

| 항목 | 값 |
|---|---:|
| 완료 step | 71 / 250 |
| 종료 이유 | pitch_limit |
| final pitch | 0.556 rad |
| final right force ratio | 0.0 |
| final right contacts | 0 |

해석:

- 오른발 하중은 빠지지만 pitch가 무너진다.
- 현재 COM +Y 모델과 이 pose/controller 조합은 안정적인 RL 시작점으로 부적합하다.

### `right_unload_left070_contact_constrained_multipoint.json`

결과:

| 항목 | 값 |
|---|---:|
| 완료 step | 38 / 250 |
| 종료 이유 | roll_limit |
| final roll | 0.563 rad |
| final right force ratio | 0.173 |

해석:

- 더 빨리 roll limit로 무너진다.
- RL 시작점으로 부적합하다.

### 선택한 시작점

최종적으로 안정적인 시작점으로 다음 pose를 사용했다.

- `configs/weight_shift_left065_contact_constrained_multipoint.json`

이 pose에서 right-unload task를 주고, policy가 오른발 하중을 더 줄이도록 학습했다.

## 기준선 평가: zero-policy

명령:

```bash
python3 scripts/evaluate_urdf_f_policy.py \
  --task right_unload \
  --pose-json configs/weight_shift_left065_contact_constrained_multipoint.json \
  --policy zero \
  --steps 250 \
  --action-scale 0.12 \
  --upright-penalty-weight 18.0 \
  --action-penalty-weight 0.08 \
  --action-delta-penalty-weight 0.03 \
  --out-dir outputs/eval/urdf_f_right_unload_from_weight_baseline
```

산출물:

- `outputs/eval/urdf_f_right_unload_from_weight_baseline/right_unload_zero_seed1/evaluation.mp4`
- `outputs/eval/urdf_f_right_unload_from_weight_baseline/right_unload_zero_seed1/evaluation.gif`
- `outputs/eval/urdf_f_right_unload_from_weight_baseline/right_unload_zero_seed1/timeline.csv`
- `outputs/eval/urdf_f_right_unload_from_weight_baseline/right_unload_zero_seed1/summary.json`

결과 요약:

| 항목 | 값 |
|---|---:|
| 완료 step | 250 / 250 |
| sim time | 5.0 s |
| total reward | 513.39 |
| mean right force ratio | 0.289 |
| final right force ratio | 0.282 |
| mean right contacts | 2.928 |
| final right contacts | 3 |
| mean abs roll | 0.0220 rad |
| max abs roll | 0.1117 rad |
| max torque command | 8.83 Nm |
| 종료 이유 | 없음 |

판단:

- 안정적으로 유지된다.
- 오른발 하중이 아직 약 28% 남아 있다.
- 오른발 접촉도 유지된다.

## PPO 학습

명령:

```bash
python3 scripts/train_urdf_f_ppo.py \
  --task right_unload \
  --pose-json configs/weight_shift_left065_contact_constrained_multipoint.json \
  --total-timesteps 30000 \
  --n-envs 4 \
  --max-episode-steps 250 \
  --action-scale 0.12 \
  --upright-penalty-weight 18.0 \
  --action-penalty-weight 0.08 \
  --action-delta-penalty-weight 0.03 \
  --run-name right_unload_from_weight_30k \
  --device cpu
```

실제 PPO rollout 단위 때문에 최종 학습 step은 `30,720` step이었다.

최종 학습 로그 요약:

| 항목 | 값 |
|---|---:|
| total_timesteps | 30,720 |
| ep_len_mean | 250 |
| ep_rew_mean | 517 |

산출물:

- `outputs/train/urdf_f/right_unload_from_weight_30k/config.json`
- `outputs/train/urdf_f/right_unload_from_weight_30k/ppo_policy.zip`
- `outputs/train/urdf_f/right_unload_from_weight_30k/vecnormalize.pkl`

판단:

- 학습 중 episode는 끝까지 유지됐다.
- 평균 reward는 `503 -> 517` 수준으로 증가했다.

## 학습 policy 평가

명령:

```bash
python3 scripts/evaluate_urdf_f_policy.py \
  --task right_unload \
  --pose-json configs/weight_shift_left065_contact_constrained_multipoint.json \
  --policy ppo \
  --policy-path outputs/train/urdf_f/right_unload_from_weight_30k/ppo_policy.zip \
  --vecnormalize-path outputs/train/urdf_f/right_unload_from_weight_30k/vecnormalize.pkl \
  --steps 250 \
  --action-scale 0.12 \
  --upright-penalty-weight 18.0 \
  --action-penalty-weight 0.08 \
  --action-delta-penalty-weight 0.03 \
  --out-dir outputs/eval/urdf_f_right_unload_30k
```

산출물:

- `outputs/eval/urdf_f_right_unload_30k/right_unload_ppo_seed1/evaluation.mp4`
- `outputs/eval/urdf_f_right_unload_30k/right_unload_ppo_seed1/evaluation.gif`
- `outputs/eval/urdf_f_right_unload_30k/right_unload_ppo_seed1/timeline.csv`
- `outputs/eval/urdf_f_right_unload_30k/right_unload_ppo_seed1/summary.json`

결과 요약:

| 항목 | 값 |
|---|---:|
| 완료 step | 250 / 250 |
| sim time | 5.0 s |
| total reward | 574.57 |
| mean right force ratio | 0.233 |
| final right force ratio | 0.201 |
| mean right contacts | 3.484 |
| final right contacts | 4 |
| final right clearance | 0.00031 m |
| mean abs roll | 0.0503 rad |
| max abs roll | 0.1151 rad |
| max torque command | 8.58 Nm |
| 종료 이유 | 없음 |

판단:

- 오른발 하중 비율은 줄었다.
- 5초 동안 안정적으로 유지됐다.
- 하지만 오른발 접촉 수는 줄지 않았다.
- 즉 “unload”는 개선됐지만 “toe-off/발 들기”는 아직 아니다.

## Zero vs PPO 비교

| 항목 | zero-policy | PPO 30k | 판단 |
|---|---:|---:|---|
| 5초 유지 | 성공 | 성공 | 둘 다 통과 |
| total reward | 513.39 | 574.57 | PPO 개선 |
| mean right force ratio | 0.289 | 0.233 | PPO 개선 |
| final right force ratio | 0.282 | 0.201 | PPO 개선 |
| mean right contacts | 2.928 | 3.484 | PPO 악화 |
| final right contacts | 3 | 4 | PPO 악화 |
| mean abs roll | 0.0220 rad | 0.0503 rad | PPO 악화 |
| max abs roll | 0.1117 rad | 0.1151 rad | 비슷하지만 PPO가 약간 큼 |
| max torque command | 8.83 Nm | 8.58 Nm | 비슷함 |

## 해석

고등학생도 이해할 수 있게 말하면 다음과 같다.

기준 controller는 오른발에도 아직 체중이 꽤 실려 있다. PPO는 몸을 조금 더 왼쪽으로 쓰면서 오른발에 실리는 무게를 줄였다.

하지만 오른발이 실제로 바닥에서 떨어진 것은 아니다. 오히려 오른발 contact pad 접촉 수는 늘었다. 즉 발 전체에 걸리는 힘은 줄었지만, 접촉 자체는 계속 남아 있다.

이 차이가 중요하다.

- unload: 발에 실린 무게를 줄이는 것
- toe-off / clearance: 발이 실제로 바닥에서 떨어지는 것

이번 단계는 unload는 개선했지만 clearance는 만들지 못했다.

## 현재 결론

성공한 것:

- right-unload RL 학습 실행
- 오른발 하중 비율 감소
- 5초 안정 유지
- 학습 전/후 영상 생성

아직 부족한 것:

- 오른발 접촉 해제
- 오른발 clearance 증가
- roll 증가 억제

## 다음 조치

다음 curriculum은 `right_clearance`다.

하지만 바로 clearance만 reward로 주면, policy가 roll을 더 키우거나 발 접촉을 이상하게 유지한 채 clearance 수치만 맞출 수 있다.

따라서 다음 단계 reward에는 다음 항목이 필요하다.

1. 오른발 force 감소 reward
2. 오른발 contact 수 감소 reward
3. 오른발 clearance reward
4. 왼발 stance contact 유지 reward
5. roll/pitch 강한 penalty
6. action/action delta penalty

현재 권장:

> `right_clearance`로 넘어가되, 오른발 contact 해제 항목을 reward에 명시적으로 추가한다.
