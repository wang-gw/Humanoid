# 106. Weight Shift RL Training

## 목적

이번 단계의 목적은 standing 다음 curriculum인 `weight_shift_left` task에서 RL이 기준 controller 위에 유용한 보정을 학습하는지 확인하는 것이다.

핵심 질문:

1. 왼발 하중 비율을 목표값 `0.65`에 더 가깝게 만들 수 있는가?
2. 5초 동안 넘어지지 않고 유지되는가?
3. 하중 이동 개선이 자세 안정성 악화와 맞바뀌는가?

## 시작점

이전 단계 `105`에서 확인한 내용:

- standing PPO 학습 파이프라인은 정상 동작했다.
- 하지만 standing은 기준 controller가 이미 좋아서 20k PPO가 기준선을 이기지 못했다.
- 따라서 standing 반복보다 weight shift RL로 넘어가는 것이 더 의미 있다.

## 코드 변경

이번 단계에서 RL 환경과 스크립트에 다음 옵션을 추가했다.

파일:

- `envs/urdf_f_env.py`
- `scripts/train_urdf_f_ppo.py`
- `scripts/evaluate_urdf_f_policy.py`

추가 옵션:

| 옵션 | 의미 |
|---|---|
| `action_scale` | policy action이 관절 목표각에 주는 최대 보정 크기 |
| `action_penalty_weight` | action 자체를 작게 유지하는 penalty |
| `action_delta_penalty_weight` | 이전 action 대비 변화량을 작게 유지하는 penalty |

이번 weight-shift 실험 설정:

| 항목 | 값 |
|---|---:|
| action_scale | `0.15 rad` |
| action_penalty_weight | `0.05` |
| action_delta_penalty_weight | `0.02` |

이렇게 줄인 이유:

- standing 실험에서 PPO가 기준 controller를 약간 흔들었다.
- weight shift에서는 안정적인 기준 controller를 크게 망가뜨리지 않으면서 하중 비율만 보정하도록 제한해야 한다.

## 사용한 모델과 pose

모델:

- `envs/robots/urdf_f_link/URDF_F_link_virtual_com_gate_com_y005.xml`

pose:

- `configs/weight_shift_left065_contact_constrained_multipoint.json`

task:

- `weight_shift_left`

목표:

- left force ratio 약 `0.65`

## 기준선 평가: zero-policy

명령:

```bash
python3 scripts/evaluate_urdf_f_policy.py \
  --task weight_shift_left \
  --pose-json configs/weight_shift_left065_contact_constrained_multipoint.json \
  --policy zero \
  --steps 250 \
  --action-scale 0.15 \
  --action-penalty-weight 0.05 \
  --action-delta-penalty-weight 0.02 \
  --out-dir outputs/eval/urdf_f_weight_shift_baseline
```

산출물:

- `outputs/eval/urdf_f_weight_shift_baseline/weight_shift_left_zero_seed1/evaluation.mp4`
- `outputs/eval/urdf_f_weight_shift_baseline/weight_shift_left_zero_seed1/evaluation.gif`
- `outputs/eval/urdf_f_weight_shift_baseline/weight_shift_left_zero_seed1/timeline.csv`
- `outputs/eval/urdf_f_weight_shift_baseline/weight_shift_left_zero_seed1/summary.json`

결과 요약:

| 항목 | 값 |
|---|---:|
| 완료 step | 250 / 250 |
| sim time | 5.0 s |
| total reward | 521.10 |
| mean left force ratio | 0.711 |
| final left force ratio | 0.718 |
| mean abs error to 0.65 | 0.0833 |
| final abs error to 0.65 | 0.0681 |
| mean abs roll | 0.0220 rad |
| max abs roll | 0.1117 rad |
| max torque command | 8.83 Nm |
| 종료 이유 | 없음 |

판단:

- 기준 controller는 5초 동안 안정적으로 유지된다.
- 하지만 목표 left force ratio `0.65`보다 왼쪽으로 더 많이 실린다.
- 즉 RL이 개선할 여지가 있다.

## PPO 학습

명령:

```bash
python3 scripts/train_urdf_f_ppo.py \
  --task weight_shift_left \
  --pose-json configs/weight_shift_left065_contact_constrained_multipoint.json \
  --total-timesteps 30000 \
  --n-envs 4 \
  --max-episode-steps 250 \
  --action-scale 0.15 \
  --action-penalty-weight 0.05 \
  --action-delta-penalty-weight 0.02 \
  --run-name weight_shift_left_30k \
  --device cpu
```

실제 PPO rollout 단위 때문에 최종 학습 step은 `30,720` step이었다.

최종 학습 로그 요약:

| 항목 | 값 |
|---|---:|
| total_timesteps | 30,720 |
| ep_len_mean | 250 |
| ep_rew_mean | 457 |

산출물:

- `outputs/train/urdf_f/weight_shift_left_30k/config.json`
- `outputs/train/urdf_f/weight_shift_left_30k/ppo_policy.zip`
- `outputs/train/urdf_f/weight_shift_left_30k/vecnormalize.pkl`

판단:

- 학습 중 episode는 처음부터 끝까지 250 step을 채웠다.
- 넘어지는 문제는 없었다.
- 학습 로그 reward는 크게 오르지 않았지만, deterministic 평가에서 하중 비율 개선이 확인됐다.

## 학습 policy 평가

명령:

```bash
python3 scripts/evaluate_urdf_f_policy.py \
  --task weight_shift_left \
  --pose-json configs/weight_shift_left065_contact_constrained_multipoint.json \
  --policy ppo \
  --policy-path outputs/train/urdf_f/weight_shift_left_30k/ppo_policy.zip \
  --vecnormalize-path outputs/train/urdf_f/weight_shift_left_30k/vecnormalize.pkl \
  --steps 250 \
  --action-scale 0.15 \
  --action-penalty-weight 0.05 \
  --action-delta-penalty-weight 0.02 \
  --out-dir outputs/eval/urdf_f_weight_shift_30k
```

산출물:

- `outputs/eval/urdf_f_weight_shift_30k/weight_shift_left_ppo_seed1/evaluation.mp4`
- `outputs/eval/urdf_f_weight_shift_30k/weight_shift_left_ppo_seed1/evaluation.gif`
- `outputs/eval/urdf_f_weight_shift_30k/weight_shift_left_ppo_seed1/timeline.csv`
- `outputs/eval/urdf_f_weight_shift_30k/weight_shift_left_ppo_seed1/summary.json`

결과 요약:

| 항목 | 값 |
|---|---:|
| 완료 step | 250 / 250 |
| sim time | 5.0 s |
| total reward | 555.56 |
| mean left force ratio | 0.672 |
| final left force ratio | 0.671 |
| mean abs error to 0.65 | 0.0396 |
| final abs error to 0.65 | 0.0211 |
| mean abs roll | 0.0725 rad |
| max abs roll | 0.1200 rad |
| max torque command | 8.57 Nm |
| 종료 이유 | 없음 |

판단:

- RL policy는 5초 동안 안정적으로 유지됐다.
- left force ratio가 목표 `0.65`에 훨씬 가까워졌다.
- 그러나 roll 흔들림은 기준선보다 커졌다.

## Zero vs PPO 비교

| 항목 | zero-policy | PPO 30k | 판단 |
|---|---:|---:|---|
| 5초 유지 | 성공 | 성공 | 둘 다 통과 |
| total reward | 521.10 | 555.56 | PPO 개선 |
| mean left force ratio | 0.711 | 0.672 | PPO가 목표 0.65에 가까움 |
| final left force ratio | 0.718 | 0.671 | PPO가 목표 0.65에 가까움 |
| mean abs error to 0.65 | 0.0833 | 0.0396 | PPO 개선 |
| final abs error to 0.65 | 0.0681 | 0.0211 | PPO 개선 |
| mean abs roll | 0.0220 rad | 0.0725 rad | PPO 악화 |
| max abs roll | 0.1117 rad | 0.1200 rad | PPO 약간 악화 |
| max torque command | 8.83 Nm | 8.57 Nm | 비슷함 |

## 해석

고등학생도 이해할 수 있게 말하면 다음과 같다.

기준 controller는 로봇을 안전하게 세우고 왼쪽으로 하중을 옮긴다. 그런데 목표가 왼발 65%라면, 기준 controller는 실제로 약 71~72%까지 너무 많이 왼쪽으로 싣는다.

PPO는 이 문제를 조금 고쳤다. 학습 후에는 왼발 하중이 약 67%로 내려와 목표 65%에 가까워졌다.

하지만 공짜는 아니었다. 하중 비율을 맞추는 대신 몸통 roll 흔들림이 커졌다.

즉 이번 결과는 다음 뜻이다.

> RL은 기준 controller 위에서 하중 분배를 조정하는 방법을 실제로 학습했다. 다만 reward가 하중 비율을 더 강하게 보상하고 roll 안정성을 충분히 강하게 막지 않아서, 안정성을 일부 희생했다.

## 현재 결론

성공한 것:

- `weight_shift_left` RL 학습 실행
- PPO policy 저장
- 학습 전/후 영상 생성
- 목표 left force ratio 추종 개선
- 5초 동안 비낙상 유지

아직 부족한 것:

- roll 안정성
- target force ratio와 posture 안정성의 균형
- 다음 단계인 right unload / right clearance로 바로 넘어가기 전 reward 균형 조정

## 다음 조치

다음 단계는 두 가지 중 하나다.

### 선택 A: weight-shift reward 개선 후 재학습

수정 후보:

- roll/pitch penalty 증가
- force ratio target reward는 유지
- max roll이 `0.08 rad`를 넘으면 강한 penalty 추가
- action delta penalty 강화

목표:

- left force ratio는 `0.65~0.68`
- mean abs roll은 `0.03 rad` 이하

### 선택 B: right unload로 이동

이유:

- PPO가 하중 비율을 조정할 수 있다는 것은 확인했다.
- 다음 보행 준비 단계는 오른발 하중을 더 줄이는 것이다.

주의:

- 현재 PPO는 roll을 키우는 방식으로 하중 비율을 맞추는 경향이 있다.
- right unload로 이동하기 전 또는 이동하면서 roll penalty를 강화해야 한다.

현재 권장:

> 바로 right unload로 넘어가되, reward에 roll 안정성 penalty를 강화해서 진행한다.
