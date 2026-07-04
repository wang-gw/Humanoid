# 108. Right Clearance RL Training

## 목적

이번 단계의 목적은 `right_unload` 다음 curriculum인 `right_clearance` task에서 RL이 오른발을 실제로 들어 올릴 수 있는지 확인하는 것이다.

핵심 질문:

1. 오른발 clearance를 `2 mm` 이상 만들 수 있는가?
2. 오른발 contact 수를 줄일 수 있는가?
3. 그 상태를 유지하면서 roll/pitch 안정성을 지킬 수 있는가?

## 시작점

이전 단계 `107`의 결론:

- PPO는 오른발 하중 비율을 줄였다.
- 하지만 오른발 contact 수는 줄지 않았다.
- 즉 unload는 개선됐지만 toe-off/clearance는 아직 아니었다.

따라서 이번 단계에서는 reward에 다음 항목을 명시적으로 추가했다.

- 오른발 clearance reward
- 오른발 contact penalty
- 강한 roll/pitch penalty

## 코드 변경

파일:

- `envs/urdf_f_env.py`
- `scripts/train_urdf_f_ppo.py`
- `scripts/evaluate_urdf_f_policy.py`

추가 옵션:

| 옵션 | 의미 |
|---|---|
| `right_contact_penalty_weight` | 오른발 contact 수에 대한 penalty |
| `clearance_reward_weight` | 오른발 clearance reward 가중치 |
| `clearance_target` | 목표 clearance |

## 공통 설정

모델:

- `envs/robots/urdf_f_link/URDF_F_link_virtual_com_gate_com_y005.xml`

pose:

- `configs/weight_shift_left065_contact_constrained_multipoint.json`

task:

- `right_clearance`

목표:

- 오른발 clearance `>= 0.002 m`
- 오른발 contact 감소
- roll/pitch 안정 유지

## 기준선 평가: zero-policy

명령:

```bash
python3 scripts/evaluate_urdf_f_policy.py \
  --task right_clearance \
  --pose-json configs/weight_shift_left065_contact_constrained_multipoint.json \
  --policy zero \
  --steps 250 \
  --action-scale 0.10 \
  --upright-penalty-weight 24.0 \
  --action-penalty-weight 0.10 \
  --action-delta-penalty-weight 0.04 \
  --right-contact-penalty-weight 0.25 \
  --clearance-reward-weight 4.0 \
  --clearance-target 0.002 \
  --out-dir outputs/eval/urdf_f_right_clearance_baseline
```

산출물:

- `outputs/eval/urdf_f_right_clearance_baseline/right_clearance_zero_seed1/evaluation.mp4`
- `outputs/eval/urdf_f_right_clearance_baseline/right_clearance_zero_seed1/evaluation.gif`
- `outputs/eval/urdf_f_right_clearance_baseline/right_clearance_zero_seed1/timeline.csv`
- `outputs/eval/urdf_f_right_clearance_baseline/right_clearance_zero_seed1/summary.json`

결과 요약:

| 항목 | 값 |
|---|---:|
| 완료 step | 250 / 250 |
| 종료 이유 | 없음 |
| final right clearance | 0.00023 m |
| max right clearance | 0.01593 m |
| final right contacts | 3 |
| mean right contacts | 2.928 |
| mean abs roll | 0.0220 rad |
| max abs roll | 0.1117 rad |
| combined gate longest | 0.64 s |

해석:

- 최종 상태에서는 clearance가 거의 없다.
- 다만 transient하게 2mm 이상 clearance가 생기는 구간은 있다.
- 안정성은 좋지만, 지속적인 발 들기는 아니다.

## 실험 1: 공격형 clearance PPO

설정:

| 항목 | 값 |
|---|---:|
| total_timesteps | 40,960 |
| action_scale | 0.10 |
| upright_penalty_weight | 24.0 |
| action_penalty_weight | 0.10 |
| action_delta_penalty_weight | 0.04 |
| right_contact_penalty_weight | 0.25 |
| clearance_reward_weight | 4.0 |
| clearance_target | 0.002 m |

학습 명령:

```bash
python3 scripts/train_urdf_f_ppo.py \
  --task right_clearance \
  --pose-json configs/weight_shift_left065_contact_constrained_multipoint.json \
  --total-timesteps 40000 \
  --n-envs 4 \
  --max-episode-steps 250 \
  --action-scale 0.10 \
  --upright-penalty-weight 24.0 \
  --action-penalty-weight 0.10 \
  --action-delta-penalty-weight 0.04 \
  --right-contact-penalty-weight 0.25 \
  --clearance-reward-weight 4.0 \
  --clearance-target 0.002 \
  --run-name right_clearance_from_weight_40k \
  --device cpu
```

평가 산출물:

- `outputs/eval/urdf_f_right_clearance_40k/right_clearance_ppo_seed1/evaluation.mp4`
- `outputs/eval/urdf_f_right_clearance_40k/right_clearance_ppo_seed1/evaluation.gif`
- `outputs/eval/urdf_f_right_clearance_40k/right_clearance_ppo_seed1/timeline.csv`
- `outputs/eval/urdf_f_right_clearance_40k/right_clearance_ppo_seed1/summary.json`

결과 요약:

| 항목 | 값 |
|---|---:|
| 완료 step | 155 / 250 |
| 종료 이유 | roll_limit |
| final right clearance | 0.01764 m |
| max right clearance | 0.01810 m |
| final right contacts | 1 |
| mean right contacts | 1.006 |
| mean abs roll | 0.1294 rad |
| max abs roll | 0.5767 rad |
| mean abs pitch | 0.1350 rad |
| max abs pitch | 0.5532 rad |
| combined gate longest | 1.22 s |

판단:

- 오른발 clearance와 contact 감소는 성공했다.
- 하지만 몸을 무너뜨리는 방식으로 clearance를 만들었다.
- 3.1초에서 roll limit로 종료됐다.

즉 이 policy는 “발은 들지만 서 있지 못하는 정책”이다.

## 실험 2: 안전형 clearance PPO

공격형이 낙상했기 때문에 더 보수적인 설정으로 다시 학습했다.

설정:

| 항목 | 값 |
|---|---:|
| total_timesteps | 20,480 |
| action_scale | 0.06 |
| upright_penalty_weight | 80.0 |
| action_penalty_weight | 0.12 |
| action_delta_penalty_weight | 0.06 |
| right_contact_penalty_weight | 0.10 |
| clearance_reward_weight | 2.0 |
| clearance_target | 0.002 m |

학습 명령:

```bash
python3 scripts/train_urdf_f_ppo.py \
  --task right_clearance \
  --pose-json configs/weight_shift_left065_contact_constrained_multipoint.json \
  --total-timesteps 20000 \
  --n-envs 4 \
  --max-episode-steps 250 \
  --action-scale 0.06 \
  --upright-penalty-weight 80.0 \
  --action-penalty-weight 0.12 \
  --action-delta-penalty-weight 0.06 \
  --right-contact-penalty-weight 0.10 \
  --clearance-reward-weight 2.0 \
  --clearance-target 0.002 \
  --run-name right_clearance_safe_20k \
  --device cpu
```

평가 산출물:

- `outputs/eval/urdf_f_right_clearance_safe_20k/right_clearance_ppo_seed1/evaluation.mp4`
- `outputs/eval/urdf_f_right_clearance_safe_20k/right_clearance_ppo_seed1/evaluation.gif`
- `outputs/eval/urdf_f_right_clearance_safe_20k/right_clearance_ppo_seed1/timeline.csv`
- `outputs/eval/urdf_f_right_clearance_safe_20k/right_clearance_ppo_seed1/summary.json`

결과 요약:

| 항목 | 값 |
|---|---:|
| 완료 step | 250 / 250 |
| 종료 이유 | 없음 |
| final right clearance | 0.00111 m |
| max right clearance | 0.01613 m |
| final right contacts | 3 |
| mean right contacts | 2.624 |
| mean abs roll | 0.0162 rad |
| max abs roll | 0.1088 rad |
| mean abs pitch | 0.0665 rad |
| max abs pitch | 0.0769 rad |
| combined gate longest | 0.68 s |

판단:

- 5초 안정성은 유지했다.
- final clearance는 기준선 `0.23 mm`에서 `1.11 mm`로 개선됐다.
- 하지만 목표 `2 mm`에는 못 미친다.
- 오른발 contact도 최종 3개가 남아 있다.

즉 이 policy는 “안정적으로 아주 조금 더 드는 정책”이다.

## 세 결과 비교

| 항목 | zero | 공격형 PPO 40k | 안전형 PPO 20k |
|---|---:|---:|---:|
| 완료 step | 250 | 155 | 250 |
| 종료 이유 | 없음 | roll_limit | 없음 |
| final right clearance | 0.00023 m | 0.01764 m | 0.00111 m |
| max right clearance | 0.01593 m | 0.01810 m | 0.01613 m |
| final right contacts | 3 | 1 | 3 |
| mean right contacts | 2.928 | 1.006 | 2.624 |
| mean abs roll | 0.0220 rad | 0.1294 rad | 0.0162 rad |
| max abs roll | 0.1117 rad | 0.5767 rad | 0.1088 rad |
| combined gate longest | 0.64 s | 1.22 s | 0.68 s |

## 해석

고등학생도 이해할 수 있게 말하면 다음과 같다.

이번에는 두 가지 방법을 시도했다.

첫 번째 policy는 발을 확실히 들었다. 하지만 몸을 기울여 쓰러지면서 발이 뜬 것이다. 이건 걷기에는 쓸 수 없다.

두 번째 policy는 몸을 안정적으로 유지했다. 하지만 발을 충분히 높게 들지는 못했다. 기준선보다 조금 좋아졌지만, 목표인 2mm clearance에는 아직 부족하다.

즉 현재 RL은 다음 두 능력을 각각은 어느 정도 보여줬다.

- 발을 드는 능력
- 안정적으로 버티는 능력

하지만 둘을 동시에 만족하는 정책은 아직 얻지 못했다.

## 현재 결론

성공한 것:

- right_clearance RL 학습/평가 파이프라인 구축
- 공격형 정책에서 2mm 이상 clearance와 contact 감소 확인
- 안전형 정책에서 5초 안정 유지와 clearance 소폭 개선 확인
- 학습 전/후 영상 생성

아직 성공하지 못한 것:

- 5초 안정 유지 + 2mm 이상 clearance + 오른발 contact 감소를 동시에 만족
- swing foot을 안정적으로 들어서 다음 step으로 넘기는 것

## 다음 조치

다음 단계는 `right_clearance` reward를 다시 설계해야 한다.

권장 수정:

1. terminal fall penalty를 크게 추가
2. clearance reward는 roll/pitch 안정 조건을 만족할 때만 지급
3. right contact penalty는 stance 안정 조건과 함께 적용
4. 목표 clearance를 바로 2mm로 두지 말고 `0.5mm -> 1.0mm -> 2.0mm` curriculum으로 올림
5. right foot lift pose를 시작점으로 쓰되, base 안정 조건을 더 강하게 둠

현재 판단:

> right_clearance는 가능성은 보였지만 아직 통과는 아니다. 다음은 reward를 “안정 조건부 clearance”로 바꾸고 curriculum clearance target을 낮은 값부터 다시 올리는 것이 맞다.
