# 111. Roll-Guarded Clearance

## 목적

이전 `110` 단계에서 `gated 1.2mm/action 0.07` 정책은 오른발 clearance/contact 성능을 크게 개선했지만, 후반 roll drift가 커졌다.

문제:

- final clearance: `10.85 mm`
- final right contacts: `1`
- 하지만 final roll: `-0.2318 rad`
- 안정 gate `abs(roll) <= 0.12 rad`를 초과

이번 단계의 목적은 같은 clearance 성능을 최대한 유지하면서 roll drift를 줄이는 것이다.

## 코드 변경

파일:

- `envs/urdf_f_env.py`
- `scripts/train_urdf_f_ppo.py`
- `scripts/evaluate_urdf_f_policy.py`

추가 옵션:

| 옵션 | 의미 |
|---|---|
| `stability_excess_penalty_weight` | roll/pitch가 gate를 초과한 양에 대한 추가 penalty |
| `termination_roll_limit` | 학습/평가 roll 종료 한계 |
| `termination_pitch_limit` | 학습/평가 pitch 종료 한계 |
| `termination_base_drop` | base height drop 종료 한계 |

이번 실험 설정:

| 항목 | 값 |
|---|---:|
| action_scale | 0.07 |
| clearance_target | 0.0012 m |
| right_contact_penalty_weight | 0.15 |
| upright_penalty_weight | 100.0 |
| stability_excess_penalty_weight | 300.0 |
| clearance_gate_roll | 0.12 rad |
| clearance_gate_pitch | 0.12 rad |
| termination_roll_limit | 0.18 rad |
| termination_pitch_limit | 0.18 rad |
| fall_penalty | 40.0 |

## 기준선 확인

같은 roll-guard 설정에서 zero-policy를 평가했다.

결과:

| 항목 | 값 |
|---|---:|
| 완료 step | 250 / 250 |
| 종료 이유 | 없음 |
| final clearance | 0.00023 m |
| final right contacts | 3 |
| final roll | 0.0003 rad |
| final pitch | -0.0872 rad |

기준선은 안정적이지만 clearance는 부족하다.

## 학습

명령:

```bash
python3 scripts/train_urdf_f_ppo.py \
  --task right_clearance \
  --pose-json configs/weight_shift_left065_contact_constrained_multipoint.json \
  --total-timesteps 30000 \
  --n-envs 4 \
  --max-episode-steps 250 \
  --action-scale 0.07 \
  --upright-penalty-weight 100.0 \
  --action-penalty-weight 0.12 \
  --action-delta-penalty-weight 0.06 \
  --right-contact-penalty-weight 0.15 \
  --clearance-reward-weight 2.0 \
  --clearance-target 0.0012 \
  --gated-clearance-reward \
  --clearance-gate-roll 0.12 \
  --clearance-gate-pitch 0.12 \
  --fall-penalty 40.0 \
  --stability-excess-penalty-weight 300.0 \
  --termination-roll-limit 0.18 \
  --termination-pitch-limit 0.18 \
  --run-name right_clearance_rollguard_12mm_a007_30k \
  --device cpu
```

최종 학습 로그:

| 항목 | 값 |
|---|---:|
| total_timesteps | 30,720 |
| ep_len_mean | 250 |
| ep_rew_mean | 366 |

산출물:

- `outputs/train/urdf_f/right_clearance_rollguard_12mm_a007_30k/config.json`
- `outputs/train/urdf_f/right_clearance_rollguard_12mm_a007_30k/ppo_policy.zip`
- `outputs/train/urdf_f/right_clearance_rollguard_12mm_a007_30k/vecnormalize.pkl`

## 평가

산출물:

- `outputs/eval/urdf_f_right_clearance_rollguard_12mm_a007_30k/right_clearance_ppo_seed1/evaluation.mp4`
- `outputs/eval/urdf_f_right_clearance_rollguard_12mm_a007_30k/right_clearance_ppo_seed1/evaluation.gif`
- `outputs/eval/urdf_f_right_clearance_rollguard_12mm_a007_30k/right_clearance_ppo_seed1/timeline.csv`
- `outputs/eval/urdf_f_right_clearance_rollguard_12mm_a007_30k/right_clearance_ppo_seed1/summary.json`

결과 요약:

| 항목 | 값 |
|---|---:|
| 완료 step | 250 / 250 |
| 종료 이유 | 없음 |
| total reward | 442.98 |
| mean clearance | 0.00452 m |
| final clearance | 0.00855 m |
| max clearance | 0.01633 m |
| mean right contacts | 0.992 |
| final right contacts | 1 |
| mean abs roll | 0.0503 rad |
| max abs roll | 0.1697 rad |
| final roll | -0.1697 rad |
| mean abs pitch | 0.0640 rad |
| max abs pitch | 0.0763 rad |
| final pitch | -0.0762 rad |

## 이전 결과와 비교

| 항목 | 안정 우선 0.5mm | 1.2mm/action 0.07 | roll-guard 1.2mm/action 0.07 |
|---|---:|---:|---:|
| 완료 step | 250 | 250 | 250 |
| 종료 이유 | 없음 | 없음 | 없음 |
| final clearance | 0.00118 m | 0.01085 m | 0.00855 m |
| mean clearance | 0.00185 m | 0.00495 m | 0.00452 m |
| final right contacts | 2 | 1 | 1 |
| mean right contacts | 2.172 | 0.992 | 0.992 |
| final roll | 0.0155 rad | -0.2318 rad | -0.1697 rad |
| max abs roll | 0.1117 rad | 0.2318 rad | 0.1697 rad |
| gate12 longest | 1.90 s | 3.44 s | 3.74 s |
| gate20 longest | 0.64 s | 3.36 s | 3.62 s |
| gate50 longest | 0.34 s | 0.90 s | 0.92 s |

## 해석

roll-guard는 의도한 방향으로 작동했다.

좋아진 점:

- final roll이 `-0.2318 -> -0.1697 rad`로 줄었다.
- 2mm 안정 gate longest가 `3.36 -> 3.62 s`로 늘었다.
- 오른발 contact는 1개 수준을 유지했다.
- clearance도 여전히 8.55mm로 크다.

아직 부족한 점:

- final roll `-0.1697 rad`는 여전히 안정 gate `0.12 rad` 밖이다.
- 5초 끝까지 안정 gate 안에 머무르지는 못했다.
- 즉 roll drift는 줄었지만 해결되지는 않았다.

## 현재 결론

현재 best를 목적별로 다시 정리한다.

| 목적 | best policy | 판단 |
|---|---|---|
| 안정 우선 | `right_clearance_gated_05mm_30k` | roll 안정, clearance 약 1.18mm |
| clearance/contact 우선 | `right_clearance_rollguard_12mm_a007_30k` | contact 1개, clearance 8.55mm, roll drift 감소 |

이번 결과는 하드웨어 가능성 관점에서는 긍정적이다.

오른발 toe-off와 2mm 이상 clearance는 여러 정책에서 반복적으로 확인됐다. 다만 현재 제어/RL reward에서는 발을 든 뒤 roll을 끝까지 잡는 것이 병목이다.

## 다음 조치

다음 실험은 roll drift를 더 강하게 제한해야 한다.

권장:

1. `termination_roll_limit`을 `0.18 -> 0.14`로 낮춤
2. `stability_excess_penalty_weight`를 `300 -> 800`으로 증가
3. clearance target은 `1.2mm` 유지
4. action_scale은 `0.07` 유지 또는 `0.06`으로 소폭 낮춤

목표:

- final roll `<= 0.12~0.14 rad`
- right contacts `<= 1`
- clearance `>= 2mm`
- gate20 longest `>= 2s`
