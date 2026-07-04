# 109. Gated Clearance Curriculum

## 목적

이전 `108` 단계에서는 right clearance reward를 그냥 주면 policy가 쓰러지면서 발을 드는 방법도 학습한다는 문제가 확인됐다.

이번 단계의 목적은 reward를 다음 방식으로 바꿔서 다시 확인하는 것이다.

> roll/pitch 안정 조건을 만족할 때만 clearance reward를 지급한다.

또한 clearance target을 바로 `2.0 mm`로 두지 않고, curriculum 방식으로 낮은 목표부터 시작했다.

## 코드 변경

파일:

- `envs/urdf_f_env.py`
- `scripts/train_urdf_f_ppo.py`
- `scripts/evaluate_urdf_f_policy.py`

추가 옵션:

| 옵션 | 의미 |
|---|---|
| `gated_clearance_reward` | 안정 조건을 만족할 때만 clearance reward 지급 |
| `clearance_gate_roll` | clearance reward 허용 roll 한계 |
| `clearance_gate_pitch` | clearance reward 허용 pitch 한계 |
| `fall_penalty` | 종료 상태에 대한 추가 penalty |

이번 실험에서 사용한 안정 gate:

| 항목 | 값 |
|---|---:|
| clearance_gate_roll | `0.12 rad` |
| clearance_gate_pitch | `0.12 rad` |
| fall_penalty | `25.0` |

## 기준선

명령:

```bash
python3 scripts/evaluate_urdf_f_policy.py \
  --task right_clearance \
  --pose-json configs/weight_shift_left065_contact_constrained_multipoint.json \
  --policy zero \
  --steps 250 \
  --action-scale 0.05 \
  --upright-penalty-weight 80.0 \
  --action-penalty-weight 0.12 \
  --action-delta-penalty-weight 0.06 \
  --right-contact-penalty-weight 0.10 \
  --clearance-reward-weight 2.0 \
  --clearance-target 0.0005 \
  --gated-clearance-reward \
  --clearance-gate-roll 0.12 \
  --clearance-gate-pitch 0.12 \
  --fall-penalty 25.0 \
  --out-dir outputs/eval/urdf_f_right_clearance_gated_05mm_baseline
```

결과 요약:

| 항목 | 값 |
|---|---:|
| 완료 step | 250 / 250 |
| 종료 이유 | 없음 |
| final clearance | 0.00023 m |
| final right contacts | 3 |
| mean right contacts | 2.928 |
| gate 0.5mm longest | 0.80 s |
| gate 1.0mm longest | 0.70 s |
| gate 2.0mm longest | 0.64 s |

## 실험 1: 0.5mm gated clearance

학습 명령:

```bash
python3 scripts/train_urdf_f_ppo.py \
  --task right_clearance \
  --pose-json configs/weight_shift_left065_contact_constrained_multipoint.json \
  --total-timesteps 30000 \
  --n-envs 4 \
  --max-episode-steps 250 \
  --action-scale 0.05 \
  --upright-penalty-weight 80.0 \
  --action-penalty-weight 0.12 \
  --action-delta-penalty-weight 0.06 \
  --right-contact-penalty-weight 0.10 \
  --clearance-reward-weight 2.0 \
  --clearance-target 0.0005 \
  --gated-clearance-reward \
  --clearance-gate-roll 0.12 \
  --clearance-gate-pitch 0.12 \
  --fall-penalty 25.0 \
  --run-name right_clearance_gated_05mm_30k \
  --device cpu
```

평가 산출물:

- `outputs/eval/urdf_f_right_clearance_gated_05mm_30k/right_clearance_ppo_seed1/evaluation.mp4`
- `outputs/eval/urdf_f_right_clearance_gated_05mm_30k/right_clearance_ppo_seed1/evaluation.gif`
- `outputs/train/urdf_f/right_clearance_gated_05mm_30k/ppo_policy.zip`

결과 요약:

| 항목 | 값 |
|---|---:|
| 완료 step | 250 / 250 |
| 종료 이유 | 없음 |
| total reward | 571.50 |
| final clearance | 0.00118 m |
| mean clearance | 0.00185 m |
| final right contacts | 2 |
| mean right contacts | 2.172 |
| mean abs roll | 0.0249 rad |
| max abs roll | 0.1117 rad |
| gate 0.5mm longest | 1.90 s |
| gate 1.0mm longest | 1.90 s |
| gate 2.0mm longest | 0.64 s |

판단:

- 5초 안정성을 유지했다.
- final clearance가 `0.23 mm -> 1.18 mm`로 개선됐다.
- final right contacts가 `3 -> 2`로 줄었다.
- 0.5mm/1.0mm 안정 gate 지속 시간이 크게 늘었다.

## 실험 2: 1.0mm gated clearance

학습 명령:

```bash
python3 scripts/train_urdf_f_ppo.py \
  --task right_clearance \
  --pose-json configs/weight_shift_left065_contact_constrained_multipoint.json \
  --total-timesteps 20000 \
  --n-envs 4 \
  --max-episode-steps 250 \
  --action-scale 0.05 \
  --upright-penalty-weight 80.0 \
  --action-penalty-weight 0.12 \
  --action-delta-penalty-weight 0.06 \
  --right-contact-penalty-weight 0.10 \
  --clearance-reward-weight 2.0 \
  --clearance-target 0.001 \
  --gated-clearance-reward \
  --clearance-gate-roll 0.12 \
  --clearance-gate-pitch 0.12 \
  --fall-penalty 25.0 \
  --run-name right_clearance_gated_10mm_20k \
  --device cpu
```

평가 산출물:

- `outputs/eval/urdf_f_right_clearance_gated_10mm_20k/right_clearance_ppo_seed1/evaluation.mp4`
- `outputs/eval/urdf_f_right_clearance_gated_10mm_20k/right_clearance_ppo_seed1/evaluation.gif`
- `outputs/train/urdf_f/right_clearance_gated_10mm_20k/ppo_policy.zip`

결과 요약:

| 항목 | 값 |
|---|---:|
| 완료 step | 250 / 250 |
| 종료 이유 | 없음 |
| total reward | 563.57 |
| final clearance | 0.00115 m |
| mean clearance | 0.00186 m |
| final right contacts | 3 |
| mean right contacts | 2.436 |
| mean abs roll | 0.0154 rad |
| max abs roll | 0.1110 rad |
| gate 0.5mm longest | 0.80 s |
| gate 1.0mm longest | 0.72 s |
| gate 2.0mm longest | 0.66 s |

판단:

- 5초 안정성은 유지했다.
- clearance 수치는 0.5mm 정책과 비슷하다.
- 하지만 contact 감소와 gate 지속 시간은 0.5mm 정책보다 나쁘다.

## 비교

| 항목 | zero | gated 0.5mm 30k | gated 1.0mm 20k | 이전 safe 2mm 20k |
|---|---:|---:|---:|---:|
| 완료 step | 250 | 250 | 250 | 250 |
| 종료 이유 | 없음 | 없음 | 없음 | 없음 |
| final clearance | 0.00023 m | 0.00118 m | 0.00115 m | 0.00111 m |
| mean clearance | 0.00113 m | 0.00185 m | 0.00186 m | 0.00184 m |
| final right contacts | 3 | 2 | 3 | 3 |
| mean right contacts | 2.928 | 2.172 | 2.436 | 2.624 |
| mean abs roll | 0.0220 rad | 0.0249 rad | 0.0154 rad | 0.0162 rad |
| gate 0.5mm longest | 0.80 s | 1.90 s | 0.80 s | 0.82 s |
| gate 1.0mm longest | 0.70 s | 1.90 s | 0.72 s | 0.80 s |
| gate 2.0mm longest | 0.64 s | 0.64 s | 0.66 s | 0.68 s |

## 해석

안정 조건부 reward는 의도한 효과가 있었다.

이전 공격형 policy는 발을 크게 들었지만 쓰러졌다. 이번 gated policy는 쓰러지지 않으면서 clearance와 contact를 개선했다.

가장 좋은 결과는 `0.5mm gated clearance`였다.

이 policy는 다음을 동시에 만족했다.

- 5초 유지
- final clearance 약 1.18mm
- 오른발 contact 3개에서 2개로 감소
- 0.5mm/1.0mm 안정 gate 지속 시간 1.9초
- roll/pitch 낙상 없음

하지만 아직 2mm gate는 개선되지 않았다. 즉 toe-off 가능성은 더 좋아졌지만, swing clearance 단계 통과는 아니다.

## 현재 결론

성공한 것:

- 안정 조건부 clearance reward 구현
- 쓰러지는 방식의 clearance 해를 억제
- 안정 유지 상태에서 오른발 contact 감소
- final clearance 1mm 이상 확보

아직 부족한 것:

- 2mm clearance 안정 유지
- 오른발 contact 완전 해제
- 다음 step으로 이어질 swing foot trajectory

## 다음 조치

다음 단계는 `gated 0.5mm 30k` 정책을 기준으로 삼고, 다음 중 하나를 진행한다.

1. 같은 설정에서 더 길게 학습
2. action_scale을 `0.05 -> 0.07`로 소폭 증가
3. right_contact_penalty를 `0.10 -> 0.15`로 증가
4. clearance target을 바로 2mm가 아니라 `1.2mm` 또는 `1.5mm`로 올림

현재 권장:

> `gated 0.5mm 30k`를 현재 best clearance policy로 기록하고, 다음은 target 1.2mm 또는 action_scale 0.07 소폭 증가 실험을 진행한다.
