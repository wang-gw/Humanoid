# 112. Roll-Guard 0.14 Clearance Pass

## 목적

이전 `111` 단계에서 roll-guard를 추가하자 roll drift가 줄었지만, final roll은 여전히 안정 gate 밖이었다.

이전 결과:

- final roll: `-0.1697 rad`
- 안정 gate: `abs(roll) <= 0.12 rad`
- final clearance: `8.55 mm`
- final right contacts: `1`

이번 단계의 목적은 roll 제한을 더 강하게 걸어, clearance/contact 성능을 유지하면서 roll을 안정 gate 안으로 끌어오는 것이다.

## 변경점

이전 roll-guard 대비 변경:

| 항목 | 이전 | 이번 |
|---|---:|---:|
| termination_roll_limit | 0.18 rad | 0.14 rad |
| stability_excess_penalty_weight | 300 | 800 |
| upright_penalty_weight | 100 | 120 |
| fall_penalty | 40 | 60 |
| action_scale | 0.07 | 0.07 |
| clearance_target | 1.2 mm | 1.2 mm |

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
  --upright-penalty-weight 120.0 \
  --action-penalty-weight 0.12 \
  --action-delta-penalty-weight 0.06 \
  --right-contact-penalty-weight 0.15 \
  --clearance-reward-weight 2.0 \
  --clearance-target 0.0012 \
  --gated-clearance-reward \
  --clearance-gate-roll 0.12 \
  --clearance-gate-pitch 0.12 \
  --fall-penalty 60.0 \
  --stability-excess-penalty-weight 800.0 \
  --termination-roll-limit 0.14 \
  --termination-pitch-limit 0.18 \
  --run-name right_clearance_rollguard014_12mm_a007_30k \
  --device cpu
```

최종 학습 로그:

| 항목 | 값 |
|---|---:|
| total_timesteps | 30,720 |
| ep_len_mean | 250 |
| ep_rew_mean | 337 |

산출물:

- `outputs/train/urdf_f/right_clearance_rollguard014_12mm_a007_30k/config.json`
- `outputs/train/urdf_f/right_clearance_rollguard014_12mm_a007_30k/ppo_policy.zip`
- `outputs/train/urdf_f/right_clearance_rollguard014_12mm_a007_30k/vecnormalize.pkl`

## 평가

평가 산출물:

- `outputs/eval/urdf_f_right_clearance_rollguard014_12mm_a007_30k/right_clearance_ppo_seed1/evaluation.mp4`
- `outputs/eval/urdf_f_right_clearance_rollguard014_12mm_a007_30k/right_clearance_ppo_seed1/evaluation.gif`
- `outputs/eval/urdf_f_right_clearance_rollguard014_12mm_a007_30k/right_clearance_ppo_seed1/timeline.csv`
- `outputs/eval/urdf_f_right_clearance_rollguard014_12mm_a007_30k/right_clearance_ppo_seed1/summary.json`

결과 요약:

| 항목 | 값 |
|---|---:|
| 완료 step | 250 / 250 |
| 종료 이유 | 없음 |
| total reward | 560.41 |
| mean clearance | 0.00372 m |
| final clearance | 0.00567 m |
| max clearance | 0.01624 m |
| mean right contacts | 0.996 |
| final right contacts | 1 |
| mean abs roll | 0.0308 rad |
| max abs roll | 0.1099 rad |
| final roll | -0.0820 rad |
| mean abs pitch | 0.0582 rad |
| max abs pitch | 0.0726 rad |
| final pitch | -0.0520 rad |

## Gate 결과

Gate 정의:

- `gate12`: clearance `>= 1.2mm`, right contacts `<= 2`, roll/pitch 안정
- `gate20`: clearance `>= 2.0mm`, right contacts `<= 1`, roll/pitch 안정
- `gate50`: clearance `>= 5.0mm`, right contacts `<= 1`, roll/pitch 안정

안정 조건:

- `abs(roll) <= 0.12 rad`
- `abs(pitch) <= 0.12 rad`

| Gate | fraction | longest |
|---|---:|---:|
| gate12 | 0.988 | 4.24 s |
| gate20 | 0.936 | 4.02 s |
| gate50 | 0.156 | 0.44 s |

## 이전 best와 비교

| 항목 | 안정 우선 0.5mm | rollguard 0.18 | rollguard 0.14 |
|---|---:|---:|---:|
| 완료 step | 250 | 250 | 250 |
| 종료 이유 | 없음 | 없음 | 없음 |
| final clearance | 0.00118 m | 0.00855 m | 0.00567 m |
| mean clearance | 0.00185 m | 0.00452 m | 0.00372 m |
| final right contacts | 2 | 1 | 1 |
| mean right contacts | 2.172 | 0.992 | 0.996 |
| final roll | 0.0155 rad | -0.1697 rad | -0.0820 rad |
| max abs roll | 0.1117 rad | 0.1697 rad | 0.1099 rad |
| gate20 longest | 0.64 s | 3.62 s | 4.02 s |

## 해석

이번 실험은 지금까지의 clearance curriculum 중 가장 균형이 좋다.

이전 rollguard 0.18 정책은 clearance가 더 크지만 final roll이 안정 gate 밖이었다.

이번 rollguard 0.14 정책은 clearance가 조금 줄었지만 다음을 동시에 만족했다.

- 5초 episode 완료
- final roll/pitch 안정 gate 안쪽
- right contacts 1개
- final clearance 5.67mm
- 2mm clearance/contact/stability gate 4.02초 유지

즉 지금까지 목표였던 “발을 들면서 안정적으로 버티기”에 가장 가까운 결과다.

## 현재 결론

현재 best policy를 갱신한다.

| 목적 | best policy |
|---|---|
| right clearance 전체 균형 | `right_clearance_rollguard014_12mm_a007_30k` |
| roll 안정만 최우선 | `right_clearance_gated_05mm_30k` |
| clearance 크기만 최우선 | `right_clearance_rollguard_12mm_a007_30k` |

보행 curriculum 관점에서는 `right_clearance_rollguard014_12mm_a007_30k`를 다음 단계 시작점으로 사용하는 것이 맞다.

## 다음 조치

다음 단계는 step return 또는 controlled foot placement다.

현재는 오른발을 들어 올리는 데 성공했다. 다음에는 들어 올린 오른발을 다시 안정적으로 내려놓아야 한다.

권장 다음 gate:

1. right clearance 유지
2. right foot contact를 다시 부드럽게 회복
3. roll/pitch 안정 유지
4. landing impulse 제한
5. 좌우 contact가 회복된 뒤 standing으로 복귀

현재 판단:

> right clearance curriculum은 1차 통과로 볼 수 있다. 다음 병목은 발을 내리는 step return이다.
