# 110. Gated Clearance Push to 1.2mm

## 목적

이전 `109` 단계에서 가장 좋은 정책은 `gated 0.5mm 30k`였다.

그 결과:

- 5초 안정 유지
- final clearance 약 `1.18 mm`
- final right contacts `2`
- 1.0mm gate longest `1.90 s`
- 2.0mm gate는 아직 개선 없음

이번 단계의 목적은 이 결과에서 한 단계 더 밀어보는 것이다.

변경점:

- clearance target: `0.5 mm -> 1.2 mm`
- action_scale: `0.05 -> 0.07`
- right_contact_penalty: `0.10 -> 0.15`

목표:

- 더 큰 clearance
- 더 적은 오른발 contact
- 낙상 없이 유지

## 학습 설정

모델:

- `envs/robots/urdf_f_link/URDF_F_link_virtual_com_gate_com_y005.xml`

pose:

- `configs/weight_shift_left065_contact_constrained_multipoint.json`

학습 명령:

```bash
python3 scripts/train_urdf_f_ppo.py \
  --task right_clearance \
  --pose-json configs/weight_shift_left065_contact_constrained_multipoint.json \
  --total-timesteps 30000 \
  --n-envs 4 \
  --max-episode-steps 250 \
  --action-scale 0.07 \
  --upright-penalty-weight 80.0 \
  --action-penalty-weight 0.12 \
  --action-delta-penalty-weight 0.06 \
  --right-contact-penalty-weight 0.15 \
  --clearance-reward-weight 2.0 \
  --clearance-target 0.0012 \
  --gated-clearance-reward \
  --clearance-gate-roll 0.12 \
  --clearance-gate-pitch 0.12 \
  --fall-penalty 25.0 \
  --run-name right_clearance_gated_12mm_a007_30k \
  --device cpu
```

최종 학습 로그 요약:

| 항목 | 값 |
|---|---:|
| total_timesteps | 30,720 |
| ep_len_mean | 250 |
| ep_rew_mean | 394 |

산출물:

- `outputs/train/urdf_f/right_clearance_gated_12mm_a007_30k/config.json`
- `outputs/train/urdf_f/right_clearance_gated_12mm_a007_30k/ppo_policy.zip`
- `outputs/train/urdf_f/right_clearance_gated_12mm_a007_30k/vecnormalize.pkl`

## 평가

평가 명령:

```bash
python3 scripts/evaluate_urdf_f_policy.py \
  --task right_clearance \
  --pose-json configs/weight_shift_left065_contact_constrained_multipoint.json \
  --policy ppo \
  --policy-path outputs/train/urdf_f/right_clearance_gated_12mm_a007_30k/ppo_policy.zip \
  --vecnormalize-path outputs/train/urdf_f/right_clearance_gated_12mm_a007_30k/vecnormalize.pkl \
  --steps 250 \
  --action-scale 0.07 \
  --upright-penalty-weight 80.0 \
  --action-penalty-weight 0.12 \
  --action-delta-penalty-weight 0.06 \
  --right-contact-penalty-weight 0.15 \
  --clearance-reward-weight 2.0 \
  --clearance-target 0.0012 \
  --gated-clearance-reward \
  --clearance-gate-roll 0.12 \
  --clearance-gate-pitch 0.12 \
  --fall-penalty 25.0 \
  --out-dir outputs/eval/urdf_f_right_clearance_gated_12mm_a007_30k
```

평가 산출물:

- `outputs/eval/urdf_f_right_clearance_gated_12mm_a007_30k/right_clearance_ppo_seed1/evaluation.mp4`
- `outputs/eval/urdf_f_right_clearance_gated_12mm_a007_30k/right_clearance_ppo_seed1/evaluation.gif`
- `outputs/eval/urdf_f_right_clearance_gated_12mm_a007_30k/right_clearance_ppo_seed1/timeline.csv`
- `outputs/eval/urdf_f_right_clearance_gated_12mm_a007_30k/right_clearance_ppo_seed1/summary.json`

## 결과 요약

| 항목 | 값 |
|---|---:|
| 완료 step | 250 / 250 |
| 종료 이유 | 없음 |
| total reward | 398.67 |
| mean clearance | 0.00495 m |
| final clearance | 0.01085 m |
| max clearance | 0.01637 m |
| mean right contacts | 0.992 |
| final right contacts | 1 |
| mean abs roll | 0.0614 rad |
| max abs roll | 0.2318 rad |
| final roll | -0.2318 rad |
| mean abs pitch | 0.0711 rad |
| max abs pitch | 0.1066 rad |
| final pitch | -0.1066 rad |

## Gate 결과

안정 조건:

- `abs(roll) <= 0.12 rad`
- `abs(pitch) <= 0.12 rad`

Gate 정의:

- `gate05`: clearance `>= 0.5mm`, right contacts `<= 2`, 안정 조건 만족
- `gate10`: clearance `>= 1.0mm`, right contacts `<= 2`, 안정 조건 만족
- `gate12`: clearance `>= 1.2mm`, right contacts `<= 2`, 안정 조건 만족
- `gate20`: clearance `>= 2.0mm`, right contacts `<= 1`, 안정 조건 만족

| Gate | fraction | longest |
|---|---:|---:|
| gate05 | 0.828 | 3.44 s |
| gate10 | 0.828 | 3.44 s |
| gate12 | 0.828 | 3.44 s |
| gate20 | 0.804 | 3.36 s |

## 이전 best와 비교

| 항목 | gated 0.5mm 30k | gated 1.2mm/action 0.07 |
|---|---:|---:|
| 완료 step | 250 | 250 |
| 종료 이유 | 없음 | 없음 |
| final clearance | 0.00118 m | 0.01085 m |
| mean clearance | 0.00185 m | 0.00495 m |
| final right contacts | 2 | 1 |
| mean right contacts | 2.172 | 0.992 |
| final roll | 0.0155 rad | -0.2318 rad |
| max abs roll | 0.1117 rad | 0.2318 rad |
| gate20 longest | 0.64 s | 3.36 s |

## 해석

이번 정책은 오른발을 드는 성능만 보면 크게 좋아졌다.

- clearance가 10mm 이상까지 증가했다.
- 오른발 contact도 1개 수준까지 줄었다.
- 2mm gate를 3.36초 동안 만족하는 구간도 생겼다.

하지만 중요한 문제가 있다.

후반부에 roll이 `-0.2318 rad`까지 누적된다. MuJoCo termination limit인 `0.55 rad`에는 도달하지 않아서 episode는 끝까지 갔지만, 우리가 clearance reward를 주기로 한 안정 gate `0.12 rad`는 벗어났다.

즉 이 policy는 다음과 같이 판단해야 한다.

> toe-off/clearance 성능은 크게 개선됐지만, 5초 끝까지 안정 조건을 유지하는 정책은 아니다.

## 현재 결론

이번 실험은 실패가 아니라 중요한 trade-off를 보여준다.

- `gated 0.5mm 30k`: 안정성이 좋고 contact도 줄지만 clearance는 약 1mm 수준
- `gated 1.2mm/action 0.07`: clearance와 contact는 크게 개선되지만 roll drift가 커짐

현재 best를 두 개로 나눠 기록한다.

| 목적 | best policy |
|---|---|
| 안정 우선 | `right_clearance_gated_05mm_30k` |
| clearance/contact 우선 | `right_clearance_gated_12mm_a007_30k` |

## 다음 조치

다음에는 roll drift를 더 직접적으로 막아야 한다.

권장 수정:

1. termination threshold를 reward gate와 맞춰서 `roll_limit = 0.12~0.15 rad`로 낮춘 평가/학습 옵션 추가
2. roll이 `0.12 rad`를 넘으면 큰 penalty를 즉시 부여
3. clearance reward를 episode 후반까지 안정 조건을 유지할 때만 누적
4. 현재 `gated 1.2mm/action 0.07` 설정에서 action_scale은 유지하되 roll drift penalty를 강화

현재 판단:

> 하드웨어/RL 가능성 관점에서는 오른발 toe-off 가능성이 더 강하게 확인됐다. 하지만 보행 curriculum 통과 기준으로는 roll drift 억제가 다음 병목이다.
