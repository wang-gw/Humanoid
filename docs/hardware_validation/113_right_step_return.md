# 113. Right Step Return

## 목적

이전 `112` 단계에서 오른발 clearance는 1차 통과로 판단했다.

이번 단계의 목적은 다음이다.

> 들어 올린 오른발을 다시 바닥에 안정적으로 내려놓고, 양발 지지 상태로 복귀할 수 있는지 확인한다.

보행 curriculum에서 이 단계는 `right_clearance` 다음의 `step return / controlled foot placement`에 해당한다.

## 코드 변경

파일:

- `envs/urdf_f_env.py`
- `scripts/train_urdf_f_ppo.py`
- `scripts/evaluate_urdf_f_policy.py`

추가 task:

- `right_return`

보상 구성:

- 오른발 clearance를 낮게 유지
- 오른발 contact 회복
- 오른발 하중 일부 회복
- 왼발 stance contact 유지
- roll/pitch 안정 유지

## 시작 pose 선택

두 후보를 평가했다.

### 후보 1: `right_foot_lift002_ik_stl_footprint_contact.json`

결과:

| 항목 | 값 |
|---|---:|
| 완료 step | 250 / 250 |
| final clearance | 0.00151 m |
| final right contacts | 3 |
| final pitch | -0.1278 rad |

의미:

- 오른발 contact는 일부 회복됐지만 clearance가 아직 1.5mm 수준이다.
- pitch도 안정 gate `0.12 rad`를 약간 넘는다.
- return 학습 시작점으로 적합하다.

### 후보 2: `weight_shift_left065_contact_constrained_multipoint.json`

결과:

| 항목 | 값 |
|---|---:|
| 완료 step | 250 / 250 |
| final clearance | 0.00023 m |
| final right contacts | 3 |
| final pitch | -0.0872 rad |

의미:

- 이미 거의 내려온 상태라 return 학습 의미가 약하다.

따라서 이번 학습은 후보 1을 사용했다.

## 기준선 평가

명령:

```bash
python3 scripts/evaluate_urdf_f_policy.py \
  --task right_return \
  --pose-json configs/right_foot_lift002_ik_stl_footprint_contact.json \
  --policy zero \
  --steps 250 \
  --action-scale 0.05 \
  --upright-penalty-weight 80.0 \
  --action-penalty-weight 0.12 \
  --action-delta-penalty-weight 0.06 \
  --fall-penalty 40.0 \
  --stability-excess-penalty-weight 400.0 \
  --termination-roll-limit 0.18 \
  --termination-pitch-limit 0.18 \
  --out-dir outputs/eval/urdf_f_right_return_lift002_baseline
```

결과 요약:

| 항목 | 값 |
|---|---:|
| 완료 step | 250 / 250 |
| 종료 이유 | 없음 |
| total reward | 726.10 |
| mean clearance | 0.00219 m |
| final clearance | 0.00151 m |
| mean right contacts | 2.624 |
| final right contacts | 3 |
| final right force ratio | 0.319 |
| mean abs roll | 0.0198 rad |
| final roll | -0.0186 rad |
| mean abs pitch | 0.1129 rad |
| final pitch | -0.1278 rad |
| full contact return gate longest | 0.02 s |

판단:

- 5초 안정성은 유지된다.
- 하지만 오른발이 완전히 안정적으로 내려왔다고 보기는 어렵다.
- full contact return gate가 거의 없다.

## PPO 학습

명령:

```bash
python3 scripts/train_urdf_f_ppo.py \
  --task right_return \
  --pose-json configs/right_foot_lift002_ik_stl_footprint_contact.json \
  --total-timesteps 30000 \
  --n-envs 4 \
  --max-episode-steps 250 \
  --action-scale 0.05 \
  --upright-penalty-weight 80.0 \
  --action-penalty-weight 0.12 \
  --action-delta-penalty-weight 0.06 \
  --fall-penalty 40.0 \
  --stability-excess-penalty-weight 400.0 \
  --termination-roll-limit 0.18 \
  --termination-pitch-limit 0.18 \
  --run-name right_return_lift002_30k \
  --device cpu
```

최종 학습 로그:

| 항목 | 값 |
|---|---:|
| total_timesteps | 30,720 |
| ep_len_mean | 250 |
| ep_rew_mean | 736 |

산출물:

- `outputs/train/urdf_f/right_return_lift002_30k/config.json`
- `outputs/train/urdf_f/right_return_lift002_30k/ppo_policy.zip`
- `outputs/train/urdf_f/right_return_lift002_30k/vecnormalize.pkl`

## 학습 policy 평가

산출물:

- `outputs/eval/urdf_f_right_return_lift002_30k/right_return_ppo_seed1/evaluation.mp4`
- `outputs/eval/urdf_f_right_return_lift002_30k/right_return_ppo_seed1/evaluation.gif`
- `outputs/eval/urdf_f_right_return_lift002_30k/right_return_ppo_seed1/timeline.csv`
- `outputs/eval/urdf_f_right_return_lift002_30k/right_return_ppo_seed1/summary.json`

결과 요약:

| 항목 | 값 |
|---|---:|
| 완료 step | 250 / 250 |
| 종료 이유 | 없음 |
| total reward | 824.01 |
| mean clearance | 0.00173 m |
| final clearance | 0.00081 m |
| mean right contacts | 3.448 |
| final right contacts | 4 |
| final right force ratio | 0.275 |
| mean abs roll | 0.0448 rad |
| final roll | 0.0475 rad |
| mean abs pitch | 0.0933 rad |
| final pitch | -0.0973 rad |
| full contact return gate longest | 3.38 s |

## Gate 정의

Return gate:

- right clearance `<= 1.0mm`
- right contacts `>= 3`
- `abs(roll) <= 0.12 rad`
- `abs(pitch) <= 0.12 rad`

Full contact return gate:

- right clearance `<= 1.0mm`
- right contacts `>= 4`
- `abs(roll) <= 0.12 rad`
- `abs(pitch) <= 0.12 rad`

## 기준선 vs PPO 비교

| 항목 | zero | PPO 30k |
|---|---:|---:|
| 완료 step | 250 | 250 |
| 종료 이유 | 없음 | 없음 |
| final clearance | 0.00151 m | 0.00081 m |
| mean clearance | 0.00219 m | 0.00173 m |
| final right contacts | 3 | 4 |
| mean right contacts | 2.624 | 3.448 |
| final pitch | -0.1278 rad | -0.0973 rad |
| return gate fraction | 0.004 | 0.708 |
| return gate longest | 0.02 s | 3.38 s |
| full contact gate fraction | 0.004 | 0.684 |
| full contact gate longest | 0.02 s | 3.38 s |

## 해석

PPO는 오른발을 더 낮게 내려놓고, contact 수를 회복하고, pitch 안정성도 개선했다.

가장 중요한 변화:

- final clearance가 1.51mm에서 0.81mm로 감소
- final right contacts가 3에서 4로 증가
- full contact return gate가 0.02초에서 3.38초로 증가

즉 이번 단계는 성공으로 볼 수 있다.

## 현재 결론

`right_return`은 1차 통과다.

현재 curriculum 상태:

| 단계 | 상태 | best policy |
|---|---|---|
| standing | 통과 | 기준 controller / standing PPO |
| weight shift | 통과 | `weight_shift_left_30k` |
| right unload | 통과 | `right_unload_from_weight_30k` |
| right clearance | 통과 | `right_clearance_rollguard014_12mm_a007_30k` |
| right return | 통과 | `right_return_lift002_30k` |

## 다음 조치

다음 단계는 단일 동작들을 연결하는 것이다.

권장:

1. weight shift
2. right clearance
3. right return
4. standing 복귀

이를 하나의 sequence policy 또는 staged controller로 연결해야 한다.

다음 문서/실험에서는 개별 policy를 이어 붙이는 sequence evaluation을 진행한다.
