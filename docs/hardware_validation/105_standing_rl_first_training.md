# 105. Standing RL First Training

## 목적

이번 단계의 목적은 `104`에서 만든 URDF_F RL 환경이 실제 PPO 학습까지 이어지는지 확인하는 것이다.

중요한 점:

- 이번 목표는 “걷기 성공”이 아니다.
- 이번 목표는 “standing task에서 RL 학습 파이프라인이 정상 동작하는지” 확인하는 것이다.
- 기준 controller보다 좋아졌는지도 함께 비교한다.

## 사용한 모델과 환경

모델:

- `envs/robots/urdf_f_link/URDF_F_link_virtual_com_gate_com_y005.xml`

pose:

- `configs/dynamic_standing_pose_actuator_dynamics_contact_5s.json`

환경:

- `envs/urdf_f_env.py`

학습 스크립트:

- `scripts/train_urdf_f_ppo.py`

평가/렌더링 스크립트:

- `scripts/evaluate_urdf_f_policy.py`

## 기준선 평가: zero-policy

zero-policy는 아무 보정 action도 내지 않는 정책이다.

하지만 환경 내부에는 기존 검증에서 성공한 nominal stabilizer가 들어있다. 따라서 zero-policy는 다음 의미다.

> RL 보정 없이 기존 standing controller만 사용한 기준선

명령:

```bash
python3 scripts/evaluate_urdf_f_policy.py \
  --task standing \
  --policy zero \
  --steps 250 \
  --out-dir outputs/eval/urdf_f_standing_baseline
```

산출물:

- `outputs/eval/urdf_f_standing_baseline/standing_zero_seed1/evaluation.mp4`
- `outputs/eval/urdf_f_standing_baseline/standing_zero_seed1/evaluation.gif`
- `outputs/eval/urdf_f_standing_baseline/standing_zero_seed1/timeline.csv`
- `outputs/eval/urdf_f_standing_baseline/standing_zero_seed1/summary.json`

결과 요약:

| 항목 | 값 |
|---|---:|
| 완료 step | 250 / 250 |
| sim time | 5.0 s |
| total reward | 498.58 |
| mean reward | 1.994 |
| mean abs roll | 0.0172 rad |
| max abs roll | 0.0643 rad |
| mean abs pitch | 0.00283 rad |
| max abs pitch | 0.0133 rad |
| mean max torque command | 3.12 Nm |
| max torque command | 4.97 Nm |
| 종료 이유 | 없음 |

판단:

- 기준 controller만으로도 standing은 매우 안정적이다.
- 따라서 standing RL은 “로봇을 세우는 법을 처음 배우는 문제”가 아니라 “이미 안정적인 controller를 해치지 않거나 조금 개선하는 문제”가 된다.

## PPO 학습

명령:

```bash
python3 scripts/train_urdf_f_ppo.py \
  --task standing \
  --total-timesteps 20000 \
  --n-envs 4 \
  --max-episode-steps 250 \
  --run-name standing_20k \
  --device cpu
```

실제 PPO rollout 단위 때문에 최종 학습 step은 `20,480` step이었다.

최종 학습 로그 요약:

| 항목 | 값 |
|---|---:|
| total_timesteps | 20,480 |
| ep_len_mean | 223 |
| ep_rew_mean | 409 |
| fps | 311 |

산출물:

- `outputs/train/urdf_f/standing_20k/config.json`
- `outputs/train/urdf_f/standing_20k/ppo_policy.zip`
- `outputs/train/urdf_f/standing_20k/vecnormalize.pkl`

판단:

- PPO 학습은 정상 실행됐다.
- episode length와 reward가 초반보다 올라갔으므로 policy update 자체는 동작한다.
- 하지만 이것만으로 기준 controller보다 좋아졌다고 판단할 수는 없다.

## 학습 policy 평가

명령:

```bash
python3 scripts/evaluate_urdf_f_policy.py \
  --task standing \
  --policy ppo \
  --policy-path outputs/train/urdf_f/standing_20k/ppo_policy.zip \
  --vecnormalize-path outputs/train/urdf_f/standing_20k/vecnormalize.pkl \
  --steps 250 \
  --out-dir outputs/eval/urdf_f_standing_20k
```

산출물:

- `outputs/eval/urdf_f_standing_20k/standing_ppo_seed1/evaluation.mp4`
- `outputs/eval/urdf_f_standing_20k/standing_ppo_seed1/evaluation.gif`
- `outputs/eval/urdf_f_standing_20k/standing_ppo_seed1/timeline.csv`
- `outputs/eval/urdf_f_standing_20k/standing_ppo_seed1/summary.json`

결과 요약:

| 항목 | 값 |
|---|---:|
| 완료 step | 250 / 250 |
| sim time | 5.0 s |
| total reward | 489.09 |
| mean reward | 1.956 |
| mean abs roll | 0.0567 rad |
| max abs roll | 0.0710 rad |
| mean abs pitch | 0.0355 rad |
| max abs pitch | 0.0501 rad |
| mean max torque command | 3.19 Nm |
| max torque command | 5.57 Nm |
| 종료 이유 | 없음 |

판단:

- 학습된 PPO policy도 5초 standing은 유지했다.
- 하지만 zero-policy 기준선보다 reward가 낮고 roll/pitch 흔들림이 커졌다.
- 즉 20k step standing PPO는 “실행 성공”이지만 “성능 개선 성공”은 아니다.

## Zero vs PPO 비교

| 항목 | zero-policy | PPO 20k | 판단 |
|---|---:|---:|---|
| 5초 유지 | 성공 | 성공 | 둘 다 통과 |
| total reward | 498.58 | 489.09 | zero가 좋음 |
| mean abs roll | 0.0172 | 0.0567 | zero가 좋음 |
| mean abs pitch | 0.00283 | 0.0355 | zero가 좋음 |
| max torque command | 4.97 Nm | 5.57 Nm | PPO가 약간 큼 |

## 왜 이런 결과가 나왔나

고등학생도 이해할 수 있게 말하면 다음과 같다.

지금 standing 문제는 이미 “잘 서는 자동 자세 제어기”가 붙어 있는 상태다.

zero-policy는 그 자동 자세 제어기를 그대로 둔다.

PPO policy는 그 위에 작은 보정을 추가한다.

그런데 20k step은 매우 짧은 학습이다. 그래서 PPO가 “더 잘 서는 보정”을 찾기보다, 오히려 안정적인 자세를 조금 흔드는 보정을 배웠을 가능성이 크다.

즉 실패 의미는 아니다.

이번 결과의 정확한 의미는 다음이다.

> RL 학습 파이프라인은 정상 작동한다. 하지만 standing task는 기준 controller가 이미 강해서, 짧은 PPO 학습으로는 기준선을 이기지 못했다.

## 현재 결론

성공한 것:

- standing RL 환경 구성
- PPO 학습 실행
- 학습 policy 저장
- 학습 전/후 영상 생성
- zero-policy와 PPO policy 수치 비교

아직 성공하지 않은 것:

- PPO가 기준 standing controller보다 더 좋은 보정을 학습하는 것
- weight shift, unload, clearance, step task 학습
- 보행 policy 생성

## 다음 조치

다음 단계는 두 갈래 중 하나다.

### 선택 A: standing reward/action을 개선하고 더 학습

목표:

- PPO가 zero-policy보다 나빠지지 않게 만든다.

수정 후보:

- action penalty 증가
- action change penalty 추가
- zero action 근처에서 시작하도록 policy 초기화/제약
- standing task에서는 action_scale을 `0.35 rad`보다 작게 설정
- roll/pitch penalty 가중치 증가

장점:

- 안정적인 RL 기반을 더 단단하게 만든다.

단점:

- 이미 standing은 잘 되므로, 보행 가능성 판단에는 시간이 더 걸릴 수 있다.

### 선택 B: 바로 weight shift RL로 이동

목표:

- 기준 controller 위에서 좌우 하중 이동을 RL이 조절하게 만든다.

이유:

- standing은 이미 기준 controller가 충분히 좋다.
- 우리에게 더 중요한 것은 발을 들기 전 단계인 하중 이동이다.

권장:

- 다음 단계는 `weight_shift_left` task로 이동한다.
- 단, action_scale을 줄이고 action penalty를 강화한 뒤 시작하는 것이 좋다.

현재 판단:

> standing 파이프라인은 통과했다. 다음 검증 가치는 standing 반복보다 weight shift RL에 있다.
