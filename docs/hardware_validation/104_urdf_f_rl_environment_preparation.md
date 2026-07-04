# 104. URDF_F RL Environment Preparation

## 목적

이번 단계의 목적은 지금까지 검증한 MuJoCo 로봇 모델을 강화학습 코드가 사용할 수 있는 Gymnasium 환경으로 감싸는 것이다.

즉, 아직 “걷는 정책”을 만든 단계가 아니다. 이번 단계는 다음 질문을 확인하는 준비 단계다.

1. 현재 MuJoCo 모델을 RL 환경에서 reset/step 할 수 있는가?
2. 정책 action을 10개 관절 actuator에 안전하게 연결할 수 있는가?
3. 관측값 observation에 자세, 관절, 접촉, clearance 정보를 넣을 수 있는가?
4. standing 기준 controller를 유지한 상태에서 PPO 학습 스크립트가 실제로 실행되는가?

## 시작점

이전 문서 `103_rl_readiness_and_hardware_decision.md`의 결론은 다음과 같았다.

- 하드웨어 형상은 “RL을 해도 절대 못 걷는다”고 판단할 수준은 아니다.
- standing, weight shift, unload, transient toe-off는 가능했다.
- 단순 open-loop lift와 one-leg support는 아직 부족했다.
- 따라서 바로 walking을 학습하기보다 curriculum 방식으로 진행해야 한다.

이번 단계는 그 curriculum의 첫 번째 구현 기반을 만드는 작업이다.

## 추가한 파일

### RL 환경

- `envs/urdf_f_env.py`

역할:

- MuJoCo XML 모델 로드
- pose JSON 로드
- Gymnasium `reset()` / `step()` 제공
- observation/action space 정의
- 접촉력, 발 접촉 수, 오른발 clearance 계산
- roll/pitch/base height 기반 종료 조건 제공
- 기존 검증에서 성공한 nominal stabilizer를 기본 controller로 사용

### 환경 smoke test

- `scripts/test_urdf_f_rl_env.py`

역할:

- 환경 reset/step 확인
- zero/random action rollout 확인
- observation/action shape 확인
- 최종 roll, pitch, contact ratio, clearance, torque command 출력

### PPO 학습 진입점

- `scripts/train_urdf_f_ppo.py`

역할:

- `UrdfFEnv`를 stable-baselines3 PPO에 연결
- VecNormalize 적용
- 학습 config 저장
- policy와 normalization state 저장

### 패키지 export

- `envs/__init__.py`

변경:

- `UrdfFEnv`를 import/export 목록에 추가

## Action 정의

정책 action은 직접 토크가 아니다.

현재 action은 다음 의미다.

```text
action [-1, 1] -> nominal joint target 주변의 작은 목표각 보정
```

계산 흐름:

```text
target_q = nominal_q + action_scale * action
tau = Kp * (target_q - q) - Kd * qd + nominal_stabilizer_torque
tau = clamp(tau, +/- torque_limit)
```

기본값:

| 항목 | 값 |
|---|---:|
| 관절 수/action 수 | 10 |
| action 범위 | `[-1, 1]` |
| action_scale | `0.35 rad` |
| Kp | `20` |
| Kd | `12` |
| torque limit | `30 Nm` |

이 방식을 선택한 이유:

- 처음부터 정책이 직접 토크를 내면 탐색이 너무 거칠다.
- 기존 검증 controller와 연결된다.
- RL은 “완전히 새 제어기”가 아니라 “기준 controller를 보정하는 두뇌”로 시작한다.

## Observation 정의

현재 observation shape는 `44`다.

구성:

| 항목 | 크기 | 의미 |
|---|---:|---|
| base roll/pitch/yaw | 3 | 상체 자세 |
| base angular velocity | 3 | 상체 회전 속도 |
| joint position delta | 10 | 기준 관절각 대비 현재 관절각 |
| joint velocity | 10 | 관절 속도 |
| left/right force ratio | 2 | 좌우 하중 비율 |
| contact/force 요약 | 4 | 좌우 접촉 수와 힘 |
| right clearance/base height delta | 2 | 스윙 발 높이와 base 높이 변화 |
| previous action | 10 | action 변화 억제용 정보 |

## Reward/Task 초안

현재 지원 task:

| task | 목적 |
|---|---|
| `standing` | 양발 접촉과 upright 유지 |
| `weight_shift_left` | 왼발 하중 비율 약 0.65 추종 |
| `right_unload` | 오른발 하중 줄이기 |
| `right_clearance` | 오른발 clearance 만들기 |

현재 reward는 최종형이 아니라 curriculum 시작용 초안이다.

공통 penalty:

- roll/pitch penalty
- joint velocity penalty
- action magnitude penalty
- nominal pose 이탈 penalty

task별 reward:

- standing: 좌우 접촉 유지
- weight shift: left force ratio target 추종
- right unload: right force 감소
- right clearance: right foot clearance 증가 + right force 감소

## 중요한 구현 수정

초기 구현에서는 `frame_skip=10` 동안 같은 torque를 유지했다.

그 결과 기존 standing 성공 controller가 재현되지 않았다.

원인:

- 기존 probe는 매 MuJoCo timestep마다 PD/stabilizer torque를 다시 계산했다.
- 20 ms 동안 torque를 고정하면 빠르게 넘어지는 로봇에서는 controller가 달라진다.

수정:

- `step()` 내부 substep마다 `q`, `qd`, `roll`, `pitch`, COM error를 다시 읽고 torque를 재계산하도록 변경했다.

이 수정 후 기존 standing 기준선이 정상적으로 유지됐다.

## 실행 결과

### 1. 환경 import/reset 확인

명령:

```bash
python3 - <<'PY'
from envs import UrdfFEnv
env = UrdfFEnv()
obs, info = env.reset(seed=1)
print(obs.shape, env.action_space.shape, info["task"])
env.close()
PY
```

결과:

```text
(44,) (10,) standing
```

판단:

- observation/action shape 정상
- 환경 reset 정상

### 2. Standing zero-policy smoke test

명령:

```bash
python3 scripts/test_urdf_f_rl_env.py --task standing --steps 250 --policy zero
```

결과 요약:

| 항목 | 값 |
|---|---:|
| 완료 step | 250 / 250 |
| sim time | 5.0 s |
| terminated_reason | 없음 |
| final roll | 0.0118 rad |
| final pitch | 0.0044 rad |
| left force ratio | 0.642 |
| right force ratio | 0.358 |
| right clearance | 0.00235 m |
| max torque command | 3.18 Nm |

판단:

- RL 환경에서 nominal controller 기준 standing은 유지된다.
- 이 결과는 학습 시작점으로 사용할 수 있다.

### 3. Right-clearance random-policy smoke test

명령:

```bash
python3 scripts/test_urdf_f_rl_env.py \
  --task right_clearance \
  --pose-json configs/right_foot_lift002_ik_stl_footprint_contact.json \
  --steps 100 \
  --policy random
```

결과 요약:

| 항목 | 값 |
|---|---:|
| 완료 step | 100 / 100 |
| sim time | 2.0 s |
| terminated_reason | 없음 |
| final roll | -0.0017 rad |
| final pitch | -0.1245 rad |
| right clearance | 0.00127 m |
| max torque command | 3.20 Nm |

판단:

- 오른발 clearance task도 환경 수준에서는 실행 가능하다.
- 다만 이것은 학습 성공이 아니라 환경 배선 확인이다.

### 4. PPO smoke training

명령:

```bash
python3 scripts/train_urdf_f_ppo.py \
  --total-timesteps 64 \
  --n-envs 1 \
  --max-episode-steps 50 \
  --run-name smoke_standing \
  --device cpu
```

결과:

```text
total_timesteps: 512
ep_len_mean: 50
ep_rew_mean: 94.3
```

산출물:

- `outputs/train/urdf_f/smoke_standing/config.json`
- `outputs/train/urdf_f/smoke_standing/ppo_policy.zip`
- `outputs/train/urdf_f/smoke_standing/vecnormalize.pkl`

판단:

- PPO와 환경 연결은 정상이다.
- 아직 학습 성능 판단 단계는 아니다.

## 현재 결론

이번 단계에서 성공한 것은 다음 한 문장으로 정리할 수 있다.

> 검증된 MuJoCo 로봇 모델을 강화학습이 사용할 수 있는 Gymnasium 환경으로 연결했고, standing 기준 controller 위에서 PPO 학습 스크립트가 실제로 실행되는 것을 확인했다.

즉 이제 “RL을 돌릴 수 있는 형태”는 만들어졌다.

하지만 아직 “RL로 걷는 정책을 얻었다”는 뜻은 아니다.

## 다음 단계

다음 단계는 curriculum 1단계인 standing 학습을 짧게 돌리고 평가하는 것이다.

권장 순서:

1. `standing` task를 10만~50만 step 학습
2. 학습된 policy를 영상으로 렌더링
3. zero-policy 대비 roll/pitch, torque, action 변화를 비교
4. standing이 안정하면 `weight_shift_left` task로 이동
5. weight shift가 안정하면 `right_unload`, 이후 `right_clearance`로 이동

중요:

- 지금은 CAD 대수정 단계가 아니다.
- 먼저 RL 환경과 reward가 실제로 “기존 controller보다 나은 보정”을 학습하는지 확인해야 한다.
- 학습이 계속 한쪽으로 넘어지거나 접촉을 잃으면 reward/termination/action_scale부터 다시 조정한다.
