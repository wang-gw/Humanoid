# Periodic Gaits Research Bundle

CHIRO Humanoid v2의 reference-free periodic gait 연구를 다른 협업자가
checkpoint부터 그대로 재개할 수 있도록 만든 재현 번들입니다. 정책은 IK나
시간별 joint reference 없이 10개 절대 관절 position target을 100 Hz로 출력합니다.

## 현재 연구 상태

| 버전 | 용도 | 동일 조건 10회 평가 |
|---|---|---|
| `v1_baseline` | 현재 이어서 연구할 기준 정책 | 10/10 완주, 낙상 0 |
| `v2_experimental` | swing knee 전방 형상 reward 실험 | 7/10 완주, 낙상 3 |

새 연구는 `v1_baseline`에서 시작하는 것을 권장합니다. V2는 결과 비교와 실패
분석을 위해 보존했으며 배포 후보가 아닙니다.

## 폴더 구조

```text
periodic_gaits_research/
├── periodic_gaits/env.py       # Gymnasium/MuJoCo 환경과 reward
├── results/
│   ├── v1_baseline/            # 안정적인 시작 checkpoint와 평가 결과
│   └── v2_experimental/        # 실험 정책과 V1/V2 비교 결과
├── tests/test_env.py
├── train.py                    # V1 처음부터 학습
├── train_v2.py                 # V2 처음부터 학습(권장하지 않음)
├── finetune_v2.py              # 포함된 V1에서 이어 학습
├── evaluate.py                 # 포함된 V1 평가
├── evaluate_v2.py              # 포함된 V1/V2 비교
├── diagnose_v1.py              # reward·관절별 상세 rollout 진단
└── requirements.txt
```

로봇 모델은 저장소 안의
`humanoidv2/models/urdf_f_v2/URDF_F_v2_footprint_contact_symmetric_inertia.xml`
입니다. 절대경로를 저장하지 않으므로 저장소 위치가 달라도 동작합니다.

## 환경 설치

저장소 루트에서 실행합니다. 검증 환경은 Python 3.13입니다.

```bash
python3 -m venv .venv-periodic-gaits
source .venv-periodic-gaits/bin/activate
python -m pip install -r periodic_gaits_research/requirements.txt
python -m pytest -q periodic_gaits_research/tests
```

## 포함된 정책 즉시 재현

```bash
# V1 rollout 및 영상/JSON 생성
MUJOCO_GL=glfw python periodic_gaits_research/evaluate.py

# 동일 seed 10개와 대표 seed에서 V1/V2 비교
MUJOCO_GL=glfw python periodic_gaits_research/evaluate_v2.py
```

새 결과는 Git에서 제외되는 `outputs/periodic_gaits_research/`에 생성됩니다.
원본 checkpoint와 기록된 결과는 `results/`에서 변경하지 않습니다.
다운로드된 artifact는 `cd periodic_gaits_research/results && shasum -a 256 -c SHA256SUMS`로 검증할 수 있습니다.

## 이어서 학습

포함된 안정적인 V1 checkpoint에서 V2 reward로 보수적으로 미세학습:

```bash
python periodic_gaits_research/finetune_v2.py \
  --timesteps 150000 \
  --n-envs 4 \
  --seed 41 \
  --learning-rate 1e-5
```

V1을 처음부터 재학습하려면 기록된 설정을 사용합니다:

```bash
python periodic_gaits_research/train.py \
  --timesteps 1000000 \
  --n-envs 4 \
  --seed 7 \
  --log-interval 25000 \
  --checkpoint-interval 100000
```

학습 폴더에는 최종 정책과 함께 다음 자료가 생성됩니다.

```text
training_diagnostics.csv    # 구간별 reward component/action/성공률
training_summary.json       # 실행 설정과 시간
checkpoints/*.zip           # 기본 100k step 간격 정책
```

학습된 정책의 상세 진단은 다음처럼 실행합니다.

```bash
MUJOCO_GL=glfw python periodic_gaits_research/diagnose_v1.py \
  --model outputs/periodic_gaits_research/v1/ppo_periodic_walk_v1.zip \
  --output outputs/periodic_gaits_research/v1/diagnostics
```

`diagnostics_summary.json`에는 seed별 성공·낙상·전진거리, reward component의
평균/표준편차/최솟값/최댓값, 관절별 action 포화율, position 범위, tracking error,
velocity, unclipped torque 최댓값과 torque clipping 비율이 저장됩니다. 대표 seed의
모든 100 Hz step은 CSV로, rollout은 MP4로 저장됩니다.

대표 rollout의 시간축 torque 그래프는 다음 명령으로 다시 만들 수 있습니다.

```bash
python periodic_gaits_research/plot_v1_torque.py \
  outputs/periodic_gaits_research/v1/diagnostics/rollout_seed_17.csv
```

그래프의 파란 실선은 실제 적용 torque, 주황 점선은 clipping 전 PD torque,
빨간 점선은 관절별 ±torque limit입니다.

PPO 학습은 완전히 bitwise deterministic하다고 보장하지 않습니다. 운영체제,
CPU, PyTorch에 따라 최종 수치가 조금 달라질 수 있으므로 포함된 checkpoint와
평가 JSON을 연구 기준점으로 사용합니다.

## 정책 인터페이스

- simulation/PD: 500 Hz (`dt=0.002`)
- actor policy: 100 Hz
- observation: 실제 로봇에서 구성 가능한 44차원 값
- action: 정규화된 10차원 값을 안전 범위의 절대 joint position으로 변환
- low-level control: fixed-gain PD와 actuator torque clipping

Actor observation 순서는 projected gravity(3), body angular velocity(3), joint
position(10), joint velocity(10), previous action(10), velocity command(3),
left/right gait clock(2), swing/stance ratio(2), gait frequency(1)입니다.

MuJoCo의 정확한 base velocity, contact force, foot velocity, actuator torque와
body height는 reward·종료·평가에만 사용하며 actor observation에는 넣지 않습니다.

## 구현 상세

### 제어 흐름

한 policy step의 흐름은 다음과 같습니다.

```text
실로봇 관측 44개 → PPO actor → action [-1, 1] 10개
→ 절대 joint target으로 변환 → low-pass filter
→ 500 Hz PD torque를 5회 적용 → 다음 100 Hz policy step
```

action은 IK residual이나 torque가 아닙니다. 각 원소 `a_i`는 다음과 같이 절대
관절 목표값으로 바뀝니다.

```text
q_target_i = action_center_i + action_scale_i * clip(a_i, -1, 1)
q_filtered = 0.65 * previous_target + 0.35 * q_target
tau = clip(80 * (q_filtered - q) - 0.32 * qdot, torque_limit)
```

관절 순서와 허용 target 범위는 다음과 같습니다. 각 행은 `[최소, 중심, 최대]`
라디안과 torque limit입니다.

| 관절 | position 범위 [rad] | torque limit [Nm] |
|---|---:|---:|
| left hip roll | `[-0.220, 0.000, 0.220]` | 24 |
| left hip pitch | `[-0.524, -0.222, 0.080]` | 24 |
| left knee pitch | `[-0.007, 0.493, 0.993]` | 24 |
| left ankle pitch | `[-0.187, 0.193, 0.573]` | 24 |
| left ankle roll | `[-0.180, 0.000, 0.180]` | 7 |
| right hip roll | `[-0.220, 0.000, 0.220]` | 24 |
| right hip pitch | `[-0.080, 0.222, 0.524]` | 24 |
| right knee pitch | `[-0.993, -0.493, 0.007]` | 24 |
| right ankle pitch | `[-0.187, 0.193, 0.573]` | 24 |
| right ankle roll | `[-0.180, 0.000, 0.180]` | 7 |

좌우 knee/hip 부호가 다른 것은 MJCF joint axis가 mirror되어 있기 때문입니다.

### gait phase

현재 학습 gait는 0.8 Hz walking이며 swing ratio는 0.4입니다. 좌우 clock은
0.5 cycle 차이를 둡니다. 논문의 von Mises CDF를 그대로 사용하지 않고,
circular logistic window로 부드러운 swing weight `I_swing ∈ [0,1]`를
근사했습니다. stance weight는 `I_stance = 1 - I_swing`입니다.

### gait별 구성과 현재 구현 범위

현재 checkpoint가 실제로 학습한 gait는 **walking 하나뿐**입니다. 아래 표에서
walking은 구현·학습·평가 완료 상태이고, 나머지는 같은 periodic reward 구조로
확장할 때 사용할 설계 방향이지 현재 checkpoint가 수행할 수 있는 기능이 아닙니다.

| Gait | 좌우 phase offset | swing/stance 구성 | 현재 상태 |
|---|---:|---|---|
| Walking | 0.5 cycle | 좌우 교대, swing 0.4 / stance 0.6 | 구현·학습·평가 완료 |
| Running | 보통 0.5 cycle | swing 비율을 높여 양발 flight 구간 생성 | 미구현 |
| Hopping | 0.0 cycle | 양발 동시 swing·동시 착지 | 미구현 |
| Standing | swing 비활성 | 양발 stance, velocity command 0 | 별도 policy 미구현 |
| 비대칭 gait | 좌우 개별 설정 | 좌우 ratio/offset을 독립적으로 조건화 | observation 확장 필요 |

현재 walking 설정은 다음과 같습니다.

```text
frequency = 0.8 Hz
left phase offset = 0.0
right phase offset = 0.5
swing ratio = 0.4
stance ratio = 0.6
forward velocity command = 0.08 m/s
```

정책 observation에는 left/right clock, swing/stance ratio, gait frequency가 이미
포함되어 있어 조건부 multi-gait policy로 확장할 기본 인터페이스는 있습니다.
하지만 현재 학습에서는 이 값들을 episode마다 sampling하지 않고 위 walking
값으로 고정했습니다. 따라서 포함된 V1/V2 checkpoint를 multi-gait policy라고
부르면 안 됩니다.

다음 단계에서 multi-gait를 구현하려면 episode reset 시 gait command를 sampling하고,
gait별 frequency·좌우 offset·swing ratio를 observation과 periodic reward에 동시에
반영해야 합니다. Running은 flight 구간과 충격, hopping은 동시 접촉, standing은
zero-velocity 안정성까지 별도로 검증해야 합니다.

### V1 reward

V1은 다음 cost의 가중합을 1에서 빼는 구조입니다.

```text
periodic_cost = 0.5 * Σ(
    I_swing * bounded_foot_contact_force
  + I_stance * bounded_foot_speed
)

reward = 1
  - 0.40 * periodic_cost
  - 0.30 * (0.65 * velocity_command_cost + 0.35 * upright_cost)
  - 0.10 * (0.45 * action_change_cost
            + 0.35 * torque_cost
            + 0.20 * body_angular_velocity_cost)
```

즉 swing 발은 지면 접촉력을 줄이고, stance 발은 미끄러지는 속도를 줄이도록
학습합니다. 동시에 전진 0.08 m/s command, 상체 직립, 부드러운 action과 작은
torque/body rotation을 유도합니다. 시간별 joint pose나 foot trajectory를
따라가는 reward는 없습니다.

### V2 experimental reward

V2는 V1 reward에 아래 세 항을 추가합니다.

- swing 발 최소 clearance 0.025 m
- swing knee가 반대쪽 stance knee보다 전방 0.03 m에 있도록 하는 task-space cost
- 후진 및 action 경계 포화에 대한 작은 cost

추가 가중치는 natural swing 0.02, backward motion 0.03, saturation 0.001입니다.
무릎 각도 자체를 목표로 하지 않고 base frame의 실제 knee 위치를 비교합니다.
하지만 현재 결과가 7/10 완주이므로 이 설계는 검증 완료가 아니라 다음 연구를
위한 실험 기록입니다.

### reset, 종료 및 성공 판정

- episode 길이: 8초, 최대 800 policy step
- 초기 gait phase: `[0,1)` uniform random
- 초기 joint noise: position 표준편차 0.01 rad, velocity 0.02 rad/s
- 종료: 비정상 수치, 초기 base 높이의 60% 미만, 또는 과도한 자세 기울기
- 낙상 종료 penalty: `-5`
- 성공: 8초 완주, 0.20 m 이상 전진, swing/stance 접촉 패턴 만족

현재는 mass, friction, motor strength, latency, IMU/encoder noise에 대한 domain
randomization을 적용하지 않았습니다. 따라서 포함된 결과는 simulation baseline이며
그 자체로 실제 로봇 배포 준비 완료를 의미하지 않습니다.

### PPO 설정

V1은 actor와 critic 각각 `256-256 ELU` MLP를 사용합니다. 주요 설정은 learning
rate `3e-4`, rollout `512`, batch `256`, epoch `5`, gamma `0.99`, GAE lambda
`0.95`, clip `0.2`, entropy coefficient `0.005`입니다. 기록된 V1 checkpoint는
seed 7, 총 약 1M step 결과입니다. V2 checkpoint는 V1에서 learning rate
`1e-5`로 150k step을 추가 학습한 결과입니다.

## 논문과의 관계

*Sim-to-Real Learning of All Common Bipedal Gaits via Periodic Reward
Composition*의 periodic swing/stance reward 아이디어를 CHIRO 모델에 적용한
연구 구현입니다. 논문의 Cassie 전체 시스템이나 모든 gait를 그대로 복제한
코드는 아닙니다.
