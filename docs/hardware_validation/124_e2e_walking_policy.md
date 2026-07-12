# 124. End-to-End Walking Policy (Gait Clock)

## 목적

doc 123의 커리큘럼 방식은 각 policy 전환 시 distribution shift가 누적되어 연속 보행의 한계가 있다. 이 단계에서는 단일 PPO policy가 gait clock 신호를 받아 좌우 교번 걷기 전체를 학습하는 end-to-end 방식을 구현한다.

## 핵심 설계: Gait Clock

policy가 보행 리듬을 파악할 수 있도록 observation에 주기 신호를 추가한다.

```
phase = 2π × (step % gait_period_steps) / gait_period_steps
obs = [... 45 base obs ..., sin(phase), cos(phase)]   # 총 47차원
```

- `gait_period_steps = 400` (sim step 기준, frame_skip=10 → 실제 2초/cycle)
- `sin(phase) > 0`: right-foot swing phase
- `sin(phase) < 0`: left-foot swing phase

## 환경: GaitWalkingEnv

`envs/gait_walking_env.py` — UrdfFEnv 서브클래스

```python
class GaitWalkingEnv(UrdfFEnv):
    def __init__(self, gait_period_steps=400, **kwargs):
        kwargs.setdefault("task", "walking")
        super().__init__(**kwargs)
        self.gait_period_steps = int(gait_period_steps)
        self._gait_step = 0
        # obs space 확장: base(45) + [sin, cos](2) = 47
        low  = np.concatenate([self.observation_space.low,  [-1., -1.]])
        high = np.concatenate([self.observation_space.high, [ 1.,  1.]])
        self.observation_space = spaces.Box(low=low, high=high, dtype=np.float32)

    def _gait_phase(self):
        return 2.0 * np.pi * (self._gait_step % self.gait_period_steps) / self.gait_period_steps

    def _obs(self):
        base = super()._obs()
        p = self._gait_phase()
        return np.concatenate([base, [np.sin(p), np.cos(p)]]).astype(np.float32)
```

## 보상 함수 (urdf_f_env.py, task="walking")

```python
vel_y       = data.qvel[1]                           # 전진 속도 (+Y 방향)
vel_reward  = clip(vel_y / 0.05, -0.5, 1.0) * 3.0
swing_clr   = right_clr if sin(phase)>0 else left_clr
gait_reward = clip(swing_clr / 0.004, 0.0, 1.5)    # swing 발 clearance
contact_rew = min(stance_contacts, 1.0)              # 지지발 접촉 수
both_air    = left_clr > 0.005 and right_clr > 0.005
penalty     = 2.0 if both_air else 0.0               # 양발 동시 공중 패널티

task_reward = vel_reward + gait_reward - penalty
```

## 학습

`scripts/train_e2e_walking.py` — 체크포인트 기반 재시작 지원

**PPO 하이퍼파라미터**:

| 파라미터 | 값 |
|---|---:|
| n_steps | 2048 |
| batch_size | 256 |
| n_epochs | 10 |
| learning_rate | 3e-4 |
| gamma | 0.99 |
| gae_lambda | 0.95 |
| net_arch | [256, 256] |
| n_envs | 8 |

**체크포인트**: 50,000 steps마다 `outputs/train/urdf_f/{run_name}/checkpoints/` 저장

**학습 명령 (처음 / 이어서 동일)**:
```bash
nohup python3 scripts/train_e2e_walking.py \
  --run-name e2e_walk_v1 --total-timesteps 2000000 --n-envs 8 \
  > /tmp/e2e_walk_v1.log 2>&1 &
```

## 결과 (e2e_walk_v1, ~2M steps)

```bash
python3 scripts/render_e2e_walk.py
```

| 항목 | 값 |
|---|---:|
| 실행 steps | 2000 (낙하 없음) |
| sim time | **15.0 s** |
| 몸체 순전진 | **+13.4 cm** |
| 오른발 전진 | ~+8 cm |
| 왼발 전진 | ~+4 cm |

- 3000 steps까지 낙하 없이 완주
- 비대칭 (오른발이 왼발보다 많이 나감): 추가 학습으로 개선 가능

## 렌더링: 추적 카메라

`scripts/render_e2e_walk.py`

- `azimuth=90`, `elevation=-12`, `distance=1.4` (측면 근거리)
- `cam.lookat[1] = robot_y` (로봇 Y 위치 추적)
- PIL 오버레이: Forward +X.X cm / Time X.X s
- 출력: `outputs/e2e_walk_tracking.mp4`

## 산출물

| 파일 | 설명 |
|---|---|
| `envs/gait_walking_env.py` | GaitWalkingEnv 구현 |
| `scripts/train_e2e_walking.py` | 체크포인트 학습 스크립트 |
| `scripts/render_e2e_walk.py` | 추적 카메라 렌더러 |
| `outputs/train/urdf_f/e2e_walk_v1/ppo_policy.zip` | 학습된 policy |
| `outputs/train/urdf_f/e2e_walk_v1/vecnormalize.pkl` | VecNorm 통계 |
| `outputs/e2e_walk_tracking.mp4` | 보행 영상 |

## 현재 결론

커리큘럼 curriculum 대비:

| | 커리큘럼 (doc 123) | E2E (doc 124) |
|---|---|---|
| policy 수 | 5개 | **1개** |
| distribution shift | 누적 | 없음 |
| 전진 거리 | +3.4cm/3cycle | **+13.4cm/15s** |
| 학습 복잡도 | 높음 | 낮음 |

## 다음 조치

1. **비대칭 개선**: 왼발 전진 보상 강화 또는 추가 학습
2. **학습량 증가**: 2M → 5M+ steps
3. **reward shaping**: gait_reward 계수 조정으로 보폭 증가
