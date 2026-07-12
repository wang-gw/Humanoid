# 06. 구/신 모델 코드 격리 구조

## 목적

신모델은 형상이 달라 보상·튜닝이 크게 갈라진다. 검증된 구모델 코드를 동결하면서
신모델을 자유롭게 수정할 수 있도록 코드를 격리한다.

## 원칙: 제네릭은 공유, 갈라지는 것만 전용

`UrdfFEnv`는 모델 치수를 동적으로 읽는다(`model.nu`, `len(self.joints)`, `obs_dim` 계산).
즉 제어 루프(PD·접촉 감지·gait clock·obs)는 진짜 형상 무관이라 공유해도 된다. 이는
이 저장소가 완전히 다른 로봇 G1도 env를 포크하지 않고 에셋만 분리한 관례와 일치한다.

**갈라지는 것**(보상 커리큘럼·종료조건·튜닝)만 신모델 전용으로 둔다.

```
[공유 — 제네릭 제어, 동결]
envs/urdf_f_env.py        UrdfFEnv          ← 구모델 코드, 이제 안 건드림
envs/gait_walking_env.py  GaitWalkingEnv
        ▲ 상속
[신모델 전용 — 자유 수정]
envs/urdf_f_v2_env.py     GaitWalkingV2Env  ← 모델/포즈/발 기본값 pin + 보상 위임
rewards/urdf_f_v2_walking.py                ← 보행 보상 튜닝터 (v2 커리큘럼)
```

## GaitWalkingV2Env

```python
class GaitWalkingV2Env(GaitWalkingEnv):
    def __init__(self, **kwargs):
        kwargs.setdefault("model_path", V2_MODEL)      # urdf_f_v2 footprint contact
        kwargs.setdefault("pose_path", V2_POSE)        # configs/urdf_f_v2/...
        kwargs.setdefault("right_foot_body", "foot_R_1")
        super().__init__(**kwargs)

    def _reward(self, action):
        if self.task == "walking":
            return compute_walking_reward(self, action)  # rewards/urdf_f_v2_walking.py
        return super()._reward(action)
```

## 검증

- `GaitWalkingV2Env`의 보상 = 동일 상태에서 `GaitWalkingEnv` 보상과 **bit 단위 일치**
  (최대 차이 0.00e+00, 60스텝) — 복제 시작점이 정확함을 확인.
- 학습 스위치: `--env gait_v2`가 모델·포즈·발·보상을 자동 선택.
  - `--env gait`(기본) → 구모델, `foot_R_v1_1`, 접촉 8개 — 완전 동일 동작 확인.

## 실행 명령

```bash
# 신모델 (v2 전용 경로 자동)
python3 scripts/train_e2e_walking.py --env gait_v2 --run-name ... --train-dir outputs/train/urdf_f_v2 ...
# 구모델 (--env 생략 = 기본 gait, 기존과 동일)
python3 scripts/train_e2e_walking.py --run-name ... ...
```

## 산출물

| 파일 | 설명 |
|---|---|
| `envs/urdf_f_v2_env.py` | `GaitWalkingV2Env` (v2 기본값 + 보상 위임) |
| `rewards/urdf_f_v2_walking.py` | v2 보행 보상 (커리큘럼 튜닝 위치) |
| `scripts/train_e2e_walking.py` | `--env gait/gait_v2` 스위치 추가 |
| `scripts/render_urdf_f_v2_walk.py` | v2 전용 추적 카메라 렌더러 |

## 판단

- v2 보상/동작을 아무리 바꿔도 구모델 코드(`UrdfFEnv`/`GaitWalkingEnv`)는 안 건드린다.
- 제네릭 제어 루프 중복 없이 격리 달성 (repo의 G1 관례와 일치).

## 폴더 분리 요약

| 구분 | 구모델 | 신모델 |
|---|---|---|
| 로봇 모델 | `envs/robots/urdf_f_link/` | `envs/robots/urdf_f_v2/` |
| 학습 결과 | `outputs/train/urdf_f/` | `outputs/train/urdf_f_v2/` |
| 설정 | `configs/*.json` | `configs/urdf_f_v2/` |
| env/보상 | `urdf_f_env.py` 등 | `urdf_f_v2_env.py`, `rewards/urdf_f_v2_walking.py` |
| 문서 | `docs/hardware_validation/*.md` | `docs/hardware_validation/urdf_f_v2/` |
