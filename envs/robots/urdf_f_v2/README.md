# urdf_f_v2 — 최신 모델 학습 파이프라인

`URDF_description/`(SolidWorks 최신 export)를 MuJoCo 학습용 모델로 변환한 결과다.
구모델(`urdf_f` / `urdf_f_link`)과 **완전히 분리된 네임스페이스**로, 기존 학습에 영향을 주지 않는다.

## 산출물

| 파일 | 설명 |
|---|---|
| `URDF_F_mujoco.urdf` | xacro → MuJoCo-loadable URDF (+ STL 23개) |
| `URDF_v2_mujoco.xml` | 액추에이터 포함 기본 MJCF (관절명 `Revolute N`) |
| `URDF_v2_named.xml` | 사람이 읽을 수 있는 관절명 적용 (`left_hip_roll` 등) |
| `URDF_F_v2_footprint_contact.xml` | **학습용 최종 모델** — 발바닥 접촉 패드 8개 추가 |

- 구조: 자유 베이스 + 구동관절 10개 (nq=17, nv=16, nu=10)
- 오른발 바디명은 `foot_R_1` (구모델은 `foot_R_v1_1`) → env에 `right_foot_body="foot_R_1"` 전달 필요
- base_link 원점이 바닥 근처에 있어 서있는 base_z가 작음(~0.02)

## 처음부터 재생성하는 법

```bash
# 1) xacro → MuJoCo URDF (+ STL 복사)
python3 scripts/prepare_mujoco_urdf.py \
  --source URDF_description/URDF_description/urdf/URDF.xacro \
  --source-mesh-dir URDF_description/URDF_description/meshes \
  --out-dir envs/robots/urdf_f_v2

# 2) 액추에이터 포함 MJCF 빌드
python3 scripts/build_mujoco_mjcf.py \
  --urdf envs/robots/urdf_f_v2/URDF_F_mujoco.urdf \
  --out envs/robots/urdf_f_v2/URDF_v2_mujoco.xml --torque-limit 100

# 3) 관절 이름 매핑 적용
python3 scripts/apply_joint_mapping.py \
  --source envs/robots/urdf_f_v2/URDF_v2_mujoco.xml \
  --mapping configs/urdf_f_v2/joint_mapping.json \
  --out envs/robots/urdf_f_v2/URDF_v2_named.xml \
  --doc-dir docs/hardware_validation/urdf_f_v2

# 4) 발바닥 접촉 패드 + 관절 물성(armature/damping) 추가
python3 scripts/build_urdf_f_v2_footprint_contact.py
```

관절 매핑(`configs/urdf_f_v2/joint_mapping.json`)은 운동학 체인 + 축 방향으로 확정했다
(바디명이 `hipjoint1_L`, `thigh_L`, `calf_L`, `footJ_L`, `foot_L`로 의미 기반).

### ⚠️ 핵심 수정: 관절 armature/damping
URDF에는 관절 damping/armature가 없어서 변환 시 누락됐다. 이 값이 0이면
PD 제어가 폭주해(로봇이 튀어오름) 서지 못한다. 4단계 빌더가 구모델과 동일한 값을
자동 주입한다: **hip/knee = armature 0.02 / damping 0.2, ankle = armature 0.05 / damping 0.4**
(frictionloss 0.02). 이 수정 후 로봇은 zero-action으로 50초 안정 스탠딩한다.

## 서있는 자세 (완료)

`configs/urdf_f_v2/quasistatic_standing_pose.json` — 새 발바닥 패드에 맞춘 준정적 균형 포즈.
CEM 탐색으로 CoM을 지지면 중심에 맞췄고, PD 제어하에 안정 스탠딩 검증됨.

```bash
# (재생성 시) v2 전용 포즈 탐색
python3 scripts/search_quasistatic_standing_pose_v2.py \
  --left-foot foot_L_1 --right-foot foot_R_1 \
  --out configs/urdf_f_v2/quasistatic_standing_pose.json
```

## 코드 구조 (구/신 모델 격리)

신모델은 **전용 코드**를 가지며, 구모델(urdf_f)의 검증된 코드는 **동결**된다.
제네릭한 제어 루프(PD·접촉·gait clock·obs)는 모델 치수를 동적으로 읽으므로 공유한다.

| 갈라지는 부분 | 위치 |
|---|---|
| 모델/포즈/발/안정화기 기본값 | `envs/urdf_f_v2_env.py` → `GaitWalkingV2Env` |
| **보행 보상(커리큘럼 튜닝터)** | `rewards/urdf_f_v2_walking.py` |
| 종료 조건·obs 수정 | `GaitWalkingV2Env`에서 메서드 오버라이드 |

→ v2 보상/동작을 아무리 바꿔도 `UrdfFEnv`/`GaitWalkingEnv`(구모델 코드)는 안 건드린다.
검증: `GaitWalkingV2Env` 보상은 동일 상태에서 base와 bit 단위로 일치(복제 시작점).

## 학습 실행

`--env gait_v2`가 v2 모델·포즈·발·보상을 자동으로 잡는다:

```bash
python3 scripts/train_e2e_walking.py \
  --env gait_v2 \
  --run-name e2e_walk_v2model_v1 \
  --train-dir outputs/train/urdf_f_v2 \
  --swing-step-weight 0.5 --swing-contact-weight 0.5 \
  --total-timesteps 2000000 --n-envs 8
```

- 구모델 학습은 `--env` 생략(기본 `gait`) → 기존 그대로 동작.
- 보행 보상을 바꾸려면 `rewards/urdf_f_v2_walking.py`의 표시된 블록만 수정.
- 결과 확인: `render_e2e_walk.py` 패턴으로 롤아웃(raw env + 수동 VecNormalize obs 정규화).

## 남은 작업 (RL 연구 단계)

기계적 셋업(변환·접촉·물성·포즈·스탠딩)은 모두 완료. 이제부터는 구모델이 v1→v5로
거쳤던 것과 같은 **보행 정책 학습 반복**이다:
1. 보상 가중치 튜닝(swing_step / swing_contact 등)으로 대칭 걸음 유도
2. 필요시 커리큘럼(weight_shift → clearance → walking)
3. 관절 부호 방향 정밀 검증 — `audit_joint_sign_response.py`
