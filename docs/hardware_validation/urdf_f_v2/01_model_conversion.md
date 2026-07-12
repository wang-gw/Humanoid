# 01. 신모델 변환 (URDF_description → MuJoCo)

## 목적

새로 추가된 `URDF_description/`(SolidWorks 최신 export, xacro 형식)를 RL 학습에 쓸 수
있는 MuJoCo 모델로 변환한다. 구모델도 동일 단계(`prepare_mujoco_urdf.py` →
`build_mujoco_mjcf.py`)로 만들어졌으므로 같은 스크립트를 신모델 경로로 재사용한다.

## 입력

- xacro: `URDF_description/URDF_description/urdf/URDF.xacro`
- 메쉬: `URDF_description/URDF_description/meshes/*.stl` (23개)
- 구조: 회전관절 10개(`Revolute 3,5,7,8,11,14,16,18,20,22`) + 고정관절 12개

## 실행 명령

```bash
# 1) xacro → MuJoCo-loadable URDF (+ STL 23개 복사)
python3 scripts/prepare_mujoco_urdf.py \
  --source URDF_description/URDF_description/urdf/URDF.xacro \
  --source-mesh-dir URDF_description/URDF_description/meshes \
  --out-dir envs/robots/urdf_f_v2

# 2) 액추에이터 포함 MJCF 빌드
python3 scripts/build_mujoco_mjcf.py \
  --urdf envs/robots/urdf_f_v2/URDF_F_mujoco.urdf \
  --out envs/robots/urdf_f_v2/URDF_v2_mujoco.xml --torque-limit 100
```

## 주요 수치 (컴파일 결과)

| 항목 | 값 |
|---|---:|
| nq (자유도 좌표) | 17 (자유베이스 7 + 회전관절 10) |
| nv (자유도 속도) | 16 |
| nu (액추에이터) | 10 |
| nbody | 12 (world + base + 다리 링크 10) |
| 구동 관절 | 좌우 각 5개 (hip_roll/pitch, knee_pitch, ankle_pitch/roll) |

질량 비교 (신 vs 구):

| | base_link | 총 질량 |
|---|---:|---:|
| 신모델 (urdf_f_v2) | 2.88 kg | 8.36 kg |
| 구모델 (urdf_f) | 2.92 kg | 9.21 kg |

## 산출물

| 파일 | 설명 |
|---|---|
| `envs/robots/urdf_f_v2/URDF_F_mujoco.urdf` | MuJoCo-loadable URDF (+ STL 23) |
| `envs/robots/urdf_f_v2/URDF_v2_mujoco.xml` | 액추에이터 포함 기본 MJCF |

## 판단

- 자유 베이스 + 10 구동관절 이족보행 구조가 깨끗하게 컴파일됨 (구모델과 동일한 DOF).
- 관절명이 CAD 자동명(`Revolute N`)이라 다음 단계에서 의미 기반 이름으로 매핑 필요.

## 다음 조치

→ `02_joint_mapping.md`: 운동학 체인으로 각 `Revolute N`의 해부학적 역할 확정.
