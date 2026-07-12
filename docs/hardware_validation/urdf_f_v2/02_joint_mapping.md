# 02. 관절 이름 매핑 (운동학 기반 확정)

## 목적

CAD export가 관절을 `Revolute N`으로 자동 명명했다. torque plot·action vector·policy
output을 해석하려면 `left_hip_roll` 같은 의미 기반 이름이 필요하다. 구모델의 매핑
(`configs/joint_mapping_provisional.json`)은 번호가 달라 재사용 불가하므로 신모델용
매핑을 새로 확정한다.

## 핵심: body 이름이 의미 기반 → 100% 확정

신모델은 CAD가 body를 의미 기반으로 명명했다(`hipjoint1_L`, `thigh_L`, `calf_L`,
`footJ_L`, `foot_L`). 따라서 운동학 체인(부모→자식)과 관절 축 방향만으로 추측 없이
역할을 확정할 수 있다.

| Revolute | body | 축 | 역할 |
|---|---|---|---|
| 3 | hipjoint1_L | (0 -1 0) roll | **left_hip_roll** |
| 5 | thigh_L | (-1 0 0) pitch | **left_hip_pitch** |
| 7 | calf_L | (-1 0 0) | **left_knee_pitch** |
| 8 | footJ_L | (1 0 0) | **left_ankle_pitch** |
| 11 | foot_L | (0 -1 0) | **left_ankle_roll** |
| 14 | hipjoint1_R | (0 -1 0) | **right_hip_roll** |
| 16 | thigh_R | (1 0 0) | **right_hip_pitch** |
| 18 | calf_R | (1 0 0) | **right_knee_pitch** |
| 20 | footJ_R | (1 0 0) | **right_ankle_pitch** |
| 22 | foot_R | (0 -1 0) | **right_ankle_roll** |

## 축 부호 주의 (구모델 대비)

- roll 관절(hip_roll, ankle_roll) 축: `(0 -1 0)` — 구모델과 **동일**
- pitch 관절(hip_pitch, knee, ankle_pitch) 축: 좌우 X 부호가 **구모델과 반전**
  - 예) 구모델 left_hip_pitch `(1 0 0)` ↔ 신모델 `(-1 0 0)`

→ 이 반전은 나중에 자세 안정화기(nominal stabilizer) 부호에 영향 (`03`~`05` 참조).

## 실행 명령

```bash
python3 scripts/apply_joint_mapping.py \
  --source envs/robots/urdf_f_v2/URDF_v2_mujoco.xml \
  --mapping configs/urdf_f_v2/joint_mapping.json \
  --out envs/robots/urdf_f_v2/URDF_v2_named.xml \
  --doc-dir docs/hardware_validation/urdf_f_v2
```

## 산출물

| 파일 | 설명 |
|---|---|
| `configs/urdf_f_v2/joint_mapping.json` | Revolute N → 의미 이름 매핑 (운동학 확정) |
| `envs/robots/urdf_f_v2/URDF_v2_named.xml` | 관절/액추에이터명 적용 모델 (njnt=11, nu=10) |

## 판단

- 관절 매핑을 추측 없이 확정 (body명이 의미 기반이라 구모델보다 명확).
- pitch 관절 축 반전 확인 → 제어기 부호 검증 필요 항목으로 기록.

## 다음 조치

→ `03_footprint_contact_and_armature_fix.md`: 발 접촉 감지용 패드 추가 및 PD 안정화.
