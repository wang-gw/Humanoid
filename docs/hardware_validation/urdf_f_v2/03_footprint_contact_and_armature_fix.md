# 03. 발 접촉 패드 + 관절 armature 근본 수정

## 목적

RL env(`UrdfFEnv`)는 발바닥 접촉을 `<foot>_sole_pad_*` 박스 geom으로 감지한다. 변환된
모델의 발 메쉬는 시각 전용(contype=0)이므로 접촉 패드를 추가한다. 이 과정에서 신모델이
서지 못하는 근본 원인도 발견·수정한다.

## 1) 발바닥 접촉 패드

STL 풋프린트를 바디 좌표계에서 자동 측정해 발마다 2×2 = 4개의 박스 패드를 배치한다.
구모델과 발 방향이 다르다(구: 장축 X / 신: 장축 Y).

| 발 | 풋프린트 x | 풋프린트 y | 바닥 z |
|---|---|---|---|
| foot_L_1 / foot_R_1 | [-0.035, +0.035] (폭 0.070) | [-0.067, +0.068] (길이 0.135) | -0.0603 |

env는 오른발 body를 하드코딩(`foot_R_v1_1`)하나 신모델은 `foot_R_1` → env를
인자화(`right_foot_body`)하여 대응(구모델 기본값 유지, `06` 참조).

## 2) ⚠️ 핵심 발견: 관절 armature/damping 누락

신모델이 안 서던 진짜 원인은 자세·부호가 아니었다. **URDF에 관절 damping/armature가
없어 변환 시 누락**됐다.

| | armature | damping |
|---|---:|---:|
| 구모델 (urdf_f) | 0.02 ~ 0.05 | 0.2 ~ 0.4 |
| 신모델 (수정 전) | **0** | **0** |
| 신모델 (수정 후) | 0.02 ~ 0.05 | 0.2 ~ 0.4 |

armature(모터 기어비로 인한 반사 관성)가 0이면 PD 제어가 폭주한다. 진단 실험:

- zero-action 스탠딩: 24~45 스텝 만에 낙하
- 관절을 강체(kp=300)로 만들어도 base_z가 0.04→0.30으로 튀어오르며 24스텝 낙하
  → 제어가 에너지를 주입해 로봇을 튕겨냄 (전형적 무-armature 불안정)

### 수정

구모델과 동일한 값을 hinge 관절에 주입 (빌더가 자동 처리):

| 관절 | armature | damping | frictionloss |
|---|---:|---:|---:|
| hip_roll, hip_pitch, knee_pitch | 0.02 | 0.2 | 0.02 |
| ankle_pitch, ankle_roll | 0.05 | 0.4 | 0.02 |

## 실행 명령

```bash
# 발 패드(STL 자동측정) + 관절 물성 주입을 한 번에
python3 scripts/build_urdf_f_v2_footprint_contact.py
```

## 결과

- 접촉 geom: 발당 4패드 = 8개, env가 정상 인식
- **armature 수정 후: zero-action으로 50초(1000스텝) 안정 스탠딩** — 안정화기 없이도,
  모든 부호 조합에서 성공

## 산출물

| 파일 | 설명 |
|---|---|
| `envs/robots/urdf_f_v2/URDF_F_v2_footprint_contact.xml` | **학습용 최종 모델** (패드+물성) |
| `scripts/build_urdf_f_v2_footprint_contact.py` | 패드 생성 + armature 주입 빌더 |

## 판단

- armature/damping 누락이 신모델 불안정의 **근본 원인**. 수정으로 안정 스탠딩 확보.
- 교훈: SolidWorks/xacro export는 관절 동역학을 안 담으므로 변환 시 반드시 주입해야 함.

## 다음 조치

→ `04_standing_pose_and_stability.md`: 새 형상에 맞는 정적 균형 자세 탐색.
