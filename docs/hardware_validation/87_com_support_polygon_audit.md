# 87. COM Support Polygon 감사

## 목적

이 단계의 목적은 오른발 하중 제거 실패 원인을 정적 안정성 관점에서 분리하는 것이다.

이전 단계에서 `left force ratio ~= 0.65`까지는 안정적으로 도달했지만, `0.70+`로 이동하거나 오른발을 더 가볍게 만드는 시도는 roll 붕괴로 실패했다. 따라서 이번에는 현재 자세에서 전체 COM 투영점이 발바닥 지지 다각형 안에 있는지 확인했다.

## 사용 모델과 자세

- 기준 모델:
  - `envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_toeheel_z_grounded_step_visual_actuator_dynamics_multipoint_fullheight_contact.xml`
- 감사 스크립트:
  - `scripts/audit_com_support_polygon.py`
- 비교 자세:
  - 중립 standing:
    - `configs/dynamic_standing_pose_actuator_dynamics_contact_5s.json`
  - 안정 `0.65` weight shift:
    - `configs/weight_shift_left065_contact_constrained_multipoint.json`
  - 실패한 `0.70` 오른발 unload 후보:
    - `configs/right_unload_left070_contact_constrained_multipoint.json`

## 방법

현재 모델은 발바닥을 좌우 각각 4개 pad로 나눈 multipoint contact 모델이다. 따라서 기존 단일 sole collision 기준이 아니라, 각 foot pad box의 world corner를 모아 다음 support polygon을 계산했다.

- 왼발 support polygon
- 오른발 support polygon
- 양발 전체 support polygon

각 polygon에 대해 COM 투영점의 위치를 계산했다.

- `inside = true`:
  - COM 투영점이 polygon 내부에 있음
- `signed_margin > 0`:
  - 내부에 있고 가장 가까운 edge까지의 거리
- `signed_margin < 0`:
  - polygon 밖에 있으며 가장 가까운 edge까지의 거리

## 산출물

### 중립 standing

- JSON:
  - `outputs/analysis/com_support_polygon_neutral_standing_multipoint/com_support_polygon_audit.json`
- plot:
  - `outputs/analysis/com_support_polygon_neutral_standing_multipoint/com_support_polygon.png`

### 안정 `0.65` weight shift

- JSON:
  - `outputs/analysis/com_support_polygon_left065_multipoint/com_support_polygon_audit.json`
- plot:
  - `outputs/analysis/com_support_polygon_left065_multipoint/com_support_polygon.png`

### 실패한 `0.70` unload 후보

- JSON:
  - `outputs/analysis/com_support_polygon_right_unload_left070_multipoint/com_support_polygon_audit.json`
- plot:
  - `outputs/analysis/com_support_polygon_right_unload_left070_multipoint/com_support_polygon.png`

## 결과 1: 중립 Standing

| 항목 | 값 |
|---|---:|
| COM X | 0.0609 m |
| COM Y | -0.0480 m |
| 양발 polygon 내부 여부 | 내부 |
| 양발 polygon margin | +9.5 mm |
| 왼발 단독 polygon margin | -25.0 mm |
| 오른발 단독 polygon margin | -20.5 mm |

### 해석

중립 standing은 양발 전체 지지면 안에 COM이 들어와 있다. 다만 margin이 약 `9.5 mm`로 크지 않다.

또한 중립 자세에서는 COM이 어느 한 발 단독 지지면 안에는 들어오지 않는다. 이 자체는 정상이다. 한 발 지지를 하려면 pelvis/torso 또는 leg pose를 통해 COM을 지지발 쪽으로 더 이동시켜야 한다.

## 결과 2: 안정 `0.65` Weight Shift

| 항목 | 값 |
|---|---:|
| COM X | 0.0519 m |
| COM Y | -0.0434 m |
| 왼발 polygon 내부 여부 | 외부 |
| 왼발 polygon margin | -30.0 mm |
| 오른발 polygon margin | -66.6 mm |
| 양발 polygon margin | -7.1 mm |

### 해석

`0.65` 자세는 동역학 시뮬레이션에서는 안정적으로 유지되지만, 정적 support polygon 기준으로 보면 COM 투영점이 왼발 단독 지지면 안에 들어오지 않는다.

특히 왼발 단독 polygon 기준으로 약 `30 mm` 바깥에 있다. 따라서 현재 `0.65` 자세에서 오른발을 들어 올리면, 정적 안정 조건을 만족하지 못할 가능성이 높다.

이 결과는 이전 단계의 실패와 잘 맞는다.

- `0.65`까지는 양발 접촉과 controller가 버텨준다.
- 하지만 오른발 하중을 더 줄이면 오른발이 제공하던 안정 여유가 사라진다.
- 그 순간 COM이 왼발 support polygon 안에 충분히 들어오지 못해 roll 붕괴가 발생한다.

## 결과 3: 실패한 `0.70` 오른발 Unload 후보

| 항목 | 값 |
|---|---:|
| COM X | 0.0524 m |
| COM Y | -0.0418 m |
| 왼발 polygon 내부 여부 | 외부 |
| 왼발 polygon margin | -49.8 mm |
| 오른발 polygon margin | -147.2 mm |
| 양발 polygon margin | -40.0 mm |

### 해석

실패한 `0.70` 후보는 왼발 단독 polygon 기준으로 COM이 약 `50 mm` 바깥에 있다. 이는 안정 `0.65`보다 더 나쁘다.

즉 `0.70` 탐색 결과가 force ratio 숫자만 보면 목표에 가까워 보였더라도, 실제 자세는 support polygon 관점에서 더 불리해진 상태였다. 따라서 이 후보를 swing 준비 자세로 사용하면 안 된다.

## 핵심 결론

오른발 하중 제거 실패의 1차 원인은 모터 토크 부족이 아니라 정적/준정적 지지 조건 부족으로 보는 것이 타당하다.

근거:

- 이전 단계에서 torque saturation은 `0.0`이었다.
- 최대 토크는 torque limit `30 Nm`보다 낮았다.
- 반면 COM 투영점은 왼발 단독 support polygon 바깥에 있다.
- 안정 `0.65` 자세도 왼발 단독 기준 약 `30 mm` 바깥이다.
- 실패 `0.70` 후보는 약 `50 mm` 바깥으로 더 악화된다.

따라서 현재 모델에서 바로 오른발 lift를 시도하면 RL이 아니라도 물리적으로 어려운 조건에서 시작하게 된다.

## 설계 관점 판단

현재 로봇이 보행 가능성이 없다고 단정할 단계는 아니다. 하지만 하드웨어/RL 검증 gate 기준으로는 다음 판단이 가능하다.

1. 양발 standing과 0.65 수준의 weight shift는 가능하다.
2. 한 발 지지로 넘어가기 위한 COM 여유가 부족하다.
3. RL이 이 문제를 일부 보상할 가능성은 있지만, 하드웨어 검증에서는 먼저 support margin을 개선할 수 있는지 확인해야 한다.
4. 다음 가상 설계 변경으로 원인을 분리해야 한다.

## 다음 조치

다음 단계에서는 실제 CAD를 수정하기 전에 MuJoCo 모델에서 가상 변형을 만들어 비교한다.

1. 발바닥 support polygon 확대 변형
   - foot pad 폭/길이를 키워서 같은 자세에서 COM이 왼발 polygon 안으로 들어오는지 확인한다.
   - 성공하면 발 치수 또는 contact pad 위치가 주요 병목일 가능성이 크다.

2. 상체/base COM 좌측 이동 변형
   - base/torso inertial 위치를 가상으로 이동해서 `0.70+` unload가 가능한지 확인한다.
   - 성공하면 상체 질량 배치 또는 hip spacing/leg pose가 주요 병목일 가능성이 크다.

3. 위 두 변형을 각각 같은 standing/weight shift/unload gate로 검증한다.

현재 기준으로 swing foot lift 진행 조건은 아직 만족하지 못했다.

