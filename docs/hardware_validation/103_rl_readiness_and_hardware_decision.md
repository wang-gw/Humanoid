# 103. RL Readiness and Hardware Decision

## 목적

이 문서는 지금까지의 하드웨어/RL 가능성 검증 결과를 종합해서 다음 의사결정을 정리한다.

핵심 질문:

1. 현재 하드웨어 형상은 RL을 진행해볼 가치가 있는가?
2. 설계 수정 없이 바로 보행 RL로 넘어가도 되는가?
3. CAD에서 우선 검토해야 할 수정 후보는 무엇인가?
4. RL reward와 curriculum은 어떤 gate를 기준으로 잡아야 하는가?

## 현재 사용 가능한 기준 모델

Baseline contact model:

- `envs/robots/urdf_f_link/URDF_F_link_virtual_stl_footprint_contact.xml`

COM sensitivity candidate:

- `envs/robots/urdf_f_link/URDF_F_link_virtual_com_gate_com_y005.xml`

주요 기준 pose:

- `configs/weight_shift_left065_contact_constrained_multipoint.json`

## Gate 결과 요약

| Gate | 결과 | 주요 수치 | 판단 |
|---|---|---:|---|
| standing | 통과 | 10 s 안정, torque 약 5.16 Nm | 기본 모델 로드/접촉/질량은 사용 가능 |
| weight shift | 통과 | left force ratio 약 0.73 | 하중 이동 가능 |
| right unload | 통과 | right force 약 21.8 N, max roll 0.126 rad | 오른발 하중 감소 가능 |
| transient toe-off | 통과 | clearance 순간 9 mm 이상 | 순간적으로 발은 뜬다 |
| one-leg support baseline | 실패 | 내부 0.360 s, 렌더 0.330 s | 0.5 s 미달 |
| support polygon x1.25 | 개선, 미통과 | 내부 0.468 s, 렌더 0.462 s | 발 접촉 길이에 민감 |
| base COM +Y 5 mm | 조건부 통과 | 내부 0.512 s, 렌더 0.462 s | COM 배치에 매우 민감 |
| open-loop 2 mm lift | 실패 | combined gate 0.198 s | pose 보간만으로 부족 |
| alpha closed-loop lift | 실패 | combined gate 0.198 s | alpha 조절만으로 부족 |
| task-space safe lift | 개선, 미통과 | combined gate 0.396 s | closed-loop 제어 가능성 있음 |

## 핵심 결론

현재 하드웨어 형상은 “RL을 해도 절대 못 걷는다”고 판단할 수준은 아니다.

근거:

- standing 가능
- weight shift 가능
- right unload 가능
- 순간 toe-off 가능
- COM +5 mm 가상 변경에서 one-leg support gate가 조건부 통과
- task-space controller에서 lift gate가 `0.198 s -> 0.396 s`까지 개선

하지만 현재 하드웨어/제어 조합은 “단순 PD trajectory만 주면 안정적으로 걸을 수 있다”고 볼 수도 없다.

근거:

- 2 mm clearance 0.5초 gate는 아직 실패
- open-loop lift는 반복적으로 실패
- 단순 alpha closed-loop도 clearance gate를 개선하지 못함
- task-space controller는 개선됐지만 0.5초에는 미달
- 공격적인 lift는 roll collapse로 이어짐

따라서 현재 판단은 다음과 같다.

> RL 진행은 가능하다. 단, 보행 RL을 바로 시작하기보다 balance/one-leg/swing-clearance curriculum으로 단계화해야 한다. 동시에 CAD에서는 COM 배치와 toe/heel 접촉 길이를 수정 후보로 유지해야 한다.

## 하드웨어 설계 판단

### 1. 모터/토크 관점

지금까지의 probe에서 torque saturation은 대부분 발생하지 않았다.

대표 수치:

- standing max torque: 약 `5.16 Nm`
- unload/lift probe 대부분: 약 `11.8 Nm`
- 공격형 task-space lift collapse: 약 `23.3 Nm`
- torque limit: `30 Nm`

해석:

- 현재 실패의 1차 원인은 모터 토크 부족으로 보이지 않는다.
- 단, 이 수치는 현재 MuJoCo actuator/gear 가정 기준이다.
- 실제 모터 선정에서는 torque-speed curve, 감속기 효율, backlash, thermal limit, peak/continuous torque를 별도로 반영해야 한다.

즉 지금 단계에서 “모터가 부족해서 못 걷는다”는 결론은 아니다.

### 2. COM 배치 관점

가장 중요한 설계 민감도는 `base_link` COM 위치였다.

결과:

- baseline one-leg gate: `0.360 s`
- `base_link` inertial COM `+Y 5 mm`: `0.512 s`
- `+10 mm` 이상은 오히려 나빠짐
- `+40~60 mm`는 roll collapse 발생

해석:

- 상체/base 질량 배치가 보행 가능성에 직접 영향을 준다.
- 필요한 조정량은 크지 않다.
- 너무 큰 COM 이동은 정적 margin이 좋아져도 동역학적으로 불안정할 수 있다.

CAD 후보:

- 배터리, 제어보드, 전장부, 프레임 보강재 배치를 통해 base COM을 약간 조정할 여유 확보
- 큰 이동이 아니라 수 mm 단위의 미세 조정 가능성을 남길 것

### 3. 발바닥/접촉 형상 관점

발바닥 local X 방향, 즉 toe/heel 접촉 길이도 의미 있는 민감도를 보였다.

결과:

- baseline: `0.360 s`
- local X 1.25배 확장: `0.468 s`
- local X 1.30배 이상: 다시 악화
- local Y 폭 확장: COM margin은 개선되지만 gate는 악화

해석:

- 발바닥을 무조건 크게 만들면 해결되는 문제가 아니다.
- toe/heel 접촉 위치와 발목 pitch/roll 축의 상대 위치가 중요하다.
- 폭보다 전후 접촉 길이 쪽이 더 민감하다.

CAD 후보:

- toe/heel 접촉 edge 위치 재검토
- 실제 고무 패드 또는 접촉면이 MuJoCo contact pad와 일치하는지 확인
- 발 전체 확대보다 접촉 패드 배치와 유효 접촉 길이 보강 우선

## RL 진행 가능성

RL은 진행할 가치가 있다. 다만 바로 walking reward로 들어가면 원인 분리가 어렵다.

권장 curriculum:

1. Standing stabilization
   - 목표: base roll/pitch 억제, 양발 접촉 유지
2. Weight shift
   - 목표: left/right force ratio target 추종
3. One-leg support
   - 목표: `right force <= 5 N`, `right contacts = 0`, `abs(roll) <= 0.12 rad`
4. Swing clearance
   - 목표: `right clearance >= 2 mm`, `abs(roll) <= 0.12 rad`
5. Step return
   - 목표: 들어 올린 발을 다시 안정적으로 접촉
6. Alternating stepping
   - 목표: 좌우 다리 교대
7. Forward walking
   - 목표: 속도 추종, 에너지/토크 제한

## RL Reward 초안

초기 reward는 다음 항목을 동시에 봐야 한다.

필수 항목:

- base roll/pitch penalty
- angular velocity penalty
- target force ratio tracking
- swing foot clearance reward
- stance foot contact 유지 reward
- swing foot contact penalty
- torque penalty
- joint velocity penalty
- fall penalty

권장 gate reward:

| 항목 | 목표 |
|---|---|
| standing | `abs(roll), abs(pitch) <= 0.08 rad` |
| one-leg support | `right force <= 5 N`, `right contacts = 0` |
| swing clearance | `right clearance >= 0.002 m` |
| torque | saturation 없음 |
| stability | `abs(roll) <= 0.12 rad` |

## RL 전 모델 준비 필요사항

RL로 넘어가기 전에 다음을 정리해야 한다.

1. 최종 RL baseline model 선택
   - 원본 STL footprint model
   - 또는 COM +5 mm candidate model
2. actuator model 확정
   - torque limit
   - velocity limit
   - gear ratio
   - damping/friction
   - armature
3. observation 정의
   - base orientation
   - base angular velocity
   - joint q/qd
   - foot contact force/contact state
   - foot clearance
   - command phase
4. action 정의
   - joint target offset
   - 또는 joint torque
5. termination 조건
   - roll/pitch collapse
   - base height drop
   - excessive contact impulse
   - joint limit violation

## 지금 당장 CAD에 반영할지 여부

아직 CAD를 바로 크게 수정하는 것은 이르다.

이유:

- COM +5 mm는 가상 inertial 변경이고, 실제 CAD mass distribution과 정확히 같지 않다.
- 발바닥 x1.25는 접촉 패드만 키운 것이며 실제 발 형상/고무/마찰/edge compliance를 완전히 반영하지 않는다.
- task-space controller에서 gate가 크게 개선됐으므로, 제어기 발전 여지가 남아 있다.

하지만 다음 설계 여유는 CAD에 반영할 준비를 해야 한다.

우선순위:

1. 상체 내부 부품 배치로 COM을 수 mm 단위 조정할 수 있게 설계
2. toe/heel 접촉 패드 위치와 유효 접촉 길이 조정 가능하게 설계
3. 발목 roll/pitch 축과 접촉면의 상대 위치 재검토
4. 실제 발바닥 마찰재/패드 두께/압축성을 모델에 반영할 수 있게 치수 확보

## 현재 최종 판단

현재 상태에서의 판단:

- 하드웨어 형상: `조건부 가능`
- 단순 PD/open-loop 보행: `불충분`
- RL 진행 가치: `있음`
- CAD 즉시 대수정: `보류`
- CAD 미세 조정 후보: `COM +Y 수 mm`, `toe/heel 접촉 길이`
- 다음 기술 단계: `RL/MPC용 closed-loop stepping 환경 구성`

## 다음 작업

다음 단계는 RL 실험 준비다.

권장 순서:

1. RL용 MuJoCo env wrapper 생성
2. observation/action/reward/termination 정의
3. standing task부터 학습
4. weight shift task 학습
5. one-leg support task 학습
6. swing clearance task 학습
7. 여기서 gate가 계속 실패하면 CAD 수정 후보를 실제 설계에 반영
