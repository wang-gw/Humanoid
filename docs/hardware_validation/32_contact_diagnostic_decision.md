# Contact 진단 후 현재 판단

## 수행한 일

이번 단계에서는 새 STL 모델의 발바닥 접촉 문제가 실제 standing 실패와 얼마나 관련이 있는지 분리했다.

1. `URDF_F_link_named.xml`의 좌우 body/geom world 위치를 감사했다.
2. 좌우 foot body는 떨어져 있지만, sole collision이 같은 위치로 겹치는 것을 확인했다.
3. 최종 설계값이 아닌 진단용 contact 모델 `URDF_F_link_body_contact.xml`을 만들었다.
4. 진단 모델에서 COM/support 분석과 PD standing probe를 실행했다.

## 핵심 수치

- 좌우 foot body 거리: `0.154000 m`
- 좌우 sole collision 거리: `0.000000 m`
- body-centered contact 적용 후 support AABB: `x [-0.122, 0.102]`, `y [-0.0815, 0.0385]`
- body-centered contact 적용 후 COM inside support: `x=True`, `y=True`
- base z 보정 PD probe 최종 roll: `-2.088366 rad`
- base z 보정 PD probe 최종 pitch: `-1.093511 rad`
- base z 보정 PD probe 최대 qvel norm: `959.736245`
- base z 보정 PD probe 최대 contact force: `3930.414182`

## 판단

좌우 발바닥 contact가 겹치는 문제는 명확한 모델 오류 후보다. 이 상태로 RL을 돌리면 정책이 실제 로봇과 다른 접촉 조건을 학습하게 된다.

하지만 foot body 중심 contact를 넣어 support polygon을 넓혀도 PD standing은 통과하지 못했다. 따라서 현재 실패 원인은 contact 겹침 하나만이 아니라 다음 문제가 함께 섞여 있다.

- 실제 standing pose 미확정
- foot link origin과 visual/collision origin의 좌표계 불확실성
- ankle 계열 torque saturation
- 좌우 thigh/calf 질량 차이
- joint axis와 action sign의 CAD 기준 확인 필요

## 다음 단계

다음에는 임의 contact 수정이 아니라 CAD 기준 정보를 채워야 한다.

1. Fusion 360 또는 STEP assembly에서 foot link origin이 어디인지 확인한다.
2. visual mesh origin이 body-local 좌표인지 assembly-absolute 좌표인지 확인한다.
3. 실제 발바닥 패드 중심과 크기를 좌우 각각 확정한다.
4. 실제 양발 standing pose를 확정한다.
5. 위 값을 반영한 새 설계 기준 모델 variant를 만든다.
6. 그 모델에서 PD standing을 다시 실행한다.

현재 단계에서는 RL 학습으로 넘어가지 않는다.
