# 초기 URDF/Xacro 모델 감사

## 출처

- 모델 파일: `/home/king0519/projects/Humanoid/URDF_F_description/URDF_F_description/urdf/URDF_F.xacro`
- 목적: RL 보행 가능성과 하드웨어 sanity validation의 기준선 확보

## 요약

- mass entry가 있는 link 수: `23`
- 모델 총 질량: `8.623984 kg`
- 전체 joint 수: `22`
- revolute joint 수: `10`
- fixed joint 수: `12`

## 발견 사항

- 10개 revolute joint가 CAD 자동 생성 이름을 사용한다.
- 모든 revolute joint의 effort limit이 `100.0`으로 동일하다.
- 모든 revolute joint의 velocity limit이 `100.0`으로 동일하다.
- 23개 link가 mesh collision geometry를 사용하므로 단순 collision을 고려해야 한다.
- 좌우 link mass pair 중 2개가 허용 오차보다 크게 다르다.

## Revolute Joint 목록

| Joint | Parent | Child | Axis | Lower rad | Upper rad | Effort | Velocity |
| --- | --- | --- | --- | ---: | ---: | ---: | ---: |
| Revolute 19 | AK45-36_tpL_1 | thigh_L_1 | 1 -0 -0 | -1.570796 | 1.570796 | 100 | 100 |
| Revolute 21 | thigh_L_1 | calf_L_1 | 1 0 -0 | -1.570796 | 1.570796 | 100 | 100 |
| Revolute 23 | calf_L_1 | footJ_L_1 | 1 -0 -0 | -1.570796 | 1.570796 | 100 | 100 |
| Revolute 25 | AK45-10_frL_1 | foot_L_1 | 0 -1 0 | -1.570796 | 1.570796 | 100 | 100 |
| Revolute 26 | AK45-36_trL_1 | thighJ_L_1 | 0 -1 -0 | -1.570796 | 1.570796 | 100 | 100 |
| Revolute 44 | AK45-36_tpR_1 | thigh_R_1 | -1 -0 -0 | -1.570796 | 1.570796 | 100 | 100 |
| Revolute 46 | thigh_R_1 | calf_R_1 | 1 0 0 | -1.570796 | 1.570796 | 100 | 100 |
| Revolute 48 | calf_R_1 | footJ_R_1 | -1 -0 -0 | -1.570796 | 1.570796 | 100 | 100 |
| Revolute 51 | AK45-36_trR_1 | thighJ_R_1 | -0 -1 -0 | -1.570796 | 1.570796 | 100 | 100 |
| Revolute 53 | AK45-10_R_1 | foot_R_v1_1 | -0 -1 -0 | -1.570796 | 1.570796 | 100 | 100 |

## 다음 검증 단계

1. 모델을 MuJoCo에서 로드 가능한 asset으로 변환한다.
2. neutral standing pose를 정의하고 지면 접촉 안정성을 확인한다.
3. PD standing, squat, weight-shift, one-leg-support probe를 실행한다.
4. joint torque, joint velocity, base pose, contact force, failure reason을 기록한다.
