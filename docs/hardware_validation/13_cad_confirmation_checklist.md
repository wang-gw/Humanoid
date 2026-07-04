# CAD 확인 체크리스트

## 목적

현재까지의 시뮬레이션 결과만으로는 RL 학습을 시작하기 어렵다. standing 실패의 원인이 하드웨어 형상인지, URDF/MJCF 변환 문제인지, pose/contact/controller 문제인지 분리하려면 CAD 기준 정보가 필요하다.

이 문서는 다음 검증 단계로 넘어가기 전에 CAD에서 확인해야 할 항목을 정리한다.

## Joint 확인

각 revolute joint마다 아래 항목을 확인한다.

| 항목 | 필요한 이유 |
| --- | --- |
| 실제 관절 이름 | `Revolute 19` 같은 자동 생성명을 RL action 이름으로 사용할 수 없음 |
| parent/child link | 조인트가 어떤 기구 branch에 속하는지 확인 |
| 회전축 방향 | pitch/roll/yaw 역할 판단 |
| positive direction | action `+`가 실제로 어떤 방향 회전인지 확인 |
| joint zero pose | `q=0`이 실제 조립 기준 자세인지 확인 |
| joint limit | 보행 자세와 충돌 회피 가능 여부 판단 |

확인 결과는 아래 CSV에 채운다.

- `/home/king0519/projects/Humanoid/configs/cad_joint_confirmation_template.csv`

## Standing Pose 확인

CAD 또는 IK 기준으로 아래 값을 정의해야 한다.

- base/pelvis 기준 높이
- 좌우 발바닥 중심 위치
- stance width
- 양발 toe/heel 방향
- hip, knee, ankle의 standing 각도
- COM projection이 support polygon 안에 들어오는지 여부

입력 템플릿:

- `/home/king0519/projects/Humanoid/configs/standing_pose_template.json`

## Foot Contact 확인

현재 foot box collision은 임시값이다. 실제 설계 기준으로 아래 값을 확정해야 한다.

- 발바닥 접촉면 길이
- 발바닥 접촉면 폭
- foot sole의 기준 좌표계 위치
- 고무 패드 또는 접촉재 마찰계수
- toe/heel이 별도 접촉점으로 나뉘는지 여부

입력 템플릿:

- `/home/king0519/projects/Humanoid/configs/foot_contact_template.json`

## Mass/Inertia 확인

URDF/MJCF 변환 과정에서 inertial 값이 합쳐지거나 바뀔 수 있으므로, CAD mass property와 비교해야 한다.

- 전체 질량
- 좌우 다리 질량 대칭성
- link별 질량
- link별 COM
- link별 inertia tensor
- actuator/감속기 질량이 link에 포함되어 있는지 여부

확인 결과는 아래 CSV에 채운다.

- `/home/king0519/projects/Humanoid/configs/cad_mass_properties_template.csv`

## 다음 실행 조건

아래 항목이 채워진 뒤에 PD standing을 다시 실행한다.

1. joint mapping 확정
2. positive direction 확정
3. 실제 standing pose 정의
4. 실제 foot contact 치수 반영
5. CAD mass property와 URDF/MJCF mass property 비교

## Actuator 후보 확인

현재 actuator는 placeholder이므로, 후보 모터/감속기 사양은 아래 CSV에 채운다.

- `/home/king0519/projects/Humanoid/configs/actuator_candidate_template.csv`
