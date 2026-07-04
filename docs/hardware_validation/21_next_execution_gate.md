# 다음 실행 조건 정리

## 목적

현재까지의 자동 검증만으로는 RL 학습을 시작하면 안 된다. 다음 PD standing probe를 의미 있게 실행하려면 CAD 또는 설계 기준 입력값이 먼저 채워져야 한다.

## 현재 기준 모델

- 모델: `/home/king0519/projects/Humanoid/envs/robots/urdf_f/URDF_F_named.xml`
- joint/action order: `/home/king0519/projects/Humanoid/configs/joint_mapping_provisional.json`
- 현재 상태: MuJoCo 컴파일 가능, torque actuator 10개 존재, 하지만 standing probe 실패

## 반드시 채워야 할 입력 파일

| 입력 파일 | 목적 | 현재 상태 |
| --- | --- | --- |
| `configs/cad_joint_confirmation_template.csv` | joint 이름, 역할, positive direction, limit 확정 | 비어 있음 |
| `configs/cad_mass_properties_template.csv` | CAD mass/COM/inertia와 MJCF 비교 | 비어 있음 |
| `configs/standing_pose_template.json` | 실제 standing pose 정의 | 비어 있음 |
| `configs/foot_contact_template.json` | 실제 발바닥 접촉 치수와 마찰 정의 | 비어 있음 |
| `configs/actuator_candidate_template.csv` | 후보 모터/감속기 사양 입력 | 비어 있음 |

## 제공된 정보로 미리 채운 파일

아래 파일은 현재 MJCF와 기존 분석 결과에서 확정적으로 알 수 있는 값만 미리 채운 초안이다. CAD에서만 확인 가능한 값은 여전히 비어 있거나 `확인 필요`로 남겨 두었다.

| Prefill 파일 | 채워진 내용 | 주의 |
| --- | --- | --- |
| `configs/prefilled/cad_joint_confirmation_prefilled.csv` | 임시 이름, 모델 축, 모델 joint limit | 실제 이름과 positive direction은 CAD 확인 필요 |
| `configs/prefilled/cad_mass_properties_prefilled.csv` | MJCF mass, inertial position, diagonal inertia | CAD mass/COM/inertia는 미확정 |
| `configs/prefilled/standing_pose_prefilled_from_failed_candidate.json` | 기존 geometry search pose 후보 | 동역학 standing 실패값이므로 확정 pose로 사용 금지 |
| `configs/prefilled/foot_contact_prefilled_placeholder.json` | 현재 placeholder foot box collision | 실제 발바닥 치수로 교체 필요 |

## 현재까지 확인된 문제

- neutral pose는 standing pose가 아니다.
- 단순 foot box collision만으로 standing이 안정화되지 않는다.
- geometry search pose는 COM 조건을 만족했지만 동역학 standing을 통과하지 못했다.
- placeholder actuator가 계속 torque limit에 닿는다.
- 좌우 thigh/calf body mass 차이가 CAD 확인 필요 수준이다.

## 다음 실행 순서

1. CAD에서 joint mapping과 positive direction을 확인한다.
2. CAD mass property를 템플릿에 채운다.
3. 실제 standing pose를 템플릿에 채운다.
4. 실제 foot contact 치수를 템플릿에 채운다.
5. 후보 actuator 사양을 템플릿에 채운다.
6. 템플릿 값을 반영한 새 MJCF를 생성한다.
7. PD standing probe를 다시 실행한다.
8. 안정화되면 squat, weight shift, one-leg support 순서로 넘어간다.

## RL 진행 조건

아래 조건을 만족하기 전에는 RL walking 학습을 시작하지 않는다.

- PD standing에서 base가 2초 이상 안정적으로 유지된다.
- torque saturation이 지속적으로 발생하지 않는다.
- 접촉력이 비정상적으로 튀지 않는다.
- COM projection이 support polygon 안에 유지된다.
- joint/action 이름과 positive direction이 확정되어 로그 해석이 가능하다.
