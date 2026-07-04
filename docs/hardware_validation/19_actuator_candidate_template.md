# Actuator 후보 입력 템플릿

## 목적

현재 MJCF actuator는 `+/-100 Nm` placeholder torque motor다. 이 값은 실제 모터/감속기 사양이 아니므로, torque saturation 결과를 하드웨어 문제로 해석하려면 joint별 후보 actuator 사양을 입력해야 한다.

## 템플릿 파일

- `/home/king0519/projects/Humanoid/configs/actuator_candidate_template.csv`

## 입력해야 할 값

| 컬럼 | 의미 |
| --- | --- |
| `joint_name` | named MJCF joint 이름 |
| `actuator_name` | named MJCF actuator 이름 |
| `motor_model` | 후보 모터 모델명 |
| `reduction_ratio` | 감속비 |
| `gear_efficiency` | 감속기 효율 |
| `continuous_output_torque_nm` | 출력축 기준 연속 토크 |
| `peak_output_torque_nm` | 출력축 기준 피크 토크 |
| `max_output_speed_rad_s` | 출력축 기준 최대 속도 |
| `rotor_inertia_kgm2` | 모터 rotor inertia |
| `estimated_reflected_inertia_kgm2` | 감속비 반영 후 관절에 보이는 등가 inertia |
| `thermal_time_limit_s` | 피크 토크를 유지할 수 있는 시간 |
| `cad_confirmed` | 실제 설계 반영 여부 |
| `notes` | 데이터시트 링크, 가정, 메모 |

## 다음 단계

이 CSV가 채워지면 MJCF의 `ctrlrange`, `forcerange`, armature, velocity 한계를 실제 후보 actuator 기준으로 업데이트할 수 있다. 그 후 PD standing, squat, weight shift에서 요구 torque와 후보 actuator margin을 비교한다.

