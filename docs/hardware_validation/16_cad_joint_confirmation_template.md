# CAD Joint 확인 템플릿

## 목적

시뮬레이션에서 임시로 적용한 joint 이름과 CAD에서 확인한 실제 joint 정보를 비교하기 위한 입력 템플릿을 만든다.

## 템플릿 파일

- `/home/king0519/projects/Humanoid/configs/cad_joint_confirmation_template.csv`

## 컬럼 의미

| 컬럼 | 의미 |
| --- | --- |
| `exported_joint` | 원본 CAD/URDF export 이름 |
| `provisional_name` | 현재 시뮬레이션에서 임시로 쓰는 이름 |
| `cad_confirmed_name` | CAD에서 확인한 최종 이름 |
| `side` | left/right |
| `role_candidate` | 현재 추정한 역할 |
| `axis_in_model` | MJCF/URDF에 들어간 회전축 |
| `positive_direction_description` | `+` command가 실제로 만드는 회전 방향 |
| `joint_zero_pose_description` | `q=0`이 의미하는 조립 자세 |
| `lower_limit_rad` | CAD 또는 설계 기준 lower limit |
| `upper_limit_rad` | CAD 또는 설계 기준 upper limit |
| `cad_confirmed` | CAD 확인 완료 여부 |
| `notes` | 추가 메모 |

## 사용 방법

CAD에서 각 joint를 하나씩 확인한 뒤 `cad_confirmed_name`, `positive_direction_description`, `joint_zero_pose_description`, limit 값을 채운다.

이 CSV가 채워지면 다음 단계에서 `configs/joint_mapping_provisional.json`을 확정 mapping으로 업데이트하고, `URDF_F_named.xml`의 이름과 action order를 최종 기준으로 다시 생성한다.

