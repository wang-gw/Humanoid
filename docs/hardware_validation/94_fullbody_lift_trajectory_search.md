# 94. Full-body Lift Trajectory Search

## 목적

이 단계의 목적은 오른발 lift trajectory에서 swing leg만 움직이던 한계를 줄이기 위해 왼쪽 지지다리까지 함께 보정하는 것이다.

이전 단계 결과:

- 오른발 clearance는 순간적으로 `9 mm` 이상 가능
- 하지만 `2 mm` 이상 유지 시간은 약 `0.13~0.14 s`
- 목표 gate `0.5 s`에는 미달

따라서 이번에는 오른발 lift와 동시에 왼쪽 hip/ankle roll, pitch 계열 보정을 포함한 full-body trajectory search를 수행했다.

## 사용 모델

- `envs/robots/urdf_f_link/URDF_F_link_virtual_stl_footprint_contact.xml`

## 추가 스크립트

- `scripts/search_right_foot_lift_fullbody_trajectory.py`

탐색 대상 joint:

- left hip roll/pitch
- left knee pitch
- left ankle pitch/roll
- right hip roll/pitch
- right knee pitch
- right ankle pitch/roll

## 탐색 조건

- seed pose:
  - `configs/weight_shift_left065_contact_constrained_multipoint.json`
- output pose:
  - `configs/right_foot_lift002_fullbody_duration_stl_footprint_contact.json`
- target clearance:
  - `2 mm`
- target stable clearance time:
  - `0.5 s`
- duration:
  - `8 s`
- lift ramp:
  - `2 s`
- lift hold:
  - `2 s`
- return ramp:
  - `2 s`
- samples:
  - `220`

## Full-body 탐색 결과

산출물:

- summary:
  - `outputs/analysis/right_foot_lift002_fullbody_duration_search_stl_footprint_contact/right_foot_lift_fullbody_trajectory_search_summary.json`
- candidates:
  - `outputs/analysis/right_foot_lift002_fullbody_duration_search_stl_footprint_contact/right_foot_lift_fullbody_trajectory_candidates.csv`

Best candidate:

| 항목 | 값 |
|---|---:|
| stable clearance time | 0.140 s |
| clearance time | 0.140 s |
| low force time | 4.216 s |
| low contact time | 3.870 s |
| max right clearance | 9.43 mm |
| final right clearance | -0.20 mm |
| final right force | 19.50 N |
| final right contacts | 4 |
| max roll | 0.116 rad |
| final roll | 0.021 rad |
| max torque | 11.57 Nm |
| saturation | 0.0 |

## 렌더 검증

산출물:

- 영상:
  - `outputs/analysis/render_right_foot_lift002_fullbody_duration_return_stl_footprint_contact_8s/right_foot_lift_render.mp4`
- timeline:
  - `outputs/analysis/render_right_foot_lift002_fullbody_duration_return_stl_footprint_contact_8s/right_foot_lift_timeline.csv`

렌더 timeline:

| 항목 | full-body | swing-leg-only 이전 결과 |
|---|---:|---:|
| clearance >= 2 mm duration | 0.131 s | 0.131 s |
| positive clearance duration | 0.131 s | 0.131 s |
| right force <= 15 N duration | 4.20 s | 1.77 s |
| right force <= 25 N duration | 7.54 s | 6.75 s |
| right contacts <= 1 duration | 3.80 s | 2.56 s |
| max right clearance | 9.12 mm | 9.12 mm |
| max roll | 0.116 rad | 0.161 rad |
| final roll | 0.021 rad | 0.040 rad |

## 짧은 Return 후보 확인

탐색 후보 중 일부는 `0.5 s` 이상 clearance time을 보였지만, 장기 시뮬레이션에서는 roll이 붕괴했다. 그래서 그 후보를 짧게 잘라 `lift 후 즉시 return`하는 3초/4초 trajectory로 확인했다.

선택 pose:

- `configs/right_foot_lift002_short_return_candidate_stl_footprint_contact.json`

산출물:

- 3초 영상:
  - `outputs/analysis/render_right_foot_lift002_short_return_candidate_stl_footprint_contact_3s/right_foot_lift_render.mp4`
- 4초 영상:
  - `outputs/analysis/render_right_foot_lift002_short_return_candidate_stl_footprint_contact_4s/right_foot_lift_render.mp4`

결과:

| 항목 | 3초 return | 4초 return |
|---|---:|---:|
| clearance >= 2 mm duration | 0.160 s | 0.160 s |
| clearance >= 5 mm duration | 0.096 s | 0.096 s |
| right force <= 15 N duration | 0.766 s | 1.120 s |
| right contacts <= 1 duration | 1.915 s | 2.784 s |
| max right clearance | 9.01 mm | 9.01 mm |
| max roll | 0.118 rad | 0.118 rad |
| final roll | 0.079 rad | 0.066 rad |

짧게 잘라내면 roll 붕괴는 피할 수 있다. 하지만 clearance 유지 시간은 여전히 `0.16 s` 정도로, 목표 `0.5 s`에는 미달한다.

## 판단

Full-body 보정은 다음을 개선했다.

- max roll 감소
- right force 낮은 상태 유지 시간 증가
- right contact 수 감소 시간 증가
- lift 후 return 안정성 개선

하지만 핵심 목표인 sustained clearance는 개선하지 못했다.

즉 현재 문제는 단순히 지지다리 roll 보정 부족만은 아니다. 오른발 sole 자체를 지면과 평행하게, 그리고 충분한 시간 동안 띄우는 foot-frame 제어가 필요하다.

## 현재 Gate 상태

| 단계 | 결과 |
|---|---|
| standing | 통과 |
| weight shift | 통과 |
| right unload | 통과 |
| transient toe-off | 통과 |
| 2 mm clearance 0.5초 유지 | 실패 |
| 5 mm clearance hold | 실패 |

## 다음 조치

다음부터는 random joint offset/pose trajectory 탐색을 계속 늘리는 것은 효율이 낮다.

다음 구현 방향:

1. MuJoCo Jacobian 또는 finite-difference IK로 오른발 sole center/corner 위치를 직접 제어한다.
2. 목표를 joint angle이 아니라 `right sole lowest point >= 2 mm`로 둔다.
3. lift 중 foot orientation을 유지하도록 ankle pitch/roll을 함께 보정한다.
4. support leg는 roll/yaw 안정화용 보조 제어로만 둔다.

최소 다음 gate:

- right clearance `>= 2 mm` for `>= 0.5 s`
- max roll `<= 0.18 rad`
- right force `<= 15 N`
- saturation `0`

