# 92. 오른발 Lift Trajectory Probe

## 목적

이 단계의 목적은 STL footprint contact 기준 모델에서 오른발 lift를 정적 pose가 아니라 짧은 trajectory로 시도하는 것이다.

이전 단계에서 확인한 상태:

- standing 통과
- `0.65` weight shift 통과
- `0.78` right unload 통과
- 오른발 5 mm static lift hold 실패

따라서 이번에는 목표 clearance를 낮춰 `2 mm` lift부터 다시 탐색하고, unload → lift → unload return trajectory를 확인했다.

## 사용 모델

- `envs/robots/urdf_f_link/URDF_F_link_virtual_stl_footprint_contact.xml`

## 사용 스크립트

- 탐색:
  - `scripts/search_right_foot_lift_pose.py`
- 렌더:
  - `scripts/render_right_foot_lift_sequence.py`

## 1. 오른발 2 mm Lift Pose 탐색

### 산출물

- pose:
  - `configs/right_foot_lift002_stl_footprint_contact.json`
- summary:
  - `outputs/analysis/right_foot_lift002_search_stl_footprint_contact/right_foot_lift_search_summary.json`
- candidates:
  - `outputs/analysis/right_foot_lift002_search_stl_footprint_contact/right_foot_lift_candidates.csv`

### Best candidate

| 항목 | 값 |
|---|---:|
| max right clearance | 9.43 mm |
| final right clearance | -0.26 mm |
| final left ratio | 0.875 |
| final right force | 11.31 N |
| final right contacts | 1 |
| final roll | 0.153 rad |
| max torque | 11.57 Nm |
| saturation | 0.0 |

### 판단

2 mm 목표에서도 최종 clearance는 양수가 되지 않았다.

다만 오른발 normal force는 `11.31 N`까지 내려갔고, 오른발 contact도 `1`개만 남았다. 즉 오른발은 거의 unload 상태까지 가지만, 완전히 지면에서 떠서 유지되지는 않는다.

## 2. Stable Candidate 분석

후보군에서 안정 조건을 다음처럼 두고 다시 필터링했다.

- max roll `<= 0.22 rad`
- torque saturation `0`
- max torque `< 30 Nm`

그 결과:

- 안정 후보 중 최종 clearance가 양수인 후보는 없었다.
- 안정 후보의 best final clearance는 약 `-0.037 mm`였다.
- 최종 clearance가 양수인 후보들은 모두 roll이 크게 무너졌다.

즉 현재 joint-space target 탐색 범위에서는 “오른발을 들어서 유지하는 안정 자세”가 아직 발견되지 않았다.

## 3. Unload → Lift → Return Trajectory

정적 hold가 안 되므로, 짧은 trajectory를 시도했다.

Trajectory:

1. `0.65` weight shift pose
2. `2 mm` lift 후보 pose
3. 다시 `0.65` weight shift pose

### 산출물

- 영상:
  - `outputs/analysis/render_right_foot_lift002_return_stl_footprint_contact_9s/right_foot_lift_render.mp4`
- timeline:
  - `outputs/analysis/render_right_foot_lift002_return_stl_footprint_contact_9s/right_foot_lift_timeline.csv`
- summary:
  - `outputs/analysis/render_right_foot_lift002_return_stl_footprint_contact_9s/right_foot_lift_summary.json`

### 결과

| 항목 | 값 |
|---|---:|
| max right clearance | 9.12 mm |
| final right clearance | -0.18 mm |
| positive clearance duration | 약 0.13 s |
| right force <= 25 N duration | 약 7.75 s |
| right contacts <= 1 duration | 약 2.63 s |
| max roll | 0.153 rad |
| final roll | 0.024 rad |
| max torque | 11.57 Nm |
| saturation | 0.0 |

### 판단

부분 성공이다.

오른발이 순간적으로 떠오르는 구간은 있다. max clearance는 `9.12 mm`까지 올라갔다. 또한 trajectory 전체에서 roll은 안정적이고 torque saturation도 없다.

하지만 clearance가 양수인 시간은 약 `0.13 s`로 매우 짧다. 최종적으로는 다시 접촉한다. 따라서 이 결과를 “swing foot lift hold 성공”으로 보기는 어렵다.

## 종합 판단

현재 단계의 결론:

| 단계 | 결과 |
|---|---|
| right unload | 통과 |
| 순간 right lift | 부분 성공 |
| right lift hold | 실패 |
| lift 후 안정 return | 통과 |

중요한 점:

- 모터 토크는 아직 병목으로 보이지 않는다.
- 오른발 하중은 충분히 줄일 수 있다.
- 발을 순간적으로 띄울 수는 있다.
- 하지만 단일 joint-space target 또는 단순 return trajectory로는 clearance를 유지하지 못한다.

따라서 다음 병목은 하드웨어 토크라기보다 다음 쪽에 가깝다.

1. 오른발 lift를 위한 joint-space trajectory 설계가 부족함
2. lift 중 COM/roll을 동시에 제어하는 strategy가 부족함
3. IK 기반으로 foot clearance를 직접 목표로 삼지 않고 joint offset random search만 사용한 한계

## 다음 조치

다음 단계에서는 joint offset random search를 계속 늘리기보다, 오른발 foot 위치를 직접 보는 trajectory/IK 방식으로 바꿔야 한다.

추천 순서:

1. 오른발 foot body 또는 sole center의 world position을 기준으로 lift metric을 만든다.
2. right knee/ankle pitch를 coordinated pattern으로 움직이는 parameterized trajectory를 만든다.
3. 목표를 `right clearance >= 2 mm`로 두되, 최소 유지 시간을 gate에 포함한다.
4. gate:
   - clearance `>= 2 mm` 유지 `>= 0.5 s`
   - max roll `<= 0.18 rad`
   - right force `<= 15 N`
   - saturation `0`

