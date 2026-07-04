# 93. Clearance Duration Trajectory Search

## 목적

이 단계의 목적은 오른발 lift trajectory를 “최대 clearance”가 아니라 “clearance 유지 시간” 기준으로 다시 탐색하는 것이다.

이전 단계에서는 오른발이 순간적으로 9 mm 이상 떠오르지만, 양수 clearance 지속 시간이 약 `0.13 s`에 불과했다. 따라서 이번에는 다음 gate를 직접 점수화했다.

- right clearance `>= 2 mm`
- 안정 roll 조건에서 유지 시간 `>= 0.5 s`
- max roll `<= 0.18 rad`
- right force `<= 15 N`
- torque saturation `0`

## 사용 모델

- `envs/robots/urdf_f_link/URDF_F_link_virtual_stl_footprint_contact.xml`

## 추가 스크립트

- `scripts/search_right_foot_lift_trajectory.py`

이 스크립트는 기존 static pose 탐색과 달리 다음 trajectory 전체를 평가한다.

1. unload pose에서 시작
2. lift pose로 ramp
3. lift pose hold
4. unload pose로 return

평가 metric:

- `clearance_time_s`
- `stable_clearance_time_s`
- `low_force_time_s`
- `low_contact_time_s`
- `max_abs_roll`
- `max_abs_tau`
- `saturation_fraction`

## 탐색 조건

- seed pose:
  - `configs/weight_shift_left065_contact_constrained_multipoint.json`
- output pose:
  - `configs/right_foot_lift002_duration_stl_footprint_contact.json`
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
  - `180`

## 탐색 결과

산출물:

- summary:
  - `outputs/analysis/right_foot_lift002_duration_search_stl_footprint_contact/right_foot_lift_trajectory_search_summary.json`
- candidates:
  - `outputs/analysis/right_foot_lift002_duration_search_stl_footprint_contact/right_foot_lift_trajectory_candidates.csv`

Best candidate:

| 항목 | 값 |
|---|---:|
| stable clearance time | 0.140 s |
| clearance time | 0.140 s |
| max right clearance | 9.43 mm |
| final right clearance | -0.18 mm |
| low force time | 1.79 s |
| low contact time | 2.61 s |
| final left ratio | 0.787 |
| final right force | 19.26 N |
| final right contacts | 6 |
| max roll | 0.161 rad |
| max torque | 11.57 Nm |
| saturation | 0.0 |

## 렌더 검증

산출물:

- 영상:
  - `outputs/analysis/render_right_foot_lift002_duration_return_stl_footprint_contact_8s/right_foot_lift_render.mp4`
- timeline:
  - `outputs/analysis/render_right_foot_lift002_duration_return_stl_footprint_contact_8s/right_foot_lift_timeline.csv`
- summary:
  - `outputs/analysis/render_right_foot_lift002_duration_return_stl_footprint_contact_8s/right_foot_lift_summary.json`

렌더 timeline 계산 결과:

| 항목 | 값 |
|---|---:|
| positive clearance duration | 0.131 s |
| clearance >= 2 mm duration | 0.131 s |
| clearance >= 5 mm duration | 0.131 s |
| right force <= 15 N duration | 1.77 s |
| right force <= 25 N duration | 6.75 s |
| right contacts <= 1 duration | 2.56 s |
| max right clearance | 9.12 mm |
| final right clearance | -0.18 mm |
| max roll | 0.161 rad |
| final roll | 0.040 rad |

## 판단

실패 기준:

- 목표: `2 mm` 이상 clearance를 `0.5 s` 유지
- 결과: 약 `0.13~0.14 s`

따라서 2 mm lift hold gate는 아직 통과하지 못했다.

하지만 다음은 긍정적이다.

- roll은 안정 범위 안에 있다.
- torque saturation은 없다.
- 오른발 force를 낮게 유지하는 시간은 충분히 길다.
- 오른발 contact 수가 줄어드는 시간도 있다.
- 순간 clearance는 9 mm 이상 나온다.

즉 지금 상태는 “발을 완전히 들고 유지하는 swing”이 아니라 “toe-off 또는 순간 이탈”에 가깝다.

## 원인 해석

현재 접근 방식은 여전히 joint-space trajectory다. 오른발 sole clearance를 직접 제어하지 않는다.

따라서 다음 문제가 남는다.

1. 오른발을 접는 동안 foot orientation이 바뀌어 한쪽 pad가 다시 지면에 닿는다.
2. clearance가 커지는 순간 roll/yaw coupling이 생기며 지속 시간이 짧아진다.
3. knee/ankle/hip pitch를 독립 random offset으로 찾는 방식은 foot sole를 평행하게 들어 올리기 어렵다.
4. 실제 swing에서는 support leg, pelvis, swing leg를 동시에 조정해야 하는데, 지금은 주로 swing leg target만 바꾸고 있다.

## 다음 조치

다음 단계는 IK 또는 foot-frame 기반 trajectory로 넘어가야 한다.

최소 구현 방향:

1. 오른발 sole center와 네 contact corner의 world position을 계산한다.
2. target은 단순 joint offset이 아니라 `sole lowest point +2 mm`가 되도록 둔다.
3. knee/ankle pitch는 foot orientation을 유지하는 coupling으로 묶는다.
4. hip roll 또는 support side ankle roll로 base roll을 보정한다.
5. gate는 다음으로 둔다.
   - clearance `>= 2 mm` 유지 `>= 0.5 s`
   - max roll `<= 0.18 rad`
   - right force `<= 15 N`
   - saturation `0`

현재까지의 하드웨어 판단:

- standing 가능
- weight shift 가능
- right unload 가능
- 순간 toe-off 가능
- sustained swing clearance는 아직 trajectory/IK 제어가 필요

