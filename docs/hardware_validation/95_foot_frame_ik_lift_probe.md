# 95. Foot-frame IK Lift Probe

## 목적

이 단계의 목적은 오른발 lift 목표를 joint angle offset이 아니라 `오른발 sole/contact pad의 실제 하단 코너 Z 좌표`로 직접 정의하는 것이다.

이전 단계까지 확인한 내용:

- standing은 통과
- weight shift는 통과
- right unload는 통과
- 오른발 clearance는 순간적으로 `9 mm` 이상 가능
- 하지만 `2 mm` 이상 clearance 지속시간은 약 `0.13~0.16 s`로, 목표 `0.5 s`에는 미달

따라서 이번에는 finite-difference IK로 오른발 sole bottom corner를 직접 올리는 포즈를 만들고, 그 포즈를 실제 동역학 시뮬레이션에 넣어 검증했다.

## 사용 모델

- `envs/robots/urdf_f_link/URDF_F_link_virtual_stl_footprint_contact.xml`

이 모델은 실제 STL footprint에 맞춘 발 접촉 패드를 사용한다.

## 추가 스크립트

- `scripts/solve_right_foot_lift_ik.py`

IK 대상:

- `right_hip_roll`
- `right_hip_pitch`
- `right_knee_pitch`
- `right_ankle_pitch`
- `right_ankle_roll`
- `left_hip_roll`
- `left_ankle_roll`

IK 목표:

- `foot_R_v1_1_sole_pad_*` geom의 bottom face corner Z 좌표를 seed pose 대비 지정한 lift만큼 상승

## 2 mm IK 포즈 생성

실행:

```bash
python3 scripts/solve_right_foot_lift_ik.py \
  --model envs/robots/urdf_f_link/URDF_F_link_virtual_stl_footprint_contact.xml \
  --seed-pose configs/weight_shift_left065_contact_constrained_multipoint.json \
  --out configs/right_foot_lift002_ik_stl_footprint_contact.json \
  --out-dir outputs/analysis/right_foot_lift002_ik_stl_footprint_contact \
  --lift 0.002 --iterations 18 --step 1e-4 --gain 0.65 --damping 1e-4 --max-dq 0.035
```

IK 결과:

| 항목 | 값 |
|---|---:|
| requested lift | 2.00 mm |
| achieved lift | 2.00 mm |
| initial min Z | -2.021 mm |
| final min Z | -0.021 mm |
| final clearance | -0.021 mm |

정적 기구학 기준으로는 목표 lift가 거의 정확히 달성됐다. 다만 2 mm 목표는 초기 발바닥 최저점이 지면보다 약 `2.02 mm` 아래에 있던 상태를 보정하는 수준이어서, 최종 clearance는 거의 `0 mm`에 가깝다.

## 2 mm IK 동역학 렌더 검증

실행:

```bash
MUJOCO_GL=egl python3 scripts/render_right_foot_lift_sequence.py \
  --model envs/robots/urdf_f_link/URDF_F_link_virtual_stl_footprint_contact.xml \
  --poses configs/weight_shift_left065_contact_constrained_multipoint.json configs/right_foot_lift002_ik_stl_footprint_contact.json configs/weight_shift_left065_contact_constrained_multipoint.json \
  --out-dir outputs/analysis/render_right_foot_lift002_ik_return_stl_footprint_contact_8s \
  --duration 8.0 --ramp 2.0 --hold 2.0 \
  --target-left-ratio 0.78 --joint-kp 20 --joint-kd 12 --torque-limit 30 \
  --kp-att 0 --kd-att 1 --kcom 1 --roll-sign -1 --pitch-sign 1 \
  --kforce 5 --force-sign 1 --fps 15 --azimuth 135 --elevation -12 --distance 1.45
```

산출물:

- 영상:
  - `outputs/analysis/render_right_foot_lift002_ik_return_stl_footprint_contact_8s/right_foot_lift_render.mp4`
- GIF:
  - `outputs/analysis/render_right_foot_lift002_ik_return_stl_footprint_contact_8s/right_foot_lift_render.gif`
- timeline:
  - `outputs/analysis/render_right_foot_lift002_ik_return_stl_footprint_contact_8s/right_foot_lift_timeline.csv`

결과:

| 항목 | 값 |
|---|---:|
| max right clearance | 9.12 mm |
| final right clearance | -0.236 mm |
| positive clearance duration | 0.132 s |
| clearance >= 2 mm duration | 0.132 s |
| clearance >= 5 mm duration | 0.132 s |
| right force <= 25 N duration | 6.996 s |
| right force <= 10 N duration | 0.198 s |
| right contacts <= 1 duration | 1.122 s |
| right contacts = 0 duration | 0.132 s |
| max roll | 0.117 rad |
| max pitch | 0.091 rad |
| max torque | 11.57 Nm |
| final right force | 23.40 N |
| final left force ratio | 0.741 |
| final right contacts | 4 |

## 5 mm IK 포즈 생성

2 mm 목표가 접촉 모델 오차에 너무 가까운지 확인하기 위해 5 mm 목표도 생성했다.

실행:

```bash
python3 scripts/solve_right_foot_lift_ik.py \
  --model envs/robots/urdf_f_link/URDF_F_link_virtual_stl_footprint_contact.xml \
  --seed-pose configs/weight_shift_left065_contact_constrained_multipoint.json \
  --out configs/right_foot_lift005_ik_stl_footprint_contact.json \
  --out-dir outputs/analysis/right_foot_lift005_ik_stl_footprint_contact \
  --lift 0.005 --iterations 24 --step 1e-4 --gain 0.65 --damping 1e-4 --max-dq 0.035
```

IK 결과:

| 항목 | 값 |
|---|---:|
| requested lift | 5.00 mm |
| achieved lift | 5.00 mm |
| initial min Z | -2.021 mm |
| final min Z | 2.979 mm |
| final clearance | 2.979 mm |

5 mm IK는 정적 기구학 기준으로 오른발 최저점을 지면 위 약 `2.98 mm`까지 올렸다.

## 5 mm IK 동역학 렌더 검증

실행:

```bash
MUJOCO_GL=egl python3 scripts/render_right_foot_lift_sequence.py \
  --model envs/robots/urdf_f_link/URDF_F_link_virtual_stl_footprint_contact.xml \
  --poses configs/weight_shift_left065_contact_constrained_multipoint.json configs/right_foot_lift005_ik_stl_footprint_contact.json configs/weight_shift_left065_contact_constrained_multipoint.json \
  --out-dir outputs/analysis/render_right_foot_lift005_ik_return_stl_footprint_contact_8s \
  --duration 8.0 --ramp 2.0 --hold 2.0 \
  --target-left-ratio 0.78 --joint-kp 20 --joint-kd 12 --torque-limit 30 \
  --kp-att 0 --kd-att 1 --kcom 1 --roll-sign -1 --pitch-sign 1 \
  --kforce 5 --force-sign 1 --fps 15 --azimuth 135 --elevation -12 --distance 1.45
```

산출물:

- 영상:
  - `outputs/analysis/render_right_foot_lift005_ik_return_stl_footprint_contact_8s/right_foot_lift_render.mp4`
- GIF:
  - `outputs/analysis/render_right_foot_lift005_ik_return_stl_footprint_contact_8s/right_foot_lift_render.gif`
- timeline:
  - `outputs/analysis/render_right_foot_lift005_ik_return_stl_footprint_contact_8s/right_foot_lift_timeline.csv`

결과:

| 항목 | 값 |
|---|---:|
| max right clearance | 9.12 mm |
| final right clearance | -0.299 mm |
| positive clearance duration | 0.132 s |
| clearance >= 2 mm duration | 0.132 s |
| clearance >= 5 mm duration | 0.132 s |
| right force <= 25 N duration | 3.630 s |
| right force <= 10 N duration | 0.198 s |
| right contacts <= 1 duration | 1.056 s |
| right contacts = 0 duration | 0.132 s |
| max roll | 0.117 rad |
| max pitch | 0.127 rad |
| max torque | 11.57 Nm |
| final right force | 24.69 N |
| final left force ratio | 0.727 |
| final right contacts | 3 |

## 비교

| 항목 | 2 mm IK | 5 mm IK |
|---|---:|---:|
| IK final clearance | -0.021 mm | 2.979 mm |
| dynamic max clearance | 9.12 mm | 9.12 mm |
| dynamic final clearance | -0.236 mm | -0.299 mm |
| clearance >= 2 mm duration | 0.132 s | 0.132 s |
| right force <= 25 N duration | 6.996 s | 3.630 s |
| right contacts <= 1 duration | 1.122 s | 1.056 s |
| max roll | 0.117 rad | 0.117 rad |
| max torque | 11.57 Nm | 11.57 Nm |

## 판단

IK는 포즈 생성 문제를 해결했다. 즉 오른발 sole bottom corner를 원하는 방향으로 올리는 joint target 자체는 만들 수 있다.

하지만 동역학 시뮬레이션에서는 2 mm와 5 mm 모두 sustained lift가 되지 않았다. 특히 5 mm IK 포즈도 `2 mm 이상 clearance`가 약 `0.132 s`만 유지됐기 때문에, 현재 병목은 단순한 foot target 높이 부족이 아니다.

현재 해석:

- 하드웨어 모델은 standing, weight shift, unload까지는 가능하다.
- transient toe-off도 가능하다.
- 하지만 한발 지지 상태에서 swing foot clearance를 유지하는 closed-loop balance/lift 제어가 아직 부족하다.
- 현 단계 결과만으로 “하드웨어 형상 때문에 절대 걸을 수 없다”고 결론내리기는 이르다.
- 반대로 “RL만 돌리면 충분히 걷는다”고 보기에도 아직 이르다. 최소한 one-leg support 안정화 gate를 더 명확히 통과해야 한다.

## 현재 Gate 상태

| 단계 | 결과 |
|---|---|
| standing | 통과 |
| weight shift | 통과 |
| right unload | 통과 |
| transient toe-off | 통과 |
| foot-frame IK pose generation | 통과 |
| 2 mm clearance 0.5초 유지 | 실패 |
| 5 mm clearance hold | 실패 |

## 다음 조치

다음 단계는 포즈 탐색을 계속 늘리는 것보다, 한발 지지 상태를 별도 gate로 분리하는 것이 맞다.

권장 진행:

1. 오른발을 강제로 들어 올리기 전에 왼발 단독 지지 상태를 만든다.
2. 목표를 `right force <= 5 N`, `right contacts = 0`, `roll <= 0.12 rad`로 둔다.
3. base/COM 목표를 왼발 support polygon 안쪽으로 제한한다.
4. 그 상태가 `0.5~1.0 s` 유지되면 foot-frame IK lift를 다시 붙인다.
5. 이 gate가 계속 실패하면 설계 수정 후보를 검토한다.

설계 수정 후보:

- 발바닥 support polygon 확대
- 발목 roll/pitch 축 위치 조정
- 상체 COM 위치 조정
- 발목 감속기/액추에이터 모델의 backlash, compliance, torque-speed 한계 반영
- 한발 지지 중 필요한 hip roll/ankle roll 토크 여유 재계산
