# Ankle/Hip Axis Response Audit

## 목적

서기 실패가 단순 pose/gain 문제가 아닌지 확인하기 위해, 현재 기준 pose에서 hip/ankle pitch-roll joint의 실제 회전축과 발바닥 모서리 높이 변화를 감사한다.

## 입력

- 모델: `/home/king0519/projects/Humanoid/envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_toeheel_z_grounded_step_visual.xml`
- pose: `/home/king0519/projects/Humanoid/configs/quasistatic_standing_pose_toeheel_z_step_visual_contact.json`
- delta: `+/-0.05` rad
- 좌표 기준: MuJoCo X=전후, Y=좌우, Z=위

## 핵심 요약

| joint | side | world axis | dominant role | front-back dz/rad | left-right dz/rad | center sensitivity/rad |
| --- | --- | --- | --- | ---: | ---: | --- |
| `left_hip_roll` | `left` | `+0.000 -1.000 +0.000` | `pitch_about_Y_lateral` | 0.039886 | 0.000735 | `[0.412869284583041, -6.938893903907228e-17, 0.09875517712893095]` |
| `left_hip_pitch` | `left` | `+0.997 +0.000 +0.077` | `roll_about_X_forward` | -0.000190 | 0.069167 | `[0.006630679448420829, 0.3520949175900223, -0.051808252624773454]` |
| `left_ankle_pitch` | `left` | `+0.992 -0.000 +0.127` | `roll_about_X_forward` | -0.000190 | 0.069167 | `[0.0029853272239263595, 0.023491753988297837, -0.023325601574907852]` |
| `left_ankle_roll` | `left` | `-0.017 -0.991 +0.131` | `pitch_about_Y_lateral` | 0.039751 | -0.000000 | `[0.02280483010974388, 0.0015354981094765907, 0.021559716097797818]` |
| `right_hip_roll` | `right` | `+0.000 -1.000 +0.000` | `pitch_about_Y_lateral` | 0.039976 | 0.001329 | `[0.4150597397327613, 0.0, 0.0548397864936152]` |
| `right_hip_pitch` | `right` | `-0.979 +0.000 -0.205` | `roll_about_X_forward` | 0.000726 | -0.067485 | `[-0.012805234201705411, -0.34540022843969914, 0.04873908190157705]` |
| `right_ankle_pitch` | `right` | `-0.967 -0.000 -0.254` | `roll_about_X_forward` | 0.000726 | -0.067485 | `[-0.0057959580028978976, -0.016781153109137747, 0.02206048459181799]` |
| `right_ankle_roll` | `right` | `-0.006 -1.000 +0.024` | `pitch_about_Y_lateral` | 0.039878 | 0.000000 | `[0.024280469094073998, -0.0019015969920413434, -0.019851502171062824]` |

## 해석 기준

- MuJoCo 기준으로 X축 회전은 roll 성분이다.
- MuJoCo 기준으로 Y축 회전은 pitch 성분이다.
- `front-back dz/rad`가 크면 발 앞뒤 높이를 바꾸는 pitch-like 효과가 크다.
- `left-right dz/rad`가 크면 발 좌우 높이를 바꾸는 roll-like 효과가 크다.

## 핵심 해석

현재 joint 이름과 MuJoCo 기준 실제 회전 역할이 서로 뒤바뀌어 있다.

| 이름 | 실제 MuJoCo dominant role |
| --- | --- |
| `left_hip_roll`, `right_hip_roll` | pitch-like, Y축 회전 |
| `left_hip_pitch`, `right_hip_pitch` | roll-like, X축 회전 |
| `left_ankle_pitch`, `right_ankle_pitch` | roll-like, X축 회전 |
| `left_ankle_roll`, `right_ankle_roll` | pitch-like, Y축 회전 |

따라서 55번/69번의 기존 stabilizer는 개념적으로 잘못된 매핑을 사용했다. 즉 roll 오차를 `*_roll` 이름의 actuator에 넣었지만, 현재 모델에서는 그 actuator들이 실제로 pitch-like 축을 돌린다. 반대로 pitch 오차를 `*_pitch` 이름의 actuator에 넣으면 실제로는 roll-like 축을 돌린다.

이것만으로 standing 실패가 전부 설명되는 것은 아니지만, controller/reward/action 설계 전에 반드시 정리해야 하는 문제다.

## Axis-aware Stabilizer 재시험

위 결과를 반영해 다음처럼 controller 매핑을 바꿔 간단히 재시험했다.

| 제어 오차 | 사용 actuator |
| --- | --- |
| roll error | `motor_left_hip_pitch`, `motor_right_hip_pitch`, `motor_left_ankle_pitch`, `motor_right_ankle_pitch` |
| pitch error | `motor_left_hip_roll`, `motor_right_hip_roll`, `motor_left_ankle_roll`, `motor_right_ankle_roll` |

결과:

| 항목 | 값 |
| --- | ---: |
| best final roll | `-1.909155 rad` |
| best final pitch | `0.327093 rad` |
| max roll | `3.120344 rad` |
| max qvel | `163.676239` |
| contact fraction | `0.199005` |
| saturation fraction | `0.990050` |

축 기준 매핑으로 바꿔도 standing은 통과하지 못했다. 즉 기존 controller 매핑 오류는 분명한 문제였지만, 그것을 보정해도 현재 하드웨어/동역학 모델은 아직 서지 못한다.

## 현재 판단

1. RL action 이름을 현재 `roll/pitch` 이름 그대로 쓰면 reward/controller 설계가 꼬인다.
2. 앞으로 제어기와 RL action space는 joint 이름이 아니라 실제 axis role 기준으로 매핑해야 한다.
3. 하지만 axis-aware stabilizer도 실패했으므로, 다음에는 joint axis 위치/방향, base/leg support pose, inertial frame, contact compliance를 더 봐야 한다.
4. 특히 ankle pitch/roll이라는 CAD 명칭이 MuJoCo global roll/pitch와 다를 수 있으므로, 문서에서는 CAD 명칭과 MuJoCo 동역학 역할을 분리해서 써야 한다.

## 산출물

- CSV: `/home/king0519/projects/Humanoid/outputs/analysis/ankle_axis_response_toeheel_z_step_visual_contact/ankle_axis_response.csv`
- JSON: `/home/king0519/projects/Humanoid/outputs/analysis/ankle_axis_response_toeheel_z_step_visual_contact/ankle_axis_response_summary.json`
- axis-aware stabilizer 결과: `outputs/analysis/axis_aware_stabilized_standing_toeheel_z_step_visual_contact/axis_aware_summary.json`
