# Axis-aware Standing 과정 영상 기록

## 목적

70번 축 응답 감사에서 확인한 실제 joint 역할을 반영해 standing stabilizer를 다시 적용하고, 그 과정을 영상으로 남긴다.

이 영상은 성공한 standing 결과가 아니라, 현재 모델이 어떤 방식으로 실패하는지 눈으로 확인하기 위한 검증 산출물이다.

## 입력

- 모델: `/home/king0519/projects/Humanoid/envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_toeheel_z_grounded_step_visual.xml`
- pose: `/home/king0519/projects/Humanoid/configs/quasistatic_standing_pose_toeheel_z_step_visual_contact.json`
- 렌더 스크립트: `/home/king0519/projects/Humanoid/scripts/render_axis_aware_standing.py`
- 시뮬레이션 시간: `2.0 s`
- 영상 FPS: `30`
- 렌더 크기: `640 x 480`

## 제어 설정

기본 joint PD는 목표 pose를 유지하고, base roll/pitch 오차는 70번 문서의 실제 축 역할 기준으로 actuator에 분배했다.

| 항목 | 값 |
| --- | ---: |
| joint Kp | `60.0` |
| joint Kd | `4.0` |
| torque limit | `100.0 Nm` |
| attitude Kp | `4.0` |
| attitude Kd | `4.0` |
| COM 보정 K | `0.0` |
| roll sign | `-1.0` |
| pitch sign | `-1.0` |

## Axis-aware actuator 매핑

현재 MuJoCo 좌표 기준은 `X=전후`, `Y=좌우`, `Z=위`이다.

70번 감사에서 joint 이름과 실제 역할이 뒤바뀐 것이 확인되었으므로, 이번 영상에서는 이름이 아니라 실제 축 역할을 기준으로 제어했다.

| 제어 오차 | 사용 actuator |
| --- | --- |
| roll error | `motor_left_hip_pitch`, `motor_right_hip_pitch`, `motor_left_ankle_pitch`, `motor_right_ankle_pitch` |
| pitch error | `motor_left_hip_roll`, `motor_right_hip_roll`, `motor_left_ankle_roll`, `motor_right_ankle_roll` |

## 실행 명령

```bash
MUJOCO_GL=egl python3 scripts/render_axis_aware_standing.py --model envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_toeheel_z_grounded_step_visual.xml --pose-json configs/quasistatic_standing_pose_toeheel_z_step_visual_contact.json --out-dir outputs/analysis/render_axis_aware_standing_toeheel_z_step_visual_contact --duration 2.0 --joint-kp 60 --joint-kd 4 --torque-limit 100 --kp-att 4 --kd-att 4 --kcom 0 --roll-sign -1 --pitch-sign -1 --fps 30 --azimuth 135 --elevation -12 --distance 1.45
```

## 산출물

| 항목 | 경로 |
| --- | --- |
| MP4 영상 | `/home/king0519/projects/Humanoid/outputs/analysis/render_axis_aware_standing_toeheel_z_step_visual_contact/axis_aware_standing_render.mp4` |
| GIF 영상 | `/home/king0519/projects/Humanoid/outputs/analysis/render_axis_aware_standing_toeheel_z_step_visual_contact/axis_aware_standing_render.gif` |
| 첫 프레임 | `/home/king0519/projects/Humanoid/outputs/analysis/render_axis_aware_standing_toeheel_z_step_visual_contact/first_frame.png` |
| 중간 프레임 | `/home/king0519/projects/Humanoid/outputs/analysis/render_axis_aware_standing_toeheel_z_step_visual_contact/mid_frame.png` |
| 마지막 프레임 | `/home/king0519/projects/Humanoid/outputs/analysis/render_axis_aware_standing_toeheel_z_step_visual_contact/last_frame.png` |
| 요약 JSON | `/home/king0519/projects/Humanoid/outputs/analysis/render_axis_aware_standing_toeheel_z_step_visual_contact/render_summary.json` |

## 결과 요약

| 항목 | 값 |
| --- | ---: |
| frame 수 | `64` |
| contact frame fraction | `0.1875` |
| max qvel norm | `209.521596` |
| max contact force | `2040.403636 N` |
| max abs torque | `100.0 Nm` |
| final base z | `-0.000606 m` |
| final roll | `-1.909155 rad` |
| final pitch | `0.327093 rad` |
| final yaw | `0.859578 rad` |
| final qvel norm | `101.182099` |
| final contact force | `422.818389 N` |

## 프레임 관찰

- 첫 프레임: 양발 접촉이 잡히고 초기 자세는 거의 직립으로 시작한다.
- 중간 프레임: 약 `1.024 s`에서 접촉이 `0`이 되고 roll이 `+1.88 rad`까지 커진다.
- 마지막 프레임: 약 `2.0 s`에서 로봇이 바닥에 쓰러진 상태이며 roll이 `-1.91 rad`이다.
- torque는 `100 Nm` 제한에 도달한다.

## 판단

1. STEP visual 조립을 적용한 모델은 형상을 알아볼 수 있게 렌더링된다.
2. 축 역할을 반영한 stabilizer를 적용해도 현재 모델은 standing에 실패한다.
3. 실패 양상은 단순히 "영상이 이상한 것"이 아니라, 접촉 상실, 큰 roll 회전, torque saturation이 함께 나타나는 동역학 실패다.
4. 따라서 RL 학습으로 바로 넘어가기 전에 다음 항목을 더 검증해야 한다.

## 다음 조치

1. 실제 joint axis 방향/부호를 CAD 의도와 다시 대조한다.
2. 현재 quasi-static pose가 실제 support polygon 안에서 충분히 안정적인지 다시 확인한다.
3. base/link inertia frame과 COM 위치가 MuJoCo link frame 기준으로 맞는지 추가 감사한다.
4. 발 접촉 geom의 마찰, stiffness/solref/solimp를 보수적으로 조정해 접촉 수치 불안정성을 분리한다.
5. 이후 low-gain sink test와 high-gain fall test를 같은 영상 포맷으로 비교한다.
