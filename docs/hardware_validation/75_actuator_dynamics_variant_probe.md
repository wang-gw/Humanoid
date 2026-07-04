# Actuator Dynamics 변형 모델 Probe

## 목적

74번 all-joint free impulse 감사에서 ankle 계열이 작은 토크에도 과도하게 움직이는 것을 확인했다.

이번 단계에서는 원본 모델은 유지하고, 실험용으로 joint `armature`, `damping`, `frictionloss`를 추가한 변형 모델을 만들어 다음을 확인했다.

1. ankle 과민성이 줄어드는가?
2. standing PD 실패 양상이 개선되는가?
3. 이 문제가 RL 전에 모델링 보정 항목으로 봐야 할 정도로 중요한가?

## 입력

- 원본 모델: `/home/king0519/projects/Humanoid/envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_toeheel_z_grounded_step_visual.xml`
- pose: `/home/king0519/projects/Humanoid/configs/quasistatic_standing_pose_toeheel_z_step_visual_contact.json`
- 생성 스크립트: `/home/king0519/projects/Humanoid/scripts/build_actuator_dynamics_variant.py`

## 변형 모델 생성

실행:

```bash
python3 scripts/build_actuator_dynamics_variant.py --input envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_toeheel_z_grounded_step_visual.xml --output envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_toeheel_z_grounded_step_visual_actuator_dynamics.xml --report docs/hardware_validation/actuator_dynamics_variant_report.json
```

생성 모델:

```text
envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_toeheel_z_grounded_step_visual_actuator_dynamics.xml
```

적용값:

| joint 계열 | armature | damping | frictionloss |
| --- | ---: | ---: | ---: |
| hip | `0.02` | `0.2` | `0.02` |
| knee | `0.02` | `0.2` | `0.02` |
| ankle | `0.05` | `0.4` | `0.02` |

주의: 이 값은 최종 모터/감속기 값이 아니다. 실제 rotor inertia, gear ratio, 감속기 마찰, 관절 마찰이 아직 확정되지 않았기 때문에, 이번 값은 원인 분리를 위한 보수적 실험값이다.

로드 확인:

| 항목 | 값 |
| --- | ---: |
| nq | `17` |
| nv | `16` |
| nu | `10` |
| nbody | `12` |
| njnt | `11` |
| ngeom | `26` |
| timestep | `0.002` |

## 1. Free impulse 재검증

조건:

- gravity: `[0, 0, 0]`
- impulse torque: `0.2 Nm`
- impulse duration: `0.03 s`
- total duration: `0.12 s`

원본 모델과 actuator dynamics 변형 모델의 평균 joint 변위 비교:

| actuator | 원본 avg abs delta q | dynamics avg abs delta q |
| --- | ---: | ---: |
| `motor_left_ankle_pitch` | `0.382509` | `0.005670` |
| `motor_left_ankle_roll` | `0.319370` | `0.005666` |
| `motor_right_ankle_roll` | `0.227513` | `0.005638` |
| `motor_right_ankle_pitch` | `0.124710` | `0.005488` |
| `motor_left_knee_pitch` | `0.032574` | `0.006846` |
| `motor_left_hip_pitch` | `0.013425` | `0.003591` |
| `motor_right_knee_pitch` | `0.013181` | `0.005243` |
| `motor_right_hip_pitch` | `0.006153` | `0.002800` |
| `motor_left_hip_roll` | `0.002932` | `0.001796` |
| `motor_right_hip_roll` | `0.002655` | `0.001649` |

해석:

- ankle 과민성이 크게 줄었다.
- 특히 `left_ankle_pitch`는 `0.3825 rad`에서 `0.00567 rad`로 줄었다.
- 이 결과는 원본 모델의 ankle dynamics가 실제 actuator/감속기 관성 또는 damping을 충분히 반영하지 못하고 있었을 가능성을 강하게 시사한다.

## 2. Standing PD 비교

조건:

- pose: `quasistatic_standing_pose_toeheel_z_step_visual_contact.json`
- Kp: `60`
- Kd: `4`
- torque limit: `100 Nm`
- duration: `2.0 s`

| 항목 | 원본 모델 | actuator dynamics 모델 |
| --- | ---: | ---: |
| final base z | `0.016142 m` | `-0.070262 m` |
| final roll | `2.607548 rad` | `-1.343365 rad` |
| final pitch | `-0.273205 rad` | `0.155590 rad` |
| max qvel norm | `151.063643` | `6.921128` |
| max contact force | `1684.520266 N` | `370.361989 N` |
| max observed torque | `100 Nm` | `13.536317 Nm` |

해석:

- 아직 standing 성공은 아니다.
- 하지만 실패 양상은 크게 개선되었다.
- 원본 모델은 빠르게 튀고 torque saturation에 들어갔다.
- dynamics 변형 모델은 훨씬 느리게 기울며, 최대 torque도 `13.5 Nm` 수준에 머문다.

즉 현재 standing 실패에는 actuator/감속기 동역학 누락이 큰 영향을 주고 있다.

## 영상 산출물

| 항목 | 경로 |
| --- | --- |
| MP4 | `/home/king0519/projects/Humanoid/outputs/analysis/render_pd_standing_actuator_dynamics_pose/pd_standing_render.mp4` |
| GIF | `/home/king0519/projects/Humanoid/outputs/analysis/render_pd_standing_actuator_dynamics_pose/pd_standing_render.gif` |
| 첫 프레임 | `/home/king0519/projects/Humanoid/outputs/analysis/render_pd_standing_actuator_dynamics_pose/first_frame.png` |
| 중간 프레임 | `/home/king0519/projects/Humanoid/outputs/analysis/render_pd_standing_actuator_dynamics_pose/mid_frame.png` |
| 마지막 프레임 | `/home/king0519/projects/Humanoid/outputs/analysis/render_pd_standing_actuator_dynamics_pose/last_frame.png` |

프레임 관찰:

- `0.0 s`: contacts `3`, roll/pitch 거의 `0`
- `1.024 s`: contacts `3`, roll `+0.59 rad`, max torque `3.0 Nm`
- `2.0 s`: contacts `2`, roll `-1.34 rad`, max torque `13.5 Nm`

## 현재 판단

이 단계에서 처음으로 failure dynamics가 크게 완화되었다.

따라서 다음 결론을 둔다.

1. 원본 모델 그대로 RL을 돌리면 ankle dynamics가 너무 가벼워 학습이 비현실적으로 불안정할 가능성이 크다.
2. 최종 모터 선정 전이라도, actuator dynamics placeholder는 반드시 필요하다.
3. 다만 지금 넣은 `armature/damping/frictionloss` 값은 최종값이 아니므로, 실제 AK45-36/AK45-10의 rotor inertia, gear ratio, output damping, friction에 맞춰 다시 산정해야 한다.
4. standing 성공까지는 아직 부족하므로, 다음에는 dynamics 변형 모델 기준으로 pose/gain/stabilizer를 다시 탐색해야 한다.

## 산출물

- 변형 모델: `/home/king0519/projects/Humanoid/envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_toeheel_z_grounded_step_visual_actuator_dynamics.xml`
- 변형 보고서: `/home/king0519/projects/Humanoid/docs/hardware_validation/actuator_dynamics_variant_report.json`
- free impulse CSV: `/home/king0519/projects/Humanoid/outputs/analysis/all_joint_free_impulse_response_actuator_dynamics_small02/all_joint_free_impulse_response_summary.csv`
- PD summary JSON: `/home/king0519/projects/Humanoid/outputs/analysis/pd_standing_actuator_dynamics_pose/neutral_pd_standing_summary.json`
- render summary JSON: `/home/king0519/projects/Humanoid/outputs/analysis/render_pd_standing_actuator_dynamics_pose/render_summary.json`

## 다음 조치

1. actuator dynamics 모델 기준으로 standing pose를 재탐색한다.
2. 재탐색 pose에서 low/high gain PD sweep를 다시 수행한다.
3. axis-aware stabilizer도 dynamics 모델 기준으로 다시 sweep한다.
4. 그 후에도 standing이 안 되면 joint axis/foot contact geometry를 다시 수정한다.
