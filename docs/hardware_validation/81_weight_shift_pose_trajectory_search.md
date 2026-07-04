# Weight Shift Pose Trajectory Search

## 목적

80번에서 다점 contact 모델은 standing을 통과했지만, 단순 lateral target 방식으로는 충분한 좌우 하중 이동이 나오지 않았다.

이번 단계에서는 left/right normal force ratio를 직접 목표로 두고 weight-shift pose를 탐색했다. 또한 탐색된 pose로 standing pose에서 실제 전이할 수 있는지 영상으로 확인했다.

## 입력

- 모델: `/home/king0519/projects/Humanoid/envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_toeheel_z_grounded_step_visual_actuator_dynamics_multipoint_fullheight_contact.xml`
- 시작 pose: `/home/king0519/projects/Humanoid/configs/dynamic_standing_pose_actuator_dynamics_contact_5s.json`
- pose 탐색 스크립트: `/home/king0519/projects/Humanoid/scripts/search_weight_shift_pose.py`
- pose 전이 렌더 스크립트: `/home/king0519/projects/Humanoid/scripts/render_pose_transition.py`

## 방법

`search_weight_shift_pose.py`는 후보 joint target을 만들고, MuJoCo rollout 후 아래 항목으로 점수를 계산한다.

- 최종 `left_force_ratio`가 목표값에 가까운지
- roll/pitch/yaw가 작은지
- qvel이 작은지
- contact를 유지하는지
- torque saturation이 없는지
- 최대 torque가 과도하지 않은지

이 방식은 보행 제어기가 아니라, RL 전에 가능한 quasi-static weight shift milestone을 찾기 위한 검증이다.

## Target 0.75 정적 pose

목표:

```text
left_force_ratio = 0.75
```

탐색 결과:

| 항목 | 값 |
| --- | ---: |
| final left force ratio | `0.760891` |
| ratio error | `0.010891` |
| final roll | `0.080937 rad` |
| final pitch | `-0.001804 rad` |
| final yaw | `0.139609 rad` |
| final qvel norm | `0.023124` |
| max qvel norm | `0.516027` |
| max contact force | `98.471406 N` |
| max torque | `3.719370 Nm` |
| contact fraction | `1.0` |
| saturation fraction | `0.0` |

정적 유지 관점에서는 좋은 후보다.

생성 pose:

```text
configs/weight_shift_left075_pose_multipoint.json
```

하지만 standing pose에서 이 pose로 4초 ramp 전이를 수행하면 실패했다.

전이 결과:

| 항목 | 값 |
| --- | ---: |
| final left force ratio | `0.731734` |
| final roll | `-1.734438 rad` |
| final pitch | `0.406191 rad` |
| final qvel norm | `5.778387` |
| final contacts | `2` |
| max torque | `13.661892 Nm` |
| saturation fraction | `0.0` |

느린 8초 ramp도 실패했다.

| 항목 | 값 |
| --- | ---: |
| final left force ratio | `0.306894` |
| final roll | `2.632773 rad` |
| final qvel norm | `2.383400` |
| max contact force | `326.890876 N` |
| max torque | `18.902448 Nm` |

판단:

- `0.75` pose는 그 상태에서 시작하면 잠깐 유지 가능하다.
- 하지만 현재 controller와 단순 joint interpolation으로는 standing에서 안정적으로 진입할 수 없다.
- 즉 실제 trajectory milestone으로는 아직 부적합하다.

영상:

```text
outputs/analysis/render_pose_transition_left075_multipoint/pose_transition_render.mp4
outputs/analysis/render_pose_transition_left075_multipoint_slow/pose_transition_render.mp4
```

## Target 0.65 정적 pose

목표:

```text
left_force_ratio = 0.65
```

탐색 결과:

| 항목 | 값 |
| --- | ---: |
| final left force ratio | `0.662330` |
| ratio error | `0.012330` |
| final roll | `0.049658 rad` |
| final pitch | `0.009435 rad` |
| final yaw | `0.060926 rad` |
| final qvel norm | `0.011812` |
| max qvel norm | `0.689974` |
| max contact force | `106.693581 N` |
| max torque | `3.905233 Nm` |
| contact fraction | `1.0` |
| saturation fraction | `0.0` |

정적 pose는 가능해 보인다.

생성 pose:

```text
configs/weight_shift_left065_pose_multipoint.json
```

하지만 standing pose에서 4초 ramp 전이는 실패했다.

전이 결과:

| 항목 | 값 |
| --- | ---: |
| final left force ratio | `0.624999` |
| final roll | `2.062795 rad` |
| final pitch | `0.171617 rad` |
| final qvel norm | `1.986067` |
| final contacts | `2` |
| max qvel norm | `7.638065` |
| max contact force | `355.081251 N` |
| max torque | `17.875176 Nm` |

판단:

- `0.65`도 정적 pose는 가능하지만, 단순 전이 trajectory는 실패한다.
- 이 수준부터는 중간 경로 또는 controller가 별도로 필요하다.

영상:

```text
outputs/analysis/render_pose_transition_left065_multipoint/pose_transition_render.mp4
```

## Target 0.60 정적 pose 및 전이

목표:

```text
left_force_ratio = 0.60
```

탐색 결과:

| 항목 | 값 |
| --- | ---: |
| final left force ratio | `0.573059` |
| ratio error | `0.026941` |
| final roll | `0.021204 rad` |
| final pitch | `-0.016452 rad` |
| final yaw | `0.045853 rad` |
| final qvel norm | `0.012370` |
| max qvel norm | `0.584373` |
| max contact force | `102.586846 N` |
| max torque | `3.794902 Nm` |
| contact fraction | `1.0` |
| saturation fraction | `0.0` |

탐색은 목표 `0.60`에 정확히 도달하지 못하고 `0.573` 근처에서 안정 후보를 찾았다.

생성 pose:

```text
configs/weight_shift_left060_pose_multipoint.json
```

standing pose에서 4초 ramp 전이는 성공했다.

전이 결과:

| 항목 | 값 |
| --- | ---: |
| final left force ratio | `0.581699` |
| final roll | `0.013051 rad` |
| final pitch | `-0.009576 rad` |
| final qvel norm | `0.005567` |
| final contacts | `10` |
| max abs roll | `0.053932 rad` |
| max abs pitch | `0.035147 rad` |
| max qvel norm | `0.570226` |
| max contact force | `99.856370 N` |
| max torque | `3.798335 Nm` |
| contact frame fraction | `1.0` |
| saturation fraction | `0.0` |

판단:

- 현재 단순 joint interpolation과 axis-aware stabilizer로 실제 진입 가능한 안정 weight shift milestone은 약 `left_force_ratio = 0.58` 수준이다.
- 이 수준은 아직 swing foot을 만들기에는 부족하지만, standing에서 weight shift로 넘어가는 첫 안정 milestone으로 사용할 수 있다.

영상:

```text
outputs/analysis/render_pose_transition_left060_multipoint/pose_transition_render.mp4
```

## 현재 결론

현재 모델은 아래 순서까지 검증됐다.

1. 10초 standing 가능
2. 다점 contact 모델에서도 standing 가능
3. 약 `0.58 / 0.42` 수준의 좌우 하중 이동 전이 가능
4. `0.65 / 0.35` 이상은 정적 pose 후보가 있어도 standing에서 직접 진입 실패
5. `0.75 / 0.25`는 swing foot 직전 목표로는 의미가 있지만, 현재 trajectory/controller로는 진입 실패

중요한 해석:

- 실패 케이스에서도 torque saturation은 없다.
- 따라서 지금 한계는 모터 토크 부족으로 보기 어렵다.
- 더 큰 하중 이동을 만들려면 단순히 목표 pose만 찾는 것이 아니라, 중간 milestone을 여러 개 두는 trajectory 또는 lateral balance controller가 필요하다.

## 다음 조치

다음 단계는 `0.58 -> 0.65` 사이를 한 번에 넘기지 말고, 중간 milestone을 더 촘촘히 찾아야 한다.

추천 순서:

1. `left_force_ratio 0.58` 안정 pose를 시작점으로 사용한다.
2. 그 pose에서 `0.62`, `0.65`, `0.70` 목표 pose를 다시 탐색한다.
3. 각 milestone 사이를 짧은 ramp로 연결해본다.
4. 성공하면 최종적으로 `standing -> 0.58 -> 0.65 -> 0.75` trajectory를 만든다.
5. 이 trajectory가 성공해야 single support와 swing foot lift 테스트로 넘어갈 수 있다.

## 산출물

- `0.75` pose: `/home/king0519/projects/Humanoid/configs/weight_shift_left075_pose_multipoint.json`
- `0.65` pose: `/home/king0519/projects/Humanoid/configs/weight_shift_left065_pose_multipoint.json`
- `0.60` pose: `/home/king0519/projects/Humanoid/configs/weight_shift_left060_pose_multipoint.json`
- `0.60` 전이 영상: `/home/king0519/projects/Humanoid/outputs/analysis/render_pose_transition_left060_multipoint/pose_transition_render.mp4`
- `0.65` 전이 영상: `/home/king0519/projects/Humanoid/outputs/analysis/render_pose_transition_left065_multipoint/pose_transition_render.mp4`
- `0.75` 전이 영상: `/home/king0519/projects/Humanoid/outputs/analysis/render_pose_transition_left075_multipoint/pose_transition_render.mp4`
- `0.75` 느린 전이 영상: `/home/king0519/projects/Humanoid/outputs/analysis/render_pose_transition_left075_multipoint_slow/pose_transition_render.mp4`
