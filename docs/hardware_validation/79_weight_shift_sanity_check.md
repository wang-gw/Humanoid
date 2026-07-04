# Weight Shift Sanity Check

## 목적

78번에서 10초 standing sanity check를 통과했으므로, 다음으로 좌우 하중 이동이 가능한지 확인한다.

보행으로 넘어가려면 단순히 서는 것만으로는 부족하다. 한쪽 발에서 다른 쪽 발로 하중을 옮길 수 있어야 swing foot을 만들 수 있다.

## 기준 모델과 pose

- 모델: `/home/king0519/projects/Humanoid/envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_toeheel_z_grounded_step_visual_actuator_dynamics.xml`
- pose: `/home/king0519/projects/Humanoid/configs/dynamic_standing_pose_actuator_dynamics_contact_5s.json`
- 스크립트: `/home/king0519/projects/Humanoid/scripts/render_weight_shift.py`

기준 standing controller:

| 항목 | 값 |
| --- | ---: |
| joint Kp | `20` |
| joint Kd | `12` |
| torque limit | `30 Nm` |
| attitude Kp | `0` |
| attitude Kd | `1` |
| COM K | `1` |
| roll sign | `-1` |
| pitch sign | `+1` |

## 방법

standing stabilizer의 lateral COM error에 시간에 따라 변하는 목표값을 추가했다.

```text
target_lateral = amplitude * sin(2*pi*t/period)
```

이 테스트는 실제 보행 제어기가 아니라, 현재 하드웨어/동역학 모델이 좌우 하중 이동 명령을 안정적으로 받아들이는지 보는 sanity check다.

## Case 1: 작은 weight shift

실행:

```bash
MUJOCO_GL=egl python3 scripts/render_weight_shift.py --model envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_toeheel_z_grounded_step_visual_actuator_dynamics.xml --pose-json configs/dynamic_standing_pose_actuator_dynamics_contact_5s.json --out-dir outputs/analysis/render_weight_shift_actuator_dynamics_amp018 --duration 10.0 --joint-kp 20 --joint-kd 12 --torque-limit 30 --kp-att 0 --kd-att 1 --kcom 1 --roll-sign -1 --pitch-sign 1 --lateral-amplitude 0.18 --period 6 --ramp 1
```

결과:

| 항목 | 값 |
| --- | ---: |
| lateral amplitude | `0.18` |
| Kcom | `1` |
| contact frame fraction | `1.0` |
| saturation fraction | `0.0` |
| max qvel norm | `0.587915` |
| max contact force | `105.095404 N` |
| max torque | `3.764081 Nm` |
| left force ratio min/max | `0.277838 / 1.000000` |
| left force ratio range | `0.722162` |
| final roll | `0.004698 rad` |
| final pitch | `-0.047473 rad` |

초기 `1 s` 이후만 보면:

| 항목 | 값 |
| --- | ---: |
| left force ratio min/max | `0.502480 / 0.559585` |
| left force ratio range | `0.057105` |
| max abs roll | `0.025683 rad` |
| max abs pitch | `0.047473 rad` |
| max qvel norm | `0.132330` |

해석:

- 안정적으로 버틴다.
- 접촉과 torque saturation은 문제 없다.
- 하지만 초기 접촉 전이를 제외하면 좌우 하중 이동 폭은 작다.
- 즉 "작은 lateral command를 견딘다"는 의미는 있지만, swing foot을 만들 만큼 충분한 하중 이동이라고 보기는 어렵다.

영상:

```text
outputs/analysis/render_weight_shift_actuator_dynamics_amp018/weight_shift_render.mp4
```

## Case 2: 중간 weight shift

조건:

| 항목 | 값 |
| --- | ---: |
| lateral amplitude | `0.25` |
| Kcom | `2` |
| roll sign | `-1` |
| pitch sign | `+1` |

결과:

| 항목 | 값 |
| --- | ---: |
| contact frame fraction | `0.996815` |
| saturation fraction | `0.0` |
| max qvel norm | `7.408567` |
| max contact force | `314.781785 N` |
| max torque | `15.419325 Nm` |
| final roll | `2.460495 rad` |
| final pitch | `-0.075802 rad` |

판단:

- 실패다.
- 접촉은 대부분 유지되지만, roll이 크게 누적되어 몸체가 사실상 넘어진다.

영상:

```text
outputs/analysis/render_weight_shift_actuator_dynamics_amp025_kcom2/weight_shift_render.mp4
```

## Case 3: 강한 weight shift

조건:

| 항목 | 값 |
| --- | ---: |
| lateral amplitude | `0.35` |
| Kcom | `4` |
| roll sign | `-1` |
| pitch sign | `+1` |

결과:

| 항목 | 값 |
| --- | ---: |
| contact frame fraction | `1.0` |
| saturation fraction | `0.0` |
| max qvel norm | `8.964827` |
| max contact force | `374.156183 N` |
| max torque | `20.760775 Nm` |
| final roll | `2.659332 rad` |
| final pitch | `0.035434 rad` |

판단:

- 실패다.
- torque saturation은 없지만, 큰 roll 회전이 발생한다.

영상:

```text
outputs/analysis/render_weight_shift_actuator_dynamics_amp035_kcom4/weight_shift_render.mp4
```

## Case 4: roll sign 반대 조건

조건:

| 항목 | 값 |
| --- | ---: |
| lateral amplitude | `0.25` |
| Kcom | `1` |
| roll sign | `+1` |
| pitch sign | `+1` |

결과:

| 항목 | 값 |
| --- | ---: |
| contact frame fraction | `1.0` |
| saturation fraction | `0.0` |
| max qvel norm | `7.814320` |
| max contact force | `335.007816 N` |
| max torque | `17.215879 Nm` |
| final roll | `-3.106917 rad` |
| final pitch | `0.276758 rad` |

판단:

- 실패다.
- roll sign을 반대로 둬도 중간 weight shift를 안정적으로 수행하지 못한다.

영상:

```text
outputs/analysis/render_weight_shift_actuator_dynamics_amp025_kcom1_rollpos/weight_shift_render.mp4
```

## 현재 판단

standing은 가능해졌지만, weight shift는 아직 충분하지 않다.

확인된 점:

1. 작은 lateral command는 안정적으로 견딘다.
2. torque saturation 없이 접촉을 유지할 수 있다.
3. 하지만 안정 조건에서 실제 좌우 하중 이동 폭은 작다.
4. 더 큰 하중 이동 명령은 roll 방향 붕괴로 이어진다.

따라서 현재 상태에서 바로 stepping/RL walking으로 넘어가면 학습이 어렵거나, 비현실적인 보상/제어로만 해결될 가능성이 있다.

## 결론

현재 하드웨어/모델은 standing sanity check는 통과했지만, weight shift sanity check는 부분 통과다.

다음 단계에서는 단순 COM target stabilizer가 아니라, 다음 중 하나를 적용해야 한다.

1. 좌우 발 접촉을 더 안정적으로 만드는 다점 foot contact 모델
2. weight shift 전용 pose trajectory
3. hip/ankle roll role에 맞춘 더 명확한 lateral balance controller
4. 실제 actuator dynamics 값 반영 후 재검증

검증 순서상 다음은 `다점 foot contact` 또는 `weight shift trajectory search`가 적절하다. 지금 결과만 보면 접촉력이 충분히 이동하지 못하므로, 먼저 foot contact patch를 개선하는 쪽이 더 직접적이다.

## 산출물

- 작은 weight shift summary: `/home/king0519/projects/Humanoid/outputs/analysis/render_weight_shift_actuator_dynamics_amp018/weight_shift_summary.json`
- 작은 weight shift video: `/home/king0519/projects/Humanoid/outputs/analysis/render_weight_shift_actuator_dynamics_amp018/weight_shift_render.mp4`
- 중간 weight shift video: `/home/king0519/projects/Humanoid/outputs/analysis/render_weight_shift_actuator_dynamics_amp025_kcom2/weight_shift_render.mp4`
- 강한 weight shift video: `/home/king0519/projects/Humanoid/outputs/analysis/render_weight_shift_actuator_dynamics_amp035_kcom4/weight_shift_render.mp4`
- roll sign 반대 video: `/home/king0519/projects/Humanoid/outputs/analysis/render_weight_shift_actuator_dynamics_amp025_kcom1_rollpos/weight_shift_render.mp4`
