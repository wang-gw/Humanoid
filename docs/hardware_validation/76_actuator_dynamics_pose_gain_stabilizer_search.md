# Actuator Dynamics 기준 Pose/Gain/Stabilizer 재탐색

## 목적

75번에서 actuator dynamics 변형 모델이 standing 실패 양상을 크게 완화하는 것을 확인했다.

이번 단계에서는 해당 모델을 기준으로 다음을 다시 수행했다.

1. 준정적 standing pose 재탐색
2. contact 시작용 base_z 보정
3. PD gain sweep
4. axis-aware stabilizer sweep
5. best 후보 영상화

## 기준 모델

```text
envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_toeheel_z_grounded_step_visual_actuator_dynamics.xml
```

이 모델은 원본 STEP visual/contact 모델에 다음 실험값을 추가한 변형이다.

| joint 계열 | armature | damping | frictionloss |
| --- | ---: | ---: | ---: |
| hip | `0.02` | `0.2` | `0.02` |
| knee | `0.02` | `0.2` | `0.02` |
| ankle | `0.05` | `0.4` | `0.02` |

## 1. Standing pose 재탐색

실행:

```bash
python3 scripts/search_quasistatic_standing_pose.py --model envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_toeheel_z_grounded_step_visual_actuator_dynamics.xml --samples 80000 --iterations 10 --elite 384 --seed 17 --limit 0.8 --out configs/quasistatic_standing_pose_actuator_dynamics.json
```

결과:

| 항목 | 값 |
| --- | ---: |
| cost | `0.055434` |
| search base_z | `0.006691 m` |
| joint norm | `0.425493` |
| COM x/y/z | `0.065238 / -0.051814 / 0.291101 m` |
| min support margin | `0.038385 m` |
| foot low z | `0.005274 / 0.005000 m` |
| initial contacts | `0` |

이 pose는 발바닥이 약 `5 mm` 떠 있는 clearance 기준이므로, 그대로 dynamics에 넣으면 contact 없이 시작한다.

## 2. Contact pose 생성

search pose에서 base_z만 `0.006 m` 낮춘 contact pose를 새로 만들었다.

```text
configs/quasistatic_standing_pose_actuator_dynamics_contact.json
```

보정:

| 항목 | 값 |
| --- | ---: |
| search base_z | `0.006691 m` |
| contact base_z | `0.000691 m` |
| base_z delta | `-0.006000 m` |
| lowest contact z | `-0.001000 m` |
| initial contacts | `3` |
| min support margin | `0.038385 m` |

## 3. PD gain sweep

실행:

```bash
python3 scripts/sweep_pd_standing_gains.py --model envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_toeheel_z_grounded_step_visual_actuator_dynamics.xml --pose-json configs/quasistatic_standing_pose_actuator_dynamics_contact.json --out-dir outputs/analysis/pd_gain_sweep_actuator_dynamics_contact_pose --duration 2.0 --kps 5,10,20,40,60,80 --kds 0.5,1,2,4,8,12 --torque-limits 30,60,100
```

총 `108`개 조합을 테스트했다.

Best PD:

| 항목 | 값 |
| --- | ---: |
| Kp | `20` |
| Kd | `12` |
| torque limit | `30 Nm` |
| final base_z | `0.047309 m` |
| final roll | `0.558019 rad` |
| final pitch | `-0.054077 rad` |
| max qvel norm | `0.695749` |
| max contact force | `92.715245 N` |
| max torque | `3.167586 Nm` |
| contact fraction | `1.0` |
| saturation fraction | `0.0` |

판단:

- 원본 모델 대비 매우 안정적이다.
- contact가 유지되고 torque saturation도 없다.
- 하지만 roll이 `0.56 rad`까지 누적되므로 아직 standing 통과는 아니다.

영상:

```text
outputs/analysis/render_pd_standing_actuator_dynamics_contact_pose_best_gain/pd_standing_render.mp4
```

## 4. Axis-aware stabilizer sweep

실행:

```bash
python3 scripts/sweep_axis_aware_standing.py --model envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_toeheel_z_grounded_step_visual_actuator_dynamics.xml --pose-json configs/quasistatic_standing_pose_actuator_dynamics_contact.json --out-dir outputs/analysis/axis_aware_sweep_actuator_dynamics_contact_pose --duration 2.0 --joint-kp 20 --joint-kd 12 --torque-limit 30 --kp-att 0,0.5,1,2,4 --kd-att 0,0.5,1,2,4 --kcom 0,0.5,1,2
```

총 `400`개 조합을 테스트했다.

Best axis-aware:

| 항목 | 값 |
| --- | ---: |
| joint Kp | `20` |
| joint Kd | `12` |
| torque limit | `30 Nm` |
| attitude Kp | `0` |
| attitude Kd | `1` |
| COM K | `1` |
| roll sign | `-1` |
| pitch sign | `-1` |
| final base_z | `0.024537 m` |
| final roll | `0.397846 rad` |
| final pitch | `-0.032835 rad` |
| max qvel norm | `0.651344` |
| max contact force | `92.969972 N` |
| max torque | `3.298711 Nm` |
| contact fraction | `1.0` |
| saturation fraction | `0.0` |

PD-only 대비:

| 항목 | PD-only best | axis-aware best |
| --- | ---: | ---: |
| final roll | `0.558019 rad` | `0.397846 rad` |
| final pitch | `-0.054077 rad` | `-0.032835 rad` |
| max qvel norm | `0.695749` | `0.651344` |
| max torque | `3.167586 Nm` | `3.298711 Nm` |
| contact fraction | `1.0` | `1.0` |
| saturation fraction | `0.0` | `0.0` |

axis-aware stabilizer가 roll drift를 줄였지만, 완전히 제거하지는 못했다.

영상:

```text
outputs/analysis/render_axis_aware_standing_actuator_dynamics_contact_pose_best/axis_aware_standing_render.mp4
```

프레임 관찰:

- `0.0 s`: contacts `3`, roll/pitch 거의 `0`
- `1.024 s`: contacts `3`, roll `+0.33 rad`, pitch `-0.01 rad`
- `2.0 s`: contacts `3`, roll `+0.40 rad`, pitch `-0.03 rad`

## 현재 판단

이번 단계는 지금까지의 standing 검증 중 가장 좋은 결과다.

좋아진 점:

1. 접촉을 2초 동안 유지한다.
2. torque saturation이 없다.
3. qvel이 낮다.
4. contact force가 안정적이다.
5. axis-aware 보정으로 roll drift가 감소했다.

남은 문제:

1. roll이 계속 누적된다.
2. 2초 후에도 직립 안정 상태라고 보기 어렵다.
3. attitude Kp가 큰 조합보다 `Kp=0, Kd/COM 중심`이 더 좋았다는 점은 pose/contact/axis 기하가 아직 완전히 맞지 않을 수 있음을 시사한다.

## 결론

현재 로봇 형상은 "강화학습을 시도해볼 가치가 전혀 없는 상태"는 아니다. actuator dynamics를 넣으면 동역학이 훨씬 현실적으로 안정된다.

하지만 아직 "RL만 돌리면 충분히 걸을 가능성이 높다"고 말하기에는 이르다. RL 전에 최소한 2초 standing에서 roll drift를 더 줄여야 한다.

다음 우선순위는 다음 둘 중 하나다.

1. actuator dynamics 모델 기준으로 contact geometry를 더 안정적인 다점 foot contact로 개선한다.
2. 현재 모델을 유지하고 roll drift를 줄이는 standing pose를 더 직접적으로 최적화한다.

현재 결과를 보면 contact는 유지되므로, 다음 단계는 `roll drift`를 비용함수에 직접 넣은 standing pose search가 더 적절하다.

## 산출물

- contact pose: `/home/king0519/projects/Humanoid/configs/quasistatic_standing_pose_actuator_dynamics_contact.json`
- PD sweep summary: `/home/king0519/projects/Humanoid/outputs/analysis/pd_gain_sweep_actuator_dynamics_contact_pose/pd_gain_sweep_summary.json`
- axis-aware sweep summary: `/home/king0519/projects/Humanoid/outputs/analysis/axis_aware_sweep_actuator_dynamics_contact_pose/axis_aware_sweep_summary.json`
- PD best video: `/home/king0519/projects/Humanoid/outputs/analysis/render_pd_standing_actuator_dynamics_contact_pose_best_gain/pd_standing_render.mp4`
- axis-aware best video: `/home/king0519/projects/Humanoid/outputs/analysis/render_axis_aware_standing_actuator_dynamics_contact_pose_best/axis_aware_standing_render.mp4`
