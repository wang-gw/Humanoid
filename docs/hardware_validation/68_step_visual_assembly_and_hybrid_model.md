# STEP Visual Assembly 및 Hybrid Visual 모델

## 목적

67번에서 확인한 visual mesh overlap 문제를 더 분리한다.

질문은 두 가지였다.

1. STL 파일 자체가 잘못되어 형체가 안 보이는가?
2. 아니면 동역학 MJCF 안에서 STL을 각 body에 붙이는 local offset이 잘못되었는가?

이번 단계에서는 STEP assembly transform만 사용한 시각 전용 모델과, 기존 동역학 모델의 visual offset만 STEP 기준으로 보정한 hybrid 모델을 만들었다.

## STEP Visual Assembly

생성 명령:

```bash
python3 scripts/build_step_visual_assembly.py \
  --step URDF_F_.step \
  --link-dir link \
  --out-dir envs/robots/urdf_f_step_visual \
  --out-name URDF_F_step_visual_assembly.xml
```

생성 모델:

```text
envs/robots/urdf_f_step_visual/URDF_F_step_visual_assembly.xml
```

이 모델은 physics/RL용이 아니다.

- joint 없음
- actuator 없음
- collision 검증용 아님
- STEP occurrence transform에 STL을 배치한 순수 시각 확인용 모델

렌더 결과:

![STEP visual iso](../../outputs/analysis/step_visual_assembly_render/render_iso.png)

![STEP visual front](../../outputs/analysis/step_visual_assembly_render/render_front.png)

판단:

- STEP assembly 기준으로 STL을 배치하면 로봇 형체가 정상적으로 보인다.
- 따라서 `link/` STL 파일 자체가 전부 한 점에 겹친 잘못된 파일은 아니다.
- 문제는 기존 동역학 MJCF에서 visual mesh를 각 body frame에 붙이는 offset/rotation 쪽에 있다.

## Hybrid Visual 모델

다음으로 기존 동역학 모델은 그대로 두고, visual mesh geom의 local `pos/quat`만 STEP assembly world pose 기준으로 다시 계산했다.

입력 모델:

```text
envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_toeheel_z_grounded.xml
```

생성 명령:

```bash
python3 scripts/apply_step_visual_offsets.py \
  --source envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_toeheel_z_grounded.xml \
  --step URDF_F_.step \
  --out envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_toeheel_z_grounded_step_visual.xml
```

생성 모델:

```text
envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_toeheel_z_grounded_step_visual.xml
```

변환 방식:

```text
target world pose = STEP occurrence pose
body world pose   = 현재 MJCF neutral pose의 body pose

geom local pos = R_body^T * (target_pos - body_pos)
geom local rot = R_body^T * target_rot
```

즉 physics body, joint, inertia, actuator, contact pad는 유지하고 visual mesh만 보기 좋게 정렬했다.

## Hybrid 모델 standing render

실행 명령:

```bash
MUJOCO_GL=egl python3 scripts/render_pd_standing.py \
  --model envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_toeheel_z_grounded_step_visual.xml \
  --pose-json configs/quasistatic_standing_pose_inertia_direct_nobase.json \
  --out-dir outputs/analysis/render_pd_standing_user_pad_toeheel_z_step_visual_wide \
  --duration 2.0 \
  --kp 60 \
  --kd 4 \
  --torque-limit 100 \
  --fps 30 \
  --azimuth 135 \
  --elevation -12 \
  --distance 1.45
```

대표 프레임:

![hybrid first](../../outputs/analysis/render_pd_standing_user_pad_toeheel_z_step_visual_wide/first_frame.png)

![hybrid mid](../../outputs/analysis/render_pd_standing_user_pad_toeheel_z_step_visual_wide/mid_frame.png)

![hybrid last](../../outputs/analysis/render_pd_standing_user_pad_toeheel_z_step_visual_wide/last_frame.png)

영상:

- MP4: `outputs/analysis/render_pd_standing_user_pad_toeheel_z_step_visual_wide/pd_standing_render.mp4`
- GIF: `outputs/analysis/render_pd_standing_user_pad_toeheel_z_step_visual_wide/pd_standing_render.gif`

## 결과 해석

초기 프레임에서는 기존 66번 영상보다 로봇 다리 형체가 훨씬 잘 보인다. 즉 visual overlap 문제는 어느 정도 해결되었다.

하지만 동역학 결과는 바뀌지 않았다.

| 항목 | 값 |
| --- | ---: |
| final time | `2.0 s` |
| final roll | `2.815632 rad` |
| final pitch | `-0.230678 rad` |
| final contact count | `1` |
| final contact force | `1662.986 N` |
| max qvel norm | `188.108` |
| max contact force | `2624.473 N` |
| max torque | `100 Nm` |

이는 의도한 결과다. 이번 모델은 visual만 바꿨고 physics/contact는 바꾸지 않았기 때문이다.

## 현재 결론

이제 문제를 다음처럼 분리할 수 있다.

1. `link/` STL은 STEP assembly 기준으로 배치하면 정상적인 로봇 외형을 만든다.
2. 기존 동역학 MJCF의 visual mesh local offset은 잘못되어 있었다.
3. STEP visual offset 보정 모델은 렌더 확인용으로 더 낫다.
4. 하지만 standing 실패는 visual 문제가 아니라 physics/joint/contact/pose 문제다.

앞으로 전체 로봇 렌더를 볼 때는 다음 모델을 사용하는 것이 낫다.

```text
envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_toeheel_z_grounded_step_visual.xml
```

단, 이 모델도 최종 CAD-to-MJCF 변환 완료본은 아니다. STEP neutral pose 기준으로 visual을 맞춘 hybrid 모델이므로, 큰 joint motion에서 모든 visual mesh가 완벽하게 실제 CAD motion을 따른다고 보장할 수는 없다. 그래도 기존처럼 actuator mesh가 한 점에 겹쳐 보이는 문제는 크게 줄었다.

## 다음 조치

이제 visual 확인에는 hybrid 모델을 쓰고, 동역학 검증은 계속 같은 physics 조건으로 진행한다.

다음 우선순위는 다음 중 하나다.

1. 확정 contact + hybrid visual 모델 기준으로 standing pose를 다시 탐색한다.
2. ankle pitch/roll joint axis와 actuator sign을 더 정밀하게 검증한다.
3. CAD에서 link-local frame 기준 STL export를 다시 받아 최종 visual/collision 모델을 만든다.

## 산출물

- STEP visual 생성 스크립트: `scripts/build_step_visual_assembly.py`
- hybrid visual offset 스크립트: `scripts/apply_step_visual_offsets.py`
- STEP visual 모델: `envs/robots/urdf_f_step_visual/URDF_F_step_visual_assembly.xml`
- hybrid visual 모델: `envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_toeheel_z_grounded_step_visual.xml`
- STEP visual report: `docs/hardware_validation/step_visual_assembly_report.json`
- hybrid visual report: `docs/hardware_validation/step_visual_offsets_report.json`
- STEP visual render: `outputs/analysis/step_visual_assembly_render/`
- hybrid standing render: `outputs/analysis/render_pd_standing_user_pad_toeheel_z_step_visual_wide/`

