# Toe/Heel Z Contact 모델 시각화

## 목적

64번에서 확정한 `foot local Z = Toe/Heel` contact 모델을 시각적으로 확인한다. 수치만으로는 contact pad 위치, COM projection, 양발 지지 영역을 직관적으로 보기 어렵기 때문에, neutral pose와 quasi-static standing pose를 각각 그림으로 출력했다.

## 사용 모델

```text
envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_toeheel_z_grounded.xml
```

시각화 스크립트:

```bash
MUJOCO_GL=egl python3 scripts/visualize_contact_model.py \
  --model envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_toeheel_z_grounded.xml \
  --out-dir outputs/analysis/visual_user_pad_toeheel_z_neutral
```

```bash
MUJOCO_GL=egl python3 scripts/visualize_contact_model.py \
  --model envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_toeheel_z_grounded.xml \
  --pose-json configs/quasistatic_standing_pose_inertia_direct_nobase.json \
  --out-dir outputs/analysis/visual_user_pad_toeheel_z_quasistatic
```

## 그림 읽는 법

- 파란 사각형: 왼발 contact pad
- 주황 사각형: 오른발 contact pad
- 연한 녹색 영역: 양발 contact pad의 convex hull, 즉 단순화한 double-support 영역
- 빨간 X: 전체 COM의 지면 투영점
- MuJoCo X: 전후 방향
- MuJoCo Y: 좌우 방향
- MuJoCo Z: 수직 방향

## Neutral Pose 시각화

Top view:

![neutral top view](../../outputs/analysis/visual_user_pad_toeheel_z_neutral/contact_top_xy.png)

Side view:

![neutral side view](../../outputs/analysis/visual_user_pad_toeheel_z_neutral/contact_side_xz.png)

MuJoCo render:

![neutral render iso](../../outputs/analysis/visual_user_pad_toeheel_z_neutral/render_iso.png)

관찰:

- COM projection은 양발 contact pad를 잇는 double-support convex hull 내부에 있다.
- 하지만 COM이 한쪽 lateral 경계에 가깝다.
- 이 그림은 “기하학적으로 완전히 밖에 있는 상태는 아니다”를 보여준다.
- 다만 이 조건만으로 standing 동역학이 안정하다는 뜻은 아니다.

## Quasi-static Pose 시각화

Top view:

![quasistatic top view](../../outputs/analysis/visual_user_pad_toeheel_z_quasistatic/contact_top_xy.png)

Side view:

![quasistatic side view](../../outputs/analysis/visual_user_pad_toeheel_z_quasistatic/contact_side_xz.png)

MuJoCo render:

![quasistatic render iso](../../outputs/analysis/visual_user_pad_toeheel_z_quasistatic/render_iso.png)

관찰:

- COM projection은 double-support convex hull 내부에 있다.
- 하지만 COM이 두 발 사이의 빈 영역 쪽에 있고, 개별 foot pad 위에는 있지 않다.
- side view에서는 quasi-static pose에서 좌우 foot contact 높이와 자세가 완전히 같은 상태가 아님을 볼 수 있다.
- 이 상태에서 2초 PD standing이 실패했으므로, 문제는 단순히 COM이 support 영역 밖에 있어서만은 아니다.

## 현재 판단

시각화 결과는 64번 결론과 일치한다.

1. `foot local Z = Toe/Heel` contact pad 배치는 그림상 의도대로 전후 방향으로 들어갔다.
2. COM projection은 double-support 영역 내부에 있다.
3. 그럼에도 standing probe는 실패한다.

따라서 다음 문제는 contact pad 위치 확정보다, 다음 항목에 더 가깝다.

- quasi-static pose에서 실제 양발 접지 자세가 균일하지 않음
- ankle pitch/roll 토크 포화
- ankle roll/pitch joint axis 또는 actuator frame 민감도
- COM이 double-support 중앙에 충분히 가깝지 않음

## 산출물

Neutral pose:

- `outputs/analysis/visual_user_pad_toeheel_z_neutral/contact_top_xy.png`
- `outputs/analysis/visual_user_pad_toeheel_z_neutral/contact_side_xz.png`
- `outputs/analysis/visual_user_pad_toeheel_z_neutral/contact_rear_yz.png`
- `outputs/analysis/visual_user_pad_toeheel_z_neutral/render_iso.png`
- `outputs/analysis/visual_user_pad_toeheel_z_neutral/render_front.png`
- `outputs/analysis/visual_user_pad_toeheel_z_neutral/render_side.png`
- `outputs/analysis/visual_user_pad_toeheel_z_neutral/render_top.png`

Quasi-static pose:

- `outputs/analysis/visual_user_pad_toeheel_z_quasistatic/contact_top_xy.png`
- `outputs/analysis/visual_user_pad_toeheel_z_quasistatic/contact_side_xz.png`
- `outputs/analysis/visual_user_pad_toeheel_z_quasistatic/contact_rear_yz.png`
- `outputs/analysis/visual_user_pad_toeheel_z_quasistatic/render_iso.png`
- `outputs/analysis/visual_user_pad_toeheel_z_quasistatic/render_front.png`
- `outputs/analysis/visual_user_pad_toeheel_z_quasistatic/render_side.png`
- `outputs/analysis/visual_user_pad_toeheel_z_quasistatic/render_top.png`

