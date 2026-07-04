# PD Standing 전체 로봇 시뮬레이션 시각화

## 목적

64번에서 확정한 `foot local Z = Toe/Heel` contact 모델이 실제 standing probe에서 어떻게 무너지는지 전체 로봇 렌더링으로 확인한다. 이전 문서들은 수치와 contact/COM 도면 중심이었기 때문에, 이번 문서는 동역학 시뮬레이션 중 로봇 자세 변화를 영상으로 남긴다.

## 사용 모델과 조건

모델:

```text
envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_toeheel_z_grounded.xml
```

Pose:

```text
configs/quasistatic_standing_pose_inertia_direct_nobase.json
```

제어 조건:

| 항목 | 값 |
| --- | ---: |
| duration | `2.0 s` |
| Kp | `60.0` |
| Kd | `4.0` |
| torque limit | `100.0 Nm` |
| fps | `30` |

실행 스크립트:

```bash
MUJOCO_GL=egl python3 scripts/render_pd_standing.py \
  --model envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_toeheel_z_grounded.xml \
  --pose-json configs/quasistatic_standing_pose_inertia_direct_nobase.json \
  --out-dir outputs/analysis/render_pd_standing_user_pad_toeheel_z_quasistatic \
  --duration 2.0 \
  --kp 60 \
  --kd 4 \
  --torque-limit 100 \
  --fps 30
```

## 시각화 파일

대각 view:

- MP4: `outputs/analysis/render_pd_standing_user_pad_toeheel_z_quasistatic/pd_standing_render.mp4`
- GIF: `outputs/analysis/render_pd_standing_user_pad_toeheel_z_quasistatic/pd_standing_render.gif`
- first frame: `outputs/analysis/render_pd_standing_user_pad_toeheel_z_quasistatic/first_frame.png`
- mid frame: `outputs/analysis/render_pd_standing_user_pad_toeheel_z_quasistatic/mid_frame.png`
- last frame: `outputs/analysis/render_pd_standing_user_pad_toeheel_z_quasistatic/last_frame.png`

정면 view:

- MP4: `outputs/analysis/render_pd_standing_user_pad_toeheel_z_front/pd_standing_render.mp4`
- GIF: `outputs/analysis/render_pd_standing_user_pad_toeheel_z_front/pd_standing_render.gif`

측면 view:

- MP4: `outputs/analysis/render_pd_standing_user_pad_toeheel_z_side/pd_standing_render.mp4`
- GIF: `outputs/analysis/render_pd_standing_user_pad_toeheel_z_side/pd_standing_render.gif`

## 대표 프레임

초기:

![first frame](../../outputs/analysis/render_pd_standing_user_pad_toeheel_z_quasistatic/first_frame.png)

중간:

![mid frame](../../outputs/analysis/render_pd_standing_user_pad_toeheel_z_quasistatic/mid_frame.png)

마지막:

![last frame](../../outputs/analysis/render_pd_standing_user_pad_toeheel_z_quasistatic/last_frame.png)

## 영상에서 확인되는 내용

초기 `t=0.000 s`:

- contacts: `4`
- contact force: 약 `95.6 N`
- roll/pitch: `0`
- actuator torque는 아직 거의 없음

중간 `t≈1.024 s`:

- contacts: `0`
- contact force: `0 N`
- roll: 약 `+2.68 rad`
- max torque: `100 Nm`

마지막 `t=2.000 s`:

- contacts: `1`
- contact force: 약 `1663 N`
- roll: 약 `+2.82 rad`
- pitch: 약 `-0.23 rad`
- max torque: `100 Nm`

## 판단

전체 로봇 영상으로 보면, standing 실패는 매우 이른 시간에 시작된다. 약 1초 시점에는 이미 양발 접촉이 끊기고 roll 방향으로 크게 넘어진다.

따라서 현재 실패를 다음처럼 해석한다.

1. Contact pad가 없어서 생긴 단순 표시 문제가 아니다.
2. 양발 support 영역은 존재하지만, 현재 pose와 PD 제어만으로는 접촉을 유지하지 못한다.
3. ankle pitch/roll actuator가 `100 Nm` 한계에 닿은 상태에서도 자세를 복구하지 못한다.
4. 다음 단계는 contact 모델 수정이 아니라, 확정 contact 모델에서 pose 재탐색과 ankle joint/actuator 검증을 해야 한다.

## 산출물

- 렌더 스크립트: `scripts/render_pd_standing.py`
- 대각 view summary: `outputs/analysis/render_pd_standing_user_pad_toeheel_z_quasistatic/render_summary.json`
- 정면 view summary: `outputs/analysis/render_pd_standing_user_pad_toeheel_z_front/render_summary.json`
- 측면 view summary: `outputs/analysis/render_pd_standing_user_pad_toeheel_z_side/render_summary.json`

