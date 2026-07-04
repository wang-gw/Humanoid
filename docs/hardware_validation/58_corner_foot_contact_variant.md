# Corner Foot Contact Variant

## 목적

단일 sole box가 roll 방향 접촉 지지 모멘트를 충분히 만들지 못하는지 확인하기 위해, 각 발의 네 모서리에 작은 contact pad를 둔 실험용 MJCF variant를 만든다.

## 실행 명령

```bash
python3 scripts/build_corner_foot_contact_variant.py
```

## 산출물

- 새 모델: `/home/king0519/projects/Humanoid/envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_corner_contact.xml`
- 리포트 JSON: `/home/king0519/projects/Humanoid/docs/hardware_validation/corner_foot_contact_variant_report.json`

## Contact Pad

- pad halfsize: `(0.015, 0.02, 0.006)`
- original sole bottom local z: `-0.04 m` 유지
- friction: `1.0 0.02 0.001`

| side | body | geom | local pos | halfsize |
| --- | --- | --- | --- | --- |
| `left` | `foot_L_1` | `foot_L_1_sole_pad_front_outer` | `0.02 0.035 -0.034` | `0.015 0.02 0.006` |
| `left` | `foot_L_1` | `foot_L_1_sole_pad_front_inner` | `0.02 -0.035 -0.034` | `0.015 0.02 0.006` |
| `left` | `foot_L_1` | `foot_L_1_sole_pad_rear_outer` | `-0.02 0.035 -0.034` | `0.015 0.02 0.006` |
| `left` | `foot_L_1` | `foot_L_1_sole_pad_rear_inner` | `-0.02 -0.035 -0.034` | `0.015 0.02 0.006` |
| `right` | `foot_R_v1_1` | `foot_R_v1_1_sole_pad_front_outer` | `0.02 0.035 -0.034` | `0.015 0.02 0.006` |
| `right` | `foot_R_v1_1` | `foot_R_v1_1_sole_pad_front_inner` | `0.02 -0.035 -0.034` | `0.015 0.02 0.006` |
| `right` | `foot_R_v1_1` | `foot_R_v1_1_sole_pad_rear_outer` | `-0.02 0.035 -0.034` | `0.015 0.02 0.006` |
| `right` | `foot_R_v1_1` | `foot_R_v1_1_sole_pad_rear_inner` | `-0.02 -0.035 -0.034` | `0.015 0.02 0.006` |

## Compile 결과

- nq: `17`
- nv: `16`
- nu: `10`
- ngeom: `32`

## 판단

이 모델은 최종 contact 모델이 아니라 roll 방향 원인 분리용 실험 모델이다. 같은 pose에서 roll impulse response를 비교해 contact 유지가 개선되는지 확인한다.
