# Standing Geometry Analysis

## Purpose

Contact 단순화 후에도 neutral PD standing이 실패했기 때문에, 초기 자세의 전체 COM이 발 지지 영역 안에 있는지 확인한다.

## Command

```bash
python3 scripts/analyze_standing_geometry.py
```

## Summary

- Model: `/home/king0519/projects/Humanoid/envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_mjcf_grounded.xml`
- Initial base z: `0.005000`
- Total mass: `9.211400 kg`
- COM world: `[0.05307600824175476, -0.04792230365714224, 0.28089664471442993]`
- Support AABB: `{'x_min': -0.122, 'x_max': 0.10200000000000001, 'y_min': -0.08149999999999999, 'y_max': -0.032999999999999995}`
- COM inside support x: `True`
- COM inside support y: `True`
- Support margins: `{'inside_x': True, 'inside_y': True, 'x_margin_min': 0.17507600824175476, 'x_margin_max': 0.048923991758245244, 'y_margin_min': 0.033577696342857746, 'y_margin_max': 0.014922303657142248}`

## Foot Collision Geoms

- `foot_L_1_sole_collision` world_pos=`[0.067, -0.057249999999999995, 0.024999999999999984]` halfsize=`[0.035, 0.02425, 0.02]`
- `foot_R_v1_1_sole_collision` world_pos=`[-0.087, -0.057249999999999995, 0.024999999999999984]` halfsize=`[0.035, 0.02425, 0.02]`

## Interpretation

Neutral joint pose is not a verified standing pose. If the COM projection is outside the simplified foot support area, joint-space PD cannot reliably produce standing. The next step is to define or search for a standing pose that places both feet on the floor and the COM projection inside the support polygon.
