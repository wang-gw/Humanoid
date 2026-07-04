# Standing Geometry Analysis

## Purpose

Contact 단순화 후에도 neutral PD standing이 실패했기 때문에, 초기 자세의 전체 COM이 발 지지 영역 안에 있는지 확인한다.

## Command

```bash
python3 scripts/analyze_standing_geometry.py
```

## Summary

- Model: `/home/king0519/projects/Humanoid/envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_user_pad_axis_grounded.xml`
- Initial base z: `0.005000`
- Total mass: `9.211400 kg`
- COM world: `[0.05307600824175476, -0.04792230365714224, 0.28089664471442993]`
- Support AABB: `{'x_min': -0.147, 'x_max': 0.05550000000000001, 'y_min': -0.0565, 'y_max': 0.013500000000000005}`
- COM inside support x: `True`
- COM inside support y: `True`
- Support margins: `{'inside_x': True, 'inside_y': True, 'x_margin_min': 0.20007600824175475, 'x_margin_max': 0.0024239917582452447, 'y_margin_min': 0.008577696342857759, 'y_margin_max': 0.06142230365714225}`

## Foot Collision Geoms

- `foot_L_1_sole_collision` world_pos=`[0.03125000000000001, -0.0215, 0.024999999999999984]` halfsize=`[0.02425, 0.035, 0.02]`
- `foot_R_v1_1_sole_collision` world_pos=`[-0.12275, -0.0215, 0.024999999999999984]` halfsize=`[0.02425, 0.035, 0.02]`

## Interpretation

Neutral joint pose is not a verified standing pose. If the COM projection is outside the simplified foot support area, joint-space PD cannot reliably produce standing. The next step is to define or search for a standing pose that places both feet on the floor and the COM projection inside the support polygon.
