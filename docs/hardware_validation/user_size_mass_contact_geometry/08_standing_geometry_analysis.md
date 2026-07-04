# Standing Geometry Analysis

## Purpose

Contact 단순화 후에도 neutral PD standing이 실패했기 때문에, 초기 자세의 전체 COM이 발 지지 영역 안에 있는지 확인한다.

## Command

```bash
python3 scripts/analyze_standing_geometry.py
```

## Summary

- Model: `/home/king0519/projects/Humanoid/envs/robots/urdf_f_link/URDF_F_link_user_size_mass_contact.xml`
- Initial base z: `0.057284`
- Total mass: `9.211400 kg`
- COM world: `[0.05335809715461275, -0.07376779677673535, 0.22049060171813703]`
- Support AABB: `{'x_min': -0.122, 'x_max': 0.10200000000000001, 'y_min': -0.08149999999999999, 'y_max': 0.0385}`
- COM inside support x: `True`
- COM inside support y: `True`
- Support margins: `{'inside_x': True, 'inside_y': True, 'x_margin_min': 0.17535809715461276, 'x_margin_max': 0.048641902845387254, 'y_margin_min': 0.007732203223264636, 'y_margin_max': 0.11226779677673535}`

## Foot Collision Geoms

- `foot_L_1_sole_collision` world_pos=`[0.067, -0.0215, 0.07728416147400481]` halfsize=`[0.035, 0.06, 0.02]`
- `foot_R_v1_1_sole_collision` world_pos=`[-0.087, -0.0215, 0.07728416147400481]` halfsize=`[0.035, 0.06, 0.02]`

## Interpretation

Neutral joint pose is not a verified standing pose. If the COM projection is outside the simplified foot support area, joint-space PD cannot reliably produce standing. The next step is to define or search for a standing pose that places both feet on the floor and the COM projection inside the support polygon.
