# Standing Geometry Analysis

## Purpose

Contact 단순화 후에도 neutral PD standing이 실패했기 때문에, 초기 자세의 전체 COM이 발 지지 영역 안에 있는지 확인한다.

## Command

```bash
python3 scripts/analyze_standing_geometry.py
```

## Summary

- Model: `/home/king0519/projects/Humanoid/envs/robots/urdf_f_link/URDF_F_link_user_mass_contact.xml`
- Initial base z: `0.042284`
- Total mass: `9.211400 kg`
- COM world: `[0.05335809715461275, -0.07376779677673535, 0.20549060171813704]`
- Support AABB: `{'x_min': -0.19925, 'x_max': 0.02525000000000001, 'y_min': -0.0215, 'y_max': 0.0985}`
- COM inside support x: `False`
- COM inside support y: `False`
- Support margins: `{'inside_x': False, 'inside_y': False, 'x_margin_min': 0.25260809715461274, 'x_margin_max': -0.028108097154612745, 'y_margin_min': -0.052267796776735355, 'y_margin_max': 0.17226779677673537}`

## Foot Collision Geoms

- `foot_L_1_sole_collision` world_pos=`[-0.009749999999999995, 0.0385, 0.0772841614740048]` halfsize=`[0.035, 0.06, 0.02]`
- `foot_R_v1_1_sole_collision` world_pos=`[-0.16425, 0.0385, 0.1272841614740048]` halfsize=`[0.035, 0.06, 0.02]`

## Interpretation

Neutral joint pose is not a verified standing pose. If the COM projection is outside the simplified foot support area, joint-space PD cannot reliably produce standing. The next step is to define or search for a standing pose that places both feet on the floor and the COM projection inside the support polygon.
