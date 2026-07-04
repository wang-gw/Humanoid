# Standing Geometry Analysis

## Purpose

Contact 단순화 후에도 neutral PD standing이 실패했기 때문에, 초기 자세의 전체 COM이 발 지지 영역 안에 있는지 확인한다.

## Command

```bash
python3 scripts/analyze_standing_geometry.py
```

## Summary

- Model: `/home/king0519/projects/Humanoid/envs/robots/urdf_f_link/URDF_F_link_user_size_mass_inertia_cad_assembly_edge_contact.xml`
- Initial base z: `0.005000`
- Total mass: `9.211400 kg`
- COM world: `[0.05307600824175476, -0.04792230365714224, 0.28089664471442993]`
- Support AABB: `{'x_min': -0.126, 'x_max': -0.11399999999999999, 'y_min': -0.11225, 'y_max': 0.11175000000000002}`
- COM inside support x: `False`
- COM inside support y: `True`
- Support margins: `{'inside_x': False, 'inside_y': True, 'x_margin_min': 0.17907600824175476, 'x_margin_max': -0.16707600824175475, 'y_margin_min': 0.06432769634285776, 'y_margin_max': 0.15967230365714224}`

## Foot Collision Geoms

- `foot_L_1_sole_collision` world_pos=`[-0.12, 0.07675000000000001, 0.007999999999999986]` halfsize=`[0.006, 0.035, 0.003]`
- `foot_R_v1_1_sole_collision` world_pos=`[-0.12, -0.07725, 0.007999999999999986]` halfsize=`[0.006, 0.035, 0.003]`

## Interpretation

Neutral joint pose is not a verified standing pose. If the COM projection is outside the simplified foot support area, joint-space PD cannot reliably produce standing. The next step is to define or search for a standing pose that places both feet on the floor and the COM projection inside the support polygon.
