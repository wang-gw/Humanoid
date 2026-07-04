# Standing Geometry Analysis

## Purpose

Contact 단순화 후에도 neutral PD standing이 실패했기 때문에, 초기 자세의 전체 COM이 발 지지 영역 안에 있는지 확인한다.

## Command

```bash
python3 scripts/analyze_standing_geometry.py
```

## Summary

- Model: `/home/king0519/projects/Humanoid/envs/robots/urdf_f_link/URDF_F_link_step_contact.xml`
- Initial base z: `0.069642`
- Total mass: `8.623980 kg`
- COM world: `[0.05423189927388513, -0.0757338960364008, 0.23174125769231277]`
- Support AABB: `{'x_min': -0.122, 'x_max': 0.10200000000000001, 'y_min': -0.08149999999999999, 'y_max': 0.0385}`
- COM inside support x: `True`
- COM inside support y: `True`
- Support margins: `{'inside_x': True, 'inside_y': True, 'x_margin_min': 0.17623189927388513, 'x_margin_max': 0.04776810072611488, 'y_margin_min': 0.005766103963599195, 'y_margin_max': 0.1142338960364008}`

## Foot Collision Geoms

- `foot_L_1_sole_collision` world_pos=`[0.067, -0.0215, 0.07464194138592065]` halfsize=`[0.035, 0.06, 0.005]`
- `foot_R_v1_1_sole_collision` world_pos=`[-0.087, -0.0215, 0.07464194138592065]` halfsize=`[0.035, 0.06, 0.005]`

## Interpretation

Neutral joint pose is not a verified standing pose. If the COM projection is outside the simplified foot support area, joint-space PD cannot reliably produce standing. The next step is to define or search for a standing pose that places both feet on the floor and the COM projection inside the support polygon.
