# Standing Geometry Analysis

## Purpose

Contact 단순화 후에도 neutral PD standing이 실패했기 때문에, 초기 자세의 전체 COM이 발 지지 영역 안에 있는지 확인한다.

## Command

```bash
python3 scripts/analyze_standing_geometry.py
```

## Summary

- Model: `/home/king0519/projects/Humanoid/envs/robots/urdf_f_link/URDF_F_link_named.xml`
- Initial base z: `0.083571`
- Total mass: `8.623980 kg`
- COM world: `[0.05423189927388513, -0.0757338960364008, 0.24567026926200503]`
- Support AABB: `{'x_min': -0.013720452999999994, 'x_max': 0.052338097, 'y_min': -0.035, 'y_max': 0.035}`
- COM inside support x: `False`
- COM inside support y: `False`
- Support margins: `{'inside_x': False, 'inside_y': False, 'x_margin_min': 0.06795235227388513, 'x_margin_max': -0.0018938022738851298, 'y_margin_min': -0.04073389603640079, 'y_margin_max': 0.1107338960364008}`

## Foot Collision Geoms

- `foot_L_1_sole_collision` world_pos=`[0.019308824000000002, 0.0, 0.08203528195561294]` halfsize=`[0.033029273, 0.035, 0.060153984]`
- `foot_R_v1_1_sole_collision` world_pos=`[0.019308820000000004, 0.0, 0.08203528195561294]` halfsize=`[0.033029273, 0.035, 0.060153984]`

## Interpretation

Neutral joint pose is not a verified standing pose. If the COM projection is outside the simplified foot support area, joint-space PD cannot reliably produce standing. The next step is to define or search for a standing pose that places both feet on the floor and the COM projection inside the support polygon.
