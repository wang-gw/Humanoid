# Standing Geometry 분석

## 목적

Contact 단순화 후에도 neutral PD standing이 실패했기 때문에, 초기 자세의 전체 COM이 발 지지 영역 안에 있는지 확인한다.

## 실행 명령

```bash
python3 scripts/analyze_standing_geometry.py
```

## 요약

- 모델: `/home/king0519/projects/Humanoid/envs/robots/urdf_f/URDF_F_contact.xml`
- 초기 base z: `0.083571`
- 총 질량: `8.623980 kg`
- COM world: `[0.05423189927388513, -0.0757338960364008, 0.24567026926200503]`
- Support AABB: `{'x_min': -0.013720452999999994, 'x_max': 0.052338097, 'y_min': -0.035, 'y_max': 0.035}`
- COM이 support x 범위 안에 있는가: `False`
- COM이 support y 범위 안에 있는가: `False`
- support margin: `{'inside_x': False, 'inside_y': False, 'x_margin_min': 0.06795235227388513, 'x_margin_max': -0.0018938022738851298, 'y_margin_min': -0.04073389603640079, 'y_margin_max': 0.1107338960364008}`

## Foot Collision Geom

- `foot_L_1_sole_collision` world_pos=`[0.019308824000000002, 0.0, 0.08203528195561294]` halfsize=`[0.033029273, 0.035, 0.060153984]`
- `foot_R_v1_1_sole_collision` world_pos=`[0.019308820000000004, 0.0, 0.08203528195561294]` halfsize=`[0.033029273, 0.035, 0.060153984]`

## 해석

neutral joint pose는 검증된 standing pose가 아니다. COM projection이 단순화한 foot support 영역 밖에 있으면 joint-space PD만으로 안정적인 standing을 만들기 어렵다. 다음 단계는 양발이 바닥에 닿고 COM projection이 support polygon 안에 들어오는 standing pose를 정의하거나 탐색하는 것이다.
