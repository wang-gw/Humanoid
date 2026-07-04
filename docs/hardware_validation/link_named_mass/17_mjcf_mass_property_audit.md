# MJCF 질량/관성 감사

## 목적

standing 실패 원인 중 하나가 잘못된 질량/관성 또는 좌우 비대칭일 수 있으므로, 현재 named MJCF의 body mass와 inertia를 정리한다.

이 리포트는 CAD mass property와 비교하기 위한 기준 자료다.

## 실행 명령

```bash
python3 scripts/audit_mjcf_mass_properties.py
```

## 요약

- 모델: `/home/king0519/projects/Humanoid/envs/robots/urdf_f_link/URDF_F_link_named.xml`
- 총 질량: `8.623980 kg`
- body 수: `12`

## 산출물

- body mass CSV: `/home/king0519/projects/Humanoid/outputs/analysis/mass_properties_link/mjcf_body_mass_properties.csv`
- 좌우 mass pair CSV: `/home/king0519/projects/Humanoid/outputs/analysis/mass_properties_link/mjcf_left_right_mass_pairs.csv`
- JSON: `/home/king0519/projects/Humanoid/outputs/analysis/mass_properties_link/mjcf_mass_properties_summary.json`

## 좌우 Mass Pair

| 왼쪽 body | 오른쪽 body | 왼쪽 kg | 오른쪽 kg | 차이 kg | 상대 차이 | CAD 확인 필요 |
| --- | --- | ---: | ---: | ---: | ---: | --- |
| `thighJ_L_1` | `thighJ_R_1` | 0.400000 | 0.400000 | 0.000000 | 0.0000 | `False` |
| `thigh_L_1` | `thigh_R_1` | 0.900000 | 0.920360 | 0.020360 | 0.0221 | `True` |
| `calf_L_1` | `calf_R_1` | 0.900000 | 1.103620 | 0.203620 | 0.1845 | `True` |
| `footJ_L_1` | `footJ_R_1` | 0.300000 | 0.300000 | 0.000000 | 0.0000 | `False` |
| `foot_L_1` | `foot_R_v1_1` | 0.300000 | 0.300000 | 0.000000 | 0.0000 | `False` |

## 해석

좌우 mass pair 차이가 큰 body는 CAD에서 의도된 차이인지 확인해야 한다. 특히 thigh/calf 질량 차이는 보행 안정성과 torque 요구량에 직접 영향을 줄 수 있다.
