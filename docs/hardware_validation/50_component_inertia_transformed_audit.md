# Component Inertia Transformed Audit

## 목적

component local frame 기준으로 받은 CG/inertia를 현재 MJCF의 mesh geom `pos/quat`로 target body frame에 변환한 뒤, body별 aggregate COM/inertia를 계산한다.

## 입력

- component CSV: `/home/king0519/projects/Humanoid/configs/cad_component_inertia_user.csv`
- 기준 MJCF: `/home/king0519/projects/Humanoid/envs/robots/urdf_f_link/URDF_F_link_user_size_mass_contact.xml`

## Body별 결과

| body | mass kg | aggregate COM | current MJCF COM | delta norm | inertia PD |
| --- | ---: | --- | --- | ---: | --- |
| `base_link` | 2.921400 | `[-0.015599, -0.095238, -0.319315]` | `[0.030886, -0.215714, 0.213569]` | 0.548307 | `True` |
| `thighJ_L_1` | 0.469700 | `[0.257915, 0.067621, -0.522928]` | `[-0.0595, 0.003116, 0.016182]` | 0.628930 | `True` |
| `thigh_L_1` | 0.911300 | `[0.013914, 0.040217, -0.50344]` | `[0.119497, 0.042768, -0.228712]` | 0.294329 | `True` |
| `calf_L_1` | 1.094500 | `[-0.019338, 0.054239, -0.222739]` | `[0.089162, 0.016478, -0.178061]` | 0.123265 | `True` |
| `footJ_L_1` | 0.348200 | `[-0.14375, -0.033092, -0.06458]` | `[0.0, 0.0, -0.000456]` | 0.160845 | `True` |
| `foot_L_1` | 0.321300 | `[-0.14375, 0.0801, -0.05361]` | `[0.0, 9.8e-05, -0.02861]` | 0.166401 | `True` |
| `thighJ_R_1` | 0.469700 | `[0.402772, 0.067621, -0.35211]` | `[-0.0595, 0.003116, 0.016182]` | 0.594554 | `True` |
| `thigh_R_1` | 0.911300 | `[0.196959, 0.040217, -0.156386]` | `[0.213404, -0.050091, -0.225147]` | 0.114690 | `True` |
| `calf_R_1` | 1.094500 | `[0.121388, 0.054239, -0.103454]` | `[0.108823, -0.020659, -0.160062]` | 0.094721 | `True` |
| `footJ_R_1` | 0.348200 | `[0.00975, -0.070733, -0.015988]` | `[0.0, 0.0, -0.000456]` | 0.073072 | `True` |
| `foot_R_v1_1` | 0.321300 | `[0.00975, 0.0801, -0.00361]` | `[0.0, 9.8e-05, -0.02861]` | 0.084383 | `True` |

## 판단

이 결과는 49번 문서의 단순 합산보다 현재 MJCF 구조에 더 가까운 해석이다. 다만 base body 안에 collapse된 `thighJR_*`, `AK45-36_tr*` mesh geom은 현재 MJCF에서 `pos=0`으로 들어가 있어, base 쪽 aggregate COM은 여전히 신뢰도가 낮다.

다리 하위 body도 delta가 남아 있으므로, 이 값을 바로 최종 설계 검증용 inertial로 확정하기 전에 MuJoCo compile 및 geometry/standing probe로 sanity check를 거쳐야 한다.
