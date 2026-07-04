# Joint Sign Response 감사

## 목적

0 rad standing pose 주변에서 각 joint를 `+/-delta`만큼 움직였을 때 COM과 좌우 foot contact가 어떻게 변하는지 기록한다. 목적은 action order, joint side, axis/sign 문제를 동역학 RL 전에 분리하는 것이다.

## 실행 명령

```bash
python3 scripts/audit_joint_sign_response.py --model envs/robots/urdf_f_link/URDF_F_link_user_size_mass_contact.xml
```

## 기준 상태

- 모델: `/home/king0519/projects/Humanoid/envs/robots/urdf_f_link/URDF_F_link_user_size_mass_contact.xml`
- base z: `0.005000`
- delta: `+/-0.05` rad
- baseline COM: `[0.05335809715461275, -0.07376779677673535, 0.16820644024413228]`
- baseline support margin: `{'inside_x': True, 'inside_y': True, 'x_min_margin': 0.17535809715461276, 'x_max_margin': 0.048641902845387254, 'y_min_margin': 0.007732203223264636, 'y_max_margin': 0.11226779677673535}`

## 요약

| joint | side | axis | own foot sensitivity / rad | own norm | COM sensitivity / rad | COM norm |
| --- | --- | --- | --- | ---: | --- | ---: |
| `left_hip_roll` | `left` | `0 -1 0` | `[0.419825021873698, 0.0, 0.02698875140616619]` | 0.420692 | `[0.10369287876426679, 0.0, 0.02721271775196793]` | 0.107204 |
| `left_hip_pitch` | `left` | `1 0 0` | `[0.0, 0.3498541848947484, -0.021491042786391662]` | 0.350514 | `[0.0, 0.08419578220706006, 0.005440216535528564]` | 0.084371 |
| `left_knee_pitch` | `left` | `1 0 0` | `[0.0, 0.2199083447909847, -0.021491042786391662]` | 0.220956 | `[0.0, 0.036693461762226515, 0.0012109053889422516]` | 0.036713 |
| `left_ankle_pitch` | `left` | `1 0 0` | `[0.0, 0.01999166770827135, -0.021491042786391662]` | 0.029352 | `[0.0, 0.0010147607263938374, -0.0007462212251982403]` | 0.001260 |
| `left_ankle_roll` | `left` | `0 -1 0` | `[0.01999166770827121, 0.0, 0.0]` | 0.019992 | `[0.0009975243102683384, 0.0, 0.0]` | 0.000998 |
| `right_hip_roll` | `right` | `0 -1 0` | `[0.4198250218736979, 0.0, -0.02698875140616619]` | 0.420692 | `[0.10120258095492114, 0.0, 0.023157466018778883]` | 0.103818 |
| `right_hip_pitch` | `right` | `-1 0 0` | `[0.0, -0.3498541848947484, 0.021491042786391662]` | 0.350514 | `[0.0, -0.08170548439771441, 0.008153414535738146]` | 0.082111 |
| `right_knee_pitch` | `right` | `1 0 0` | `[0.0, 0.2199083447909847, -0.021491042786391662]` | 0.220956 | `[0.0, 0.03455570873532321, -0.003199927949649739]` | 0.034704 |
| `right_ankle_pitch` | `right` | `-1 0 0` | `[0.0, -0.01999166770827135, 0.021491042786391662]` | 0.029352 | `[0.0, -0.0010147607263938374, 0.0007462212251982403]` | 0.001260 |
| `right_ankle_roll` | `right` | `0 -1 0` | `[0.01999166770827121, 0.0, 0.0]` | 0.019992 | `[0.0009975243102683384, 0.0, 0.0]` | 0.000998 |

## 산출물

- CSV: `/home/king0519/projects/Humanoid/outputs/analysis/joint_sign_response_user_size_mass_contact_exact005/joint_sign_response.csv`
- JSON: `/home/king0519/projects/Humanoid/outputs/analysis/joint_sign_response_user_size_mass_contact_exact005/joint_sign_response_summary.json`

## 해석

각 joint가 자기 쪽 foot contact에 가장 크게 반응하면 side/action order는 큰 틀에서 맞는 것으로 본다. 반대로 반대쪽 foot만 크게 움직이거나, 예상 역할과 전혀 다른 축 방향 민감도가 나오면 joint mapping 또는 sign 확인이 필요하다.

이 감사는 기구학적 sign response만 본다. 동역학 standing 실패 여부는 별도 PD probe 결과와 함께 판단해야 한다.