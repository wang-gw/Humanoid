# 좌우 Kinematic Alignment 감사

## 목적

`URDF_F_link_named.xml`의 좌우 발 collision이 중립 자세에서 같은 world 위치에 겹치는 원인을 확인한다. 이 단계는 controller나 RL 문제가 아니라 모델 좌표계와 mesh origin이 올바른지 확인하는 감사다.

## 실행 명령

```bash
python3 scripts/audit_kinematic_alignment.py --model envs/robots/urdf_f_link/URDF_F_link_named.xml
```

## 핵심 결과

- 모델: `/home/king0519/projects/Humanoid/envs/robots/urdf_f_link/URDF_F_link_named.xml`
- 좌우 foot body 거리: `0.154000` m
- 좌우 foot body delta L-R: `[0.154, 0.0, 0.0]`
- 좌우 sole collision 거리: `0.000000` m
- 좌우 sole collision delta L-R: `[3.999999997894577e-09, 0.0, 0.0]`

## Body/Geom 위치

| 항목 | 타입 | parent/body | local pos | world pos |
| --- | --- | --- | --- | --- |
| `thighJ_L_1` | `body` | `base_link` | `[0.04, -0.029, 0.44]` | `[0.04, -0.029, 0.44]` |
| `thighJ_R_1` | `body` | `base_link` | `[-0.06, -0.029, 0.44]` | `[-0.06, -0.029, 0.44]` |
| `footJ_L_1` | `body` | `calf_L_1` | `[0.0, 0.0, -0.2]` | `[0.067, 0.0, 0.03999999999999998]` |
| `footJ_R_1` | `body` | `calf_R_1` | `[0.0, 0.0, -0.2]` | `[-0.087, 0.0, 0.03999999999999998]` |
| `foot_L_1` | `body` | `footJ_L_1` | `[0.0, -0.0215, 0.0]` | `[0.067, -0.0215, 0.03999999999999998]` |
| `foot_R_v1_1` | `body` | `footJ_R_1` | `[0.0, -0.0215, 0.0]` | `[-0.087, -0.0215, 0.03999999999999998]` |
| `foot_L_1_sole_collision` | `geom` | `foot_L_1` | `[-0.047691176, 0.0215, -0.041535671]` | `[0.019308824000000002, 0.0, -0.0015356710000000232]` |
| `foot_R_v1_1_sole_collision` | `geom` | `foot_R_v1_1` | `[0.10630882, 0.0215, -0.041535671]` | `[0.019308820000000004, 0.0, -0.0015356710000000232]` |

## 좌우 Pair Delta

| left | right | distance m | delta L-R |
| --- | --- | ---: | --- |
| `thighJ_L_1` | `thighJ_R_1` | 0.100000 | `[0.1, 0.0, 0.0]` |
| `footJ_L_1` | `footJ_R_1` | 0.154000 | `[0.154, 0.0, 0.0]` |
| `foot_L_1` | `foot_R_v1_1` | 0.154000 | `[0.154, 0.0, 0.0]` |
| `foot_L_1_sole_collision` | `foot_R_v1_1_sole_collision` | 0.000000 | `[3.999999997894577e-09, 0.0, 0.0]` |

## 산출물

- JSON: `/home/king0519/projects/Humanoid/outputs/analysis/kinematic_alignment_link/kinematic_alignment_summary.json`
- body/geom CSV: `/home/king0519/projects/Humanoid/outputs/analysis/kinematic_alignment_link/body_and_geom_alignment.csv`
- pair delta CSV: `/home/king0519/projects/Humanoid/outputs/analysis/kinematic_alignment_link/left_right_pair_deltas.csv`
- joint axis CSV: `/home/king0519/projects/Humanoid/outputs/analysis/kinematic_alignment_link/joint_axes_world.csv`

## 판단

좌우 foot body는 서로 떨어져 있지만, foot mesh와 sole collision의 local offset이 그 차이를 상쇄해서 최종 접촉 중심이 같은 world 위치로 겹친다. 따라서 현재 standing 실패는 단순히 RL 학습 부족으로 볼 수 없다.

다음 단계에서는 CAD/Fusion 기준으로 foot link origin과 visual/collision origin이 body local 좌표인지, 또는 assembly absolute 좌표가 섞여 들어온 것인지 확인해야 한다. 확인 전에는 contact box를 임의로 옮겨서 최종 설계 검증에 사용하지 않는다.
