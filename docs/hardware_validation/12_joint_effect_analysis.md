# 조인트 영향 분석

## 목적

CAD export 조인트 이름과 positive direction이 아직 확정되지 않았기 때문에, 각 조인트를 작은 각도만큼 움직였을 때 좌우 발 collision 중심이 어떻게 이동하는지 수치로 기록한다.

이 분석은 동역학 검증이 아니라 순수 기구학적 영향 분석이다. 최종 매핑 확정에는 CAD 축 방향 확인이 필요하다.

## 실행 명령

```bash
python3 scripts/analyze_joint_effects.py
```

## 입력

- 모델: `/home/king0519/projects/Humanoid/envs/robots/urdf_f/URDF_F_named.xml`
- pose 후보: `/home/king0519/projects/Humanoid/configs/standing_pose_candidate_named.json`
- perturbation: `+/-0.1` rad

## 산출물

- CSV: `/home/king0519/projects/Humanoid/outputs/analysis/joint_effects_named/joint_effects.csv`
- JSON: `/home/king0519/projects/Humanoid/outputs/analysis/joint_effects_named/joint_effects_summary.json`

## 요약

| 조인트 | 축 | 가장 크게 영향받은 대상 | 민감도 norm / rad |
| --- | --- | --- | ---: |
| `left_hip_roll` | `0 -1 0` | `foot_L_1_sole_collision` | 0.283103 |
| `left_hip_pitch` | `1 0 0` | `foot_L_1_sole_collision` | 0.364549 |
| `left_knee_pitch` | `1 0 0` | `foot_L_1_sole_collision` | 0.249638 |
| `left_ankle_pitch` | `1 0 0` | `foot_L_1_sole_collision` | 0.054592 |
| `left_ankle_roll` | `0 -1 0` | `foot_L_1_sole_collision` | 0.063138 |
| `right_hip_roll` | `0 -1 0` | `foot_R_v1_1_sole_collision` | 0.436240 |
| `right_hip_pitch` | `-1 0 0` | `foot_R_v1_1_sole_collision` | 0.370388 |
| `right_knee_pitch` | `1 0 0` | `foot_R_v1_1_sole_collision` | 0.247755 |
| `right_ankle_pitch` | `-1 0 0` | `foot_R_v1_1_sole_collision` | 0.049956 |
| `right_ankle_roll` | `0 -1 0` | `foot_R_v1_1_sole_collision` | 0.113945 |

## 해석

각 조인트가 어느 발에 더 큰 영향을 주는지 확인하면 좌우 branch와 action order를 검증하는 데 도움이 된다. 다만 현재 pose 후보 자체가 동역학 standing을 통과하지 못했으므로, 이 결과는 조인트 매핑 보조 자료로만 사용한다.

다음 단계에서는 CAD에서 joint 이름, 실제 회전축, positive direction을 확인한 뒤 이 표와 비교해야 한다.
