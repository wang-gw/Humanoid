# 현재 Standing Probe 결과 정리

## 목적

RL로 넘어가기 전에 지금까지 수행한 standing 검증 결과를 정리한다.

## 비교

| 케이스 | 최종 base z | 최종 roll rad | 최종 pitch rad | 최대 qvel norm | 최대 접촉력 | 최대 관측 토크 | 그래프 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| `mesh_collision_neutral` | 0.505543 | -2.866803 | -0.015737 | 841.044941 | 14261.978865 | 100.000000 | `/home/king0519/projects/Humanoid/outputs/analysis/pd_standing/neutral_pd_standing_plot.png` |
| `foot_box_neutral` | 0.610656 | -2.491595 | -0.291315 | 811.402946 | 2635.417934 | 100.000000 | `/home/king0519/projects/Humanoid/outputs/analysis/pd_standing_contact/neutral_pd_standing_plot.png` |
| `foot_box_pose_candidate` | 3.673125 | -1.077786 | -0.238622 | 915.532495 | 2071.996429 | 100.000000 | `/home/king0519/projects/Humanoid/outputs/analysis/pd_standing_pose_candidate/neutral_pd_standing_plot.png` |
| `named_foot_box_pose_candidate` | 3.673125 | -1.077786 | -0.238622 | 915.532495 | 2071.996429 | 100.000000 | `/home/king0519/projects/Humanoid/outputs/analysis/pd_standing_named_pose_candidate/neutral_pd_standing_plot.png` |

## 판단

현재 모델은 아직 RL standing 또는 walking 학습으로 넘어갈 준비가 되지 않았다.

단순화한 foot collision은 full mesh collision보다 접촉력 peak를 줄였지만, 안정적인 standing을 만들지는 못했다. geometry 기반 pose 후보는 COM을 support AABB 안에 넣었지만, 동역학 probe에서는 여전히 접촉이 끊기고 torque saturation이 발생했다.

`URDF_F_named.xml`은 기존 contact 모델에 임시 joint mapping을 적용한 모델이다. 동역학 결과는 동일하게 실패하지만, torque log와 action order를 사람이 읽을 수 있으므로 이후 분석은 named 모델을 기준으로 진행한다.

## 가장 가능성 높은 문제

- neutral pose가 실제 standing pose가 아니다.
- foot support geometry가 아직 불확실하다. 단순화한 좌우 foot box가 초기 world 좌표에서 거의 겹친다.
- CAD-export된 joint 이름과 positive direction이 아직 검증되지 않았다.
- 현재 placeholder torque motor가 계속 saturation에 걸리므로 actuator 가정과 pose/control 문제가 서로 섞여 있다.
- URDF에서 MJCF로 변환되며 합쳐진 mesh 기반 inertial 값이 CAD mass property와 일치하는지 재검토해야 한다.

## 다음에 해야 할 일

1. CAD 기준으로 joint semantic mapping과 positive direction을 확인한다.
2. random geometry search가 아니라 CAD/IK 기준의 실제 mechanical standing pose를 정의한다.
3. placeholder foot box를 실제 의도한 sole contact 치수와 stance width로 교체한다.
4. 확정된 pose/contact model로 PD standing을 다시 실행한다.
5. 안정적인 standing을 통과한 뒤에만 squat, weight shift, one-leg support, RL standing으로 넘어간다.
