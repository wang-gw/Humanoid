# Standing Pose 탐색

## 목적

neutral pose는 COM이 단순화한 foot support 영역 밖에 있다. 이 탐색은 동역학 PD standing probe를 다시 실행하기 전에, 기하학적으로 가능한 첫 standing pose 후보를 찾기 위한 것이다.

## 실행 명령

```bash
python3 scripts/search_standing_pose.py
```

## 결과

- 후보 config: `/home/king0519/projects/Humanoid/configs/standing_pose_candidate.json`
- sample 수: `30000`
- base-z 조건을 통과한 sample 수: `9172`
- score, 최소 COM support margin: `0.211266`
- Base z: `0.042758`
- COM world: `[0.05104186820625519, -0.11994849311914402, 0.26845340782292987]`
- Support AABB: `{'x_min': -0.22166763386413885, 'x_max': 0.27048753432126804, 'y_min': -0.33121405624162303, 'y_max': 0.12314903367536877}`

## Joint Target

- `Revolute 26`: `0.787078` rad
- `Revolute 19`: `-0.553453` rad
- `Revolute 21`: `-0.494554` rad
- `Revolute 23`: `-0.466391` rad
- `Revolute 25`: `0.327947` rad
- `Revolute 51`: `-0.472656` rad
- `Revolute 44`: `0.030452` rad
- `Revolute 46`: `0.349832` rad
- `Revolute 48`: `-0.306447` rad
- `Revolute 53`: `-0.081376` rad

## 해석

이 pose는 geometry 후보일 뿐, 검증된 standing controller가 아니다. 반드시 forward dynamics와 torque logging으로 다시 검증해야 한다.
