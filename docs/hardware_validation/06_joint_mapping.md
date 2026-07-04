# 임시 조인트 매핑

## 목적

현재 CAD export에서 나온 `Revolute 19` 같은 조인트 이름은 하드웨어 검증이나 RL 디버깅에 그대로 쓰기 어렵다. 최종 기구 설계 이름이 확정되기 전까지, 이 문서는 export된 조인트 이름을 사람이 이해할 수 있는 역할로 임시 매핑한다.

## 현재 Revolute 조인트 순서

| 방향 | export 조인트 | 임시 역할 | 축 | 비고 |
| --- | --- | --- | --- | --- |
| Left | `Revolute 26` | 왼쪽 hip lateral/roll 후보 | `0 -1 0` | 왼쪽 상부 다리 branch의 첫 actuated joint |
| Left | `Revolute 19` | 왼쪽 hip pitch 후보 | `1 0 0` | thigh link 근처 parent |
| Left | `Revolute 21` | 왼쪽 knee pitch 후보 | `1 0 0` | thigh에서 calf로 연결 |
| Left | `Revolute 23` | 왼쪽 ankle pitch 후보 | `1 0 0` | calf에서 foot joint로 연결 |
| Left | `Revolute 25` | 왼쪽 ankle roll 후보 | `0 -1 0` | foot actuator body에서 foot으로 연결 |
| Right | `Revolute 51` | 오른쪽 hip lateral/roll 후보 | `0 -1 0` | 오른쪽 상부 다리 branch의 첫 actuated joint |
| Right | `Revolute 44` | 오른쪽 hip pitch 후보 | `-1 0 0` | thigh link 근처 parent |
| Right | `Revolute 46` | 오른쪽 knee pitch 후보 | `1 0 0` | thigh에서 calf로 연결 |
| Right | `Revolute 48` | 오른쪽 ankle pitch 후보 | `-1 0 0` | calf에서 foot joint로 연결 |
| Right | `Revolute 53` | 오른쪽 ankle roll 후보 | `0 -1 0` | foot actuator body에서 foot으로 연결 |

## 상태

이 매핑은 임시다. 본격적인 RL 학습 전에 각 조인트를 모델 안에서 직접 rename하거나, 별도 config를 통해 안정적인 action order를 정의해야 한다. 그래야 torque plot, action vector, policy output을 사람이 해석할 수 있다.

## 반드시 확인해야 할 것

- CAD에서 각 export 조인트의 실제 기구적 의미 확인
- 각 조인트의 positive direction 확인
- RL action order 확정
- MJCF joint/actuator rename 또는 안정적인 mapping config 생성
