# 현재 실행 Gate

## 통과한 항목

- 새 `link/` STL 파일을 별도 MJCF 모델에 반영했다.
- `AK45-36_trL (1).stl`을 오른쪽 `trR`로 매핑했다.
- 오른쪽 foot mesh를 `foot_R.stl`로 매핑했다.
- `URDF_F_link_named.xml`이 MuJoCo에서 컴파일된다.
- STEP assembly transform을 파싱해 좌우 foot origin이 약 `154 mm` 분리되어 있음을 확인했다.
- foot contact JSON을 읽어 새 MJCF variant를 만드는 파이프라인을 만들었다.
- `URDF_F_link_step_contact.xml`에서 COM projection은 support AABB 안에 들어온다.

## 아직 통과하지 못한 항목

- 기존 pose 후보로는 PD standing을 통과하지 못한다.
- ankle pitch/roll RMS torque가 `95-97 Nm` 수준으로 임시 `100 Nm` 제한에 거의 붙는다.
- 좌우 thigh/calf 질량 차이가 CAD 의도값인지 확인되지 않았다.
- joint axis와 action sign이 CAD 기준으로 최종 확정되지 않았다.
- 실제 standing pose가 아직 없다.
- 실제 발바닥 패드 중심/크기가 아직 없다.

## 현재 판단

현재 모델은 “시뮬레이션 파일 준비” 단계는 상당 부분 통과했다. 하지만 “RL만 하면 걸을 수 있는 하드웨어인지”를 판단하기에는 아직 부족하다.

특히 지금 상태에서 RL 학습을 시작하면 다음 문제가 섞인다.

- 정책이 잘못된 pose/contact/mass를 보상으로 학습할 수 있다.
- torque saturation 때문에 정책 실패와 actuator 부족을 구분하기 어렵다.
- 실제 로봇 발바닥 접촉과 다른 조건을 학습할 수 있다.

## 다음에 반드시 필요한 입력

1. 실제 양발 standing pose
2. 실제 foot contact center와 halfsize
3. link별 CAD mass, COM, inertia
4. 후보 모터/감속기별 continuous/peak torque, velocity, gear ratio
5. joint axis와 positive direction 확인

## 다음 실행 순서

1. 위 입력값을 config에 채운다.
2. 기존 모델을 덮어쓰지 않고 새 설계 기준 MJCF variant를 만든다.
3. PD standing을 다시 실행한다.
4. standing 통과 후 squat, weight shift, one-leg support 순서로 진행한다.
5. 그 뒤에 RL standing/walking 학습으로 넘어간다.
