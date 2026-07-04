# Frame-normalized contact 검증 후 판단

## 수행한 일

1. 사용자 제공 foot contact center를 그대로 쓰는 경우와 MJCF body 기준으로 정규화하는 경우를 비교했다.
2. 사용자 제공 발 크기 `70 x 120 x 40 mm`는 유지했다.
3. 좌우 동일 body-local center `[0, 0, -0.02] m`, halfsize `[0.035, 0.06, 0.02] m` 후보를 만들었다.
4. 사용자 mass를 다시 적용한 `URDF_F_link_user_size_mass_contact.xml`을 만들었다.
5. geometry 분석과 0 rad PD standing probe를 실행했다.

## 통과한 항목

- 좌우 contact 높이 차이: `0.0 m`
- COM inside support x: `True`
- COM inside support y: `True`
- MuJoCo compile 성공
- 총질량: `9.211400 kg`

## 실패한 항목

- 0 rad standing pose PD probe 실패
- 최대 qvel norm: `791.293011`
- 최대 contact normal force: `5696.328774`
- ankle pitch/roll RMS torque: 약 `94-97 Nm`

## 현재 판단

foot contact frame 문제는 상당히 분리됐다. 직접 입력값을 쓰면 좌우 높이가 어긋났지만, frame-normalized contact에서는 기하 조건이 통과한다.

그럼에도 standing이 실패하므로, 이제 주 원인 후보는 다음으로 이동한다.

- joint axis 또는 positive direction 불일치
- ankle pitch/roll 축 또는 sign 문제
- 0 rad pose가 CAD상 직립이어도 MuJoCo body/inertial frame에서는 동역학적으로 불안정한 자세일 가능성
- actuator torque limit 또는 ankle 기구 토크 부족
- mass/inertia aggregate에서 COM/inertia가 아직 정확히 변환되지 않은 문제

## 다음 단계

다음에는 joint별 작은 양/음 step을 넣어 COM, foot, base torque 반응을 확인한다. 목적은 각 actuator의 sign과 실제 역할이 joint 이름과 일치하는지 검증하는 것이다.

이 검증을 통과하기 전에는 RL 학습으로 넘어가지 않는다.
