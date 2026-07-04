# 사용자 제공 입력 반영 후 판단

## 반영한 입력

이번 단계에서 사용자가 제공한 다음 정보를 config와 모델 variant에 반영했다.

- standing pose: 모든 revolute joint `0.0 rad`
- foot contact box: 좌우 `70 x 120 x 40 mm`
- mass property: 전체 `9.2111 kg`, body aggregate 기준 `9.2114 kg`
- joint 정보: 좌우 hip roll/pitch, knee pitch, ankle pitch/roll 총 10개 revolute joint

## 생성한 config

- standing pose: `/home/king0519/projects/Humanoid/configs/standing_pose_confirmed_zero.json`
- foot contact: `/home/king0519/projects/Humanoid/configs/foot_contact_confirmed_from_user.json`
- component mass: `/home/king0519/projects/Humanoid/configs/cad_component_mass_properties_user.csv`
- aggregate body mass: `/home/king0519/projects/Humanoid/configs/cad_body_mass_aggregate_user.csv`

## 생성한 모델

- contact 적용 모델: `/home/king0519/projects/Humanoid/envs/robots/urdf_f_link/URDF_F_link_user_contact.xml`
- mass + contact 적용 모델: `/home/king0519/projects/Humanoid/envs/robots/urdf_f_link/URDF_F_link_user_mass_contact.xml`

## 통과한 항목

- 사용자 제공 mass를 MuJoCo body 구조에 맞게 합산했다.
- 새 모델 총질량이 `9.2114 kg`로 반영됐다.
- 새 모델은 MuJoCo에서 컴파일된다.
- foot contact config 적용 파이프라인이 동작한다.

## 새로 확인된 문제

사용자 제공 foot contact center를 현재 MJCF body frame에 그대로 넣으면 좌우 contact 높이가 다르게 나온다.

- left sole z: `0.077284 m`
- right sole z: `0.127284 m`
- 차이: 약 `0.05 m`

또한 중립 geometry에서 COM projection이 support AABB 밖에 있다.

- COM inside support x: `False`
- COM inside support y: `False`

따라서 제공된 foot contact box 값은 유효하지만, 그 local frame이 현재 MJCF foot body frame과 직접 같지는 않은 것으로 판단한다.

## PD Standing 결과

0 rad standing pose로 PD probe를 실행했지만 통과하지 못했다.

- 최종 roll: `-2.687330 rad`
- 최종 pitch: `-0.596469 rad`
- 최대 qvel norm: `857.594066`
- 최대 contact force: `3404.384879`
- ankle pitch/roll 계열 RMS torque: 약 `93.8-96.6 Nm`

## 현재 결론

사용자 제공 정보 덕분에 질량과 joint pose 가정은 이전보다 훨씬 명확해졌다. 하지만 foot contact local frame 변환 문제가 남아 있어, 아직 이 모델로 RL 학습이나 모터 최종 판단을 하면 안 된다.

다음 단계는 foot contact box를 Fusion local frame에서 MJCF foot body frame으로 변환하는 것이다. 변환이 맞으면 좌우 foot contact의 world z가 같아지고, support polygon도 실제 발 위치와 맞아야 한다.

## 다음 작업

1. Fusion local frame과 MJCF body frame 사이의 변환을 계산한다.
2. 사용자 제공 foot contact box를 MJCF body-local 좌표로 변환한다.
3. 변환된 contact로 새 모델 variant를 만든다.
4. 0 rad standing pose로 geometry와 PD standing을 다시 실행한다.
5. 그 뒤에도 실패하면 joint axis/sign 또는 actuator torque 한계를 분리한다.
