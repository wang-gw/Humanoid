# 제공된 정보 기반 입력값 Prefill

## 목적

현재 저장소와 시뮬레이션 모델에서 확정적으로 알 수 있는 값만 검증 입력 파일에 미리 채운다. CAD에서만 알 수 있는 값은 비워 두거나 `확인 필요`로 표시한다.

## 생성 파일

- joint 확인 prefill: `/home/king0519/projects/Humanoid/configs/prefilled/cad_joint_confirmation_prefilled.csv`
- mass property prefill: `/home/king0519/projects/Humanoid/configs/prefilled/cad_mass_properties_prefilled.csv`
- standing pose 후보 prefill: `/home/king0519/projects/Humanoid/configs/prefilled/standing_pose_prefilled_from_failed_candidate.json`
- foot contact placeholder prefill: `/home/king0519/projects/Humanoid/configs/prefilled/foot_contact_prefilled_placeholder.json`

## 채운 값

- joint provisional name
- model axis
- model joint limit
- MJCF body mass
- MJCF inertial position
- MJCF diagonal inertia
- 기존 geometry search pose 후보
- 현재 placeholder foot box collision 값

## 일부러 채우지 않은 값

- CAD에서 확인한 실제 joint 이름
- positive direction의 실제 기구적 의미
- CAD mass/COM/inertia
- 실제 standing pose
- 실제 발바닥 접촉 패드 치수
- 실제 모터/감속기 사양

## 해석

prefill 파일은 작업 시간을 줄이기 위한 초안이다. 특히 standing pose와 foot contact prefill은 이미 동역학 probe에서 실패한 값이므로, 실제 설계값으로 확정하면 안 된다.
