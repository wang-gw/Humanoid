# Standing Pose와 Foot Contact 템플릿

## 목적

현재 standing 실패의 핵심 원인 중 하나는 실제 standing pose와 실제 발바닥 접촉면이 확정되지 않았다는 점이다. random geometry search로 만든 pose 후보는 동역학 standing을 통과하지 못했다.

따라서 다음 PD standing 검증 전에는 CAD 또는 IK 기준의 standing pose와 실제 foot contact 치수를 입력해야 한다.

## 템플릿 파일

- standing pose: `/home/king0519/projects/Humanoid/configs/standing_pose_template.json`
- foot contact: `/home/king0519/projects/Humanoid/configs/foot_contact_template.json`

## Standing Pose에 필요한 값

- base 높이
- base orientation
- 10개 joint target
- 좌우 발 중심 위치
- stance width
- toe 방향
- pose 산출 방식: CAD, IK, 수동 측정 등

## Foot Contact에 필요한 값

- 좌우 발바닥 collision center
- 좌우 발바닥 collision halfsize
- 마찰계수
- 실제 접촉재 재질
- toe/heel을 하나의 box로 볼지, 여러 contact primitive로 나눌지

## 다음 단계

이 두 템플릿이 채워지면 `URDF_F_named.xml`에 실제 contact 치수를 반영한 새 모델을 만들고, 그 모델에서 PD standing을 다시 실행한다.

