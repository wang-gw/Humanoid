# link STL 적용 후 현재 판단

## 시작 지점

사용자가 새로 추가한 파일은 다음과 같다.

- STEP: `/home/king0519/projects/Humanoid/URDF_F_.step`
- STL 폴더: `/home/king0519/projects/Humanoid/link`

사용자 확인으로 아래 매핑을 확정했다.

- `AK45-36_trL (1).stl`은 오른쪽 `trR`로 사용한다.
- 오른쪽 foot mesh는 `foot_R.stl`을 사용한다.
- 기존 모델은 덮어쓰지 않고 새 모델을 만든다.

## 수행한 작업

1. STEP와 `link/` STL 파일을 감사했다.
2. 확정된 파일명 매핑으로 새 MJCF 모델을 만들었다.
3. 새 모델을 MuJoCo에서 컴파일했다.
4. 새 모델의 중립 기하, 질량/관성, PD standing probe를 실행했다.

## 현재 산출물

- 새 모델: `/home/king0519/projects/Humanoid/envs/robots/urdf_f_link/URDF_F_link_named.xml`
- STL 매핑 문서: `/home/king0519/projects/Humanoid/docs/hardware_validation/25_link_stl_named_model.md`
- MuJoCo 로드 리포트: `/home/king0519/projects/Humanoid/docs/hardware_validation/link_named_model_load/mujoco_load_report.md`
- 중립 기하 분석: `/home/king0519/projects/Humanoid/docs/hardware_validation/link_named_geometry/08_standing_geometry_analysis.md`
- 질량/관성 감사: `/home/king0519/projects/Humanoid/docs/hardware_validation/link_named_mass/17_mjcf_mass_property_audit.md`
- PD standing 결과: `/home/king0519/projects/Humanoid/docs/hardware_validation/26_link_named_pose_pd_standing_probe.md`

## 판단

새 STL 파일들은 별도 모델로 반영됐고 MuJoCo 컴파일도 통과했다. 따라서 “새 파일을 시뮬레이션 모델에 넣을 수 있는가”는 통과했다.

하지만 “이 하드웨어 형상이 RL만 하면 걸을 가능성이 충분한가”는 아직 통과하지 못했다. 이유는 다음과 같다.

- 중립 자세에서 COM projection이 단순 foot support 영역 밖에 있다.
- 좌우 foot collision primitive가 같은 위치에 겹쳐 있어 실제 양발 지지면을 대표하지 못한다.
- 기존 pose 후보를 넣은 PD standing에서 base가 안정적으로 유지되지 않는다.
- 대부분의 actuator가 `+/-100 Nm` 임시 토크 제한에 닿는다.
- `thigh`와 `calf` 질량이 좌우에서 차이가 나며 CAD 의도값인지 확인이 필요하다.

## 다음 단계

다음 검증은 새 모델 `URDF_F_link_named.xml`을 기준으로 진행한다.

1. CAD 또는 Fusion 360에서 실제 양발 standing pose를 확정한다.
2. 실제 발바닥 접촉면 치수와 좌우 foot 위치를 확정한다.
3. link별 질량, COM, inertia가 CAD 기준과 일치하는지 확인한다.
4. 위 정보를 반영한 새 모델 variant를 만든다.
5. PD standing을 다시 실행해 base 안정성, contact force, torque saturation을 확인한다.

이 단계가 통과되기 전에는 RL 보행 학습이나 최종 모터 선정으로 넘어가지 않는다.
