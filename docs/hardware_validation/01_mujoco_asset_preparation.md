# MuJoCo Asset 준비

## 목적

RL 검증은 MuJoCo에서 진행할 예정이므로 CAD xacro export를 MuJoCo가 직접 읽을 수 있는 URDF asset으로 준비한다.

원본 `URDF_F.xacro`는 실제 매크로 계산보다는 `materials`, `transmission`, `gazebo` include를 사용하는 구조다. 현재 단계에서는 정적 구조 검증과 MuJoCo 로딩이 목적이므로 include를 제거하고 link/joint 구조만 보존한 URDF를 생성한다.

보행 RL에는 floating base가 필요하므로 생성 스크립트는 기본적으로 `world -> base_link` floating joint를 추가한다. 고정 베이스 동역학만 보고 싶을 때는 `--fixed-base`를 사용한다.

## 실행 명령

```bash
python3 scripts/prepare_mujoco_urdf.py
```

## 산출물

- `envs/robots/urdf_f/URDF_F_mujoco.urdf`
- `envs/robots/urdf_f/*.stl`
- `docs/hardware_validation/mujoco_load_report.json`
- `docs/hardware_validation/mujoco_load_report.md`

## 로딩 확인

```bash
python3 scripts/check_mujoco_model.py
```

## 메모

- 이 파일은 최종 모델이 아니라 MuJoCo 로딩과 초기 probe를 위한 준비 asset이다.
- 생성된 URDF의 mesh filename은 basename만 사용하고, STL 파일을 URDF와 같은 디렉터리에 둔다. MuJoCo URDF 로더가 mesh 경로를 basename으로 처리하기 때문에 이 구조가 초기 로딩 검증에 가장 단순하다.
- actuator, motor limit, damping, friction, keyframe은 아직 정리되지 않았다.
- 다음 단계는 이 URDF가 MuJoCo에서 실제로 컴파일되는지 확인하고, 실패하면 collision geometry, mesh, joint naming 문제를 수정하는 것이다.
