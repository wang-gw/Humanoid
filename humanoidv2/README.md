# Humanoidv2 Clean-Room

신규 CAD 기반 CHIRO Humanoid v2를 대상으로 강화학습 환경, observation, action,
보상함수와 학습 코드를 처음부터 다시 설계하기 위한 clean-room 프로젝트다.

## 현재 포함 범위

로봇 모델과 KHR-3HV 논문 방식의 첫 clean-room 강화학습 기준 실험을 포함한다.

- 최종 MuJoCo 모델: `models/urdf_f_v2/URDF_F_v2_footprint_contact.xml`
- 모델 계보 확인용 URDF/MJCF: 같은 디렉터리의 `.urdf`, `.xml`
- 모델이 참조하는 시각/충돌 형상: 같은 디렉터리의 STL 23개
- KHR 방식 Gymnasium 환경: `humanoidv2/khr3hv_env.py`
- 학습/평가 스크립트: `scripts/`
- V1 실험 기록: `docs/KHR3HV_V1_EXPERIMENT.md`
- 학습 정책·영상·수치: `results/khr3hv_v1/`

기존 `Humanoid`, Open Duck Playground, Berkeley Humanoid 프로젝트의 다음 항목은
의도적으로 복사하지 않았다.

- 강화학습 환경과 observation/action 구현
- 보상함수
- 학습·평가·탐색 스크립트
- 기존 정책, reset pool, teacher label과 실험 결과
- 기존 자세·관절 매핑·액추에이터 설정

> **전후 방향 감사(2026-08-08):** V1–V12는 월드 `+Y`를 전방으로 사용했지만 로봇 형상의
> 해부학적 전방은 `-Y`다. 따라서 아래 V1–V12의 과거 `전진` 표기는 실제로는 `+Y` 방향
> 후진 과제의 내부 진행량이다. 결과는 실험 계보 재현을 위해 유지하며, 최초의 정방향
> 결과는 V13이다.

## 모델 기본 정보

- 자유 베이스 + 구동 관절 10개
- `nq=17`, `nv=16`, `nu=10`
- 관절 순서: 좌우 `hip_roll`, `hip_pitch`, `knee_pitch`, `ankle_pitch`, `ankle_roll`
- 최종 모델에는 발바닥 box contact geom과 관절 armature/damping이 포함되어 있다.

새 환경은 최종 XML을 직접 로드하고 reset pose와 제어 주기부터 이 프로젝트의 명시적
설정으로 정의한다. 기존 프로젝트의 Python 모듈은 import하지 않는다.

## KHR-3HV V1 결과

2026-08-04에 507,904단계 PPO 학습을 실행했다. 정책은 20초 평가 동안 낙상하지 않았지만
순전진은 1.95 cm, 평균 전진속도는 0.0011 m/s에 그쳐 목표 0.15 m/s 보행에는 실패했다.
결과 영상과 실패 원인, V2 권고안은
[`docs/KHR3HV_V1_EXPERIMENT.md`](docs/KHR3HV_V1_EXPERIMENT.md)에 기록했다.

후속 V2 계열에서는 충돌 proxy, 엄격한 속도 보상, 현재 로봇의 좌우 관절 부호 및
조기종료 유인을 검증했다. 최고 순전진은 V2 35만 단계 정책의 16.06 cm였으며 목표 보행에는
아직 실패했다. 전체 변경과 영상은
[`docs/KHR3HV_V2_EXPERIMENT.md`](docs/KHR3HV_V2_EXPERIMENT.md)에 기록했다.

> V3 분석에서 V2 collision proxy의 내부 겹침이 발견됐으므로 V2 이동 수치는 유효한 최종
> 기준이 아니다. 수정된 물리에서 실행한 V3.4의 최고 순전진은 7.40 cm였고 실제 단일지지는
> 발생하지 않았다. 정정 내용과 산출물은
> [`docs/KHR3HV_V3_EXPERIMENT.md`](docs/KHR3HV_V3_EXPERIMENT.md)에 기록했다.

V4에서는 연속 보행 전에 좌우 정적 단일지지를 별도 과제로 분리했다. 5만 단계 PPO 정책이
양쪽 모두 8초 동안 낙상 없이 스윙발 접촉력 0 N을 유지해 성공했다. 10만 단계에서는 왼쪽
과제가 회귀했으므로 양쪽 분리 검증으로 5만 단계 checkpoint를 선택했다. 영상, 수치,
성공 조건과 다음 단일-step 설계는
[`docs/KHR3HV_V4_EXPERIMENT.md`](docs/KHR3HV_V4_EXPERIMENT.md)에 기록했다.

V4 후속 감사에서 원본 CAD 관성의 전체 COM이 양발 중심에서 lateral 15.22 mm 벗어나 있고
링크별 관성도 좌우 미러 기준으로 크게 다름을 확인했다. 전체 질량을 보존한 대칭 관성 파생
모델에서는 중립 발 하중이 41.004/41.004 N으로 같아졌으며, 동일한 목표와 동일 PD gain으로
좌우 단일지지와 PPO 학습이 모두 성공했다. 생성 방법, 원본 대비 영상과 정량 비교는
[`docs/INERTIA_SYMMETRY_EXPERIMENT.md`](docs/INERTIA_SYMMETRY_EXPERIMENT.md)에 기록했다.

대칭 모델 기반 V5에서는 `양발 지지 → 한 발 들기 → 전진 → 양발 착지`의 단일 step을
좌우 모두 성공했다. 57,344-step PPO 정책은 10초 동안 낙상 없이 스윙발을 약 32 mm 앞에
놓고 착지 후 양발 접촉을 유지했다. 아직 연속 보행은 아니며, 결과와 다음 2-step 연결 조건은
[`docs/KHR3HV_V5_SINGLE_STEP_EXPERIMENT.md`](docs/KHR3HV_V5_SINGLE_STEP_EXPERIMENT.md)에 기록했다.

V6에서는 첫 착지 상태를 reset하지 않고 반대발을 이어서 옮겼다. 57,344-step PPO 정책이
좌→우와 우→좌 순서 모두 20초 동안 두 번의 발 분리·전진·착지를 완주했고, 몸통은 각각
48.61/47.52 mm 전진했다. 아직 두 걸음짜리 저속 기준동작이며 연속 보행은 아니다. 영상,
접촉력, checkpoint 비교와 V7 반복주기 조건은
[`docs/KHR3HV_V6_TWO_STEP_EXPERIMENT.md`](docs/KHR3HV_V6_TWO_STEP_EXPERIMENT.md)에 기록했다.

V7에서는 각 착지 목표를 다음 lift IK의 시작점으로 전달해 네 걸음을 reset 없이 연결했다.
50,000-step PPO checkpoint가 좌→우→좌→우와 반대 순서 모두 성공해 몸통이
113.48/111.87 mm 전진했다. 57,344-step 정책은 좌측 시작에서 회귀했으므로 양방향 검증으로
제외했다. 비교 영상, 네 step별 접촉·보폭과 온라인 re-anchoring이 필요한 이유는
[`docs/KHR3HV_V7_FOUR_STEP_EXPERIMENT.md`](docs/KHR3HV_V7_FOUR_STEP_EXPERIMENT.md)에 기록했다.

V8에서는 실제 착지 관절 상태를 다음 step의 시작점으로 갱신하고 검증된 명목 도착 자세로
오차 누적을 막는 하이브리드 재기준화를 적용했다. 57,344-step PPO 정책이 양쪽 시작에서
전방 8스텝을 모두 성공해 몸통이 249.16/248.51 mm 전진했다. 실제 월드 발 위치를 그대로
다음 IK 기준으로 쓰는 방식은 후반 낙상으로 실패했다. 비교 결과, 영상과 다음 속도 압축 조건은
[`docs/KHR3HV_V8_EIGHT_STEP_EXPERIMENT.md`](docs/KHR3HV_V8_EIGHT_STEP_EXPERIMENT.md)에 기록했다.

V9에서는 같은 전방 8스텝을 10초/step에서 6초/step으로 압축했다. 기준궤적은 엄격 접촉
표본 조건을 놓쳤지만 PPO가 이를 보정해 양쪽 모두 48초 동안 약 248 mm 전진했다. 평균
속도는 V8보다 약 66% 증가한 0.00518 m/s다. 다만 최대 착지력이 65.82 N까지 증가했으므로
다음 단계는 추가 가속보다 착지 충격 완화가 우선이다. 전체 결과는
[`docs/KHR3HV_V9_FAST_EIGHT_STEP_EXPERIMENT.md`](docs/KHR3HV_V9_FAST_EIGHT_STEP_EXPERIMENT.md)에 기록했다.

V10에서는 6초/step을 유지하고 착지 시간을 1.2초에서 2.0초로 늘려 충격을 낮췄다. V9 PPO
정책으로 양쪽 시작 모두 8스텝을 성공했으며 `land + settle` 최대 접촉력은 42.61/47.45 N으로
50 N 제한을 통과했다. V9 대비 최악 충격은 27.91% 감소했다. 별도 PPO 전이학습은 조기
낙상으로 실패해 채택하지 않았고 그 결과도 함께 보존했다. 전체 탐색, 영상과 수치는
[`docs/KHR3HV_V10_SOFT_LANDING_EXPERIMENT.md`](docs/KHR3HV_V10_SOFT_LANDING_EXPERIMENT.md)에 기록했다.

V11에서는 V10 정책을 초기 자세, 마찰, 전체 질량과 좌우 다리 질량 비대칭 아래에서 78회
검증했다. 엄격 성공은 51회(65.4%)였고 실패 27회 중 낙상 18회, 발 분리 실패 7회, 50 N
초과는 2회였다. 별도의 72회 비대칭 sweep은 현재 정책이 좌우 물성 오차에 방향 의존적으로
민감함을 확인했다. 결과와 대표 성공/실패 영상은
[`docs/KHR3HV_V11_ROBUSTNESS_EXPERIMENT.md`](docs/KHR3HV_V11_ROBUSTNESS_EXPERIMENT.md)에 기록했다.

V12에서는 1e-5 learning rate, 2 PPO epochs, 0.05 clip과 0.002 target KL로 3단계 domain
randomization 전이학습을 시도했다. 그러나 가장 약한 단계부터 우측 시작 명목 성능이 깨졌고,
40회 검증 성공 수가 V10의 28회에서 최종 16회로 감소했다. 모든 학습 checkpoint를 배제해
최종 archive는 V10과 parameter 차이 0으로 유지했다. 실패 분석과 영상은
[`docs/KHR3HV_V12_CURRICULUM_EXPERIMENT.md`](docs/KHR3HV_V12_CURRICULUM_EXPERIMENT.md)에 기록했다.

V10 영상에는 월드 고정 좌측 광각 카메라도 추가했다. 기존 카메라는 robot base를 매 frame
추적해 전진이 잘 보이지 않았지만, 새 영상은 고정 시점과 화면상 전진거리로 약 0.245 m의
`+Y` 후진 이동을 직접 확인할 수 있다.

V13에서는 전방을 월드 `-Y`로 수정하고 발 목표, 보폭, 몸통 진행량과 성공 판정에 같은
방향 부호를 적용했다. 신규 PPO가 고정 참조 위에서 10% 범위의 작은 관절 잔차만 학습하도록
제한하고, `lift → hold → advance → land → settle`의 6초 주기를 사용했다. 좌·우 선행 모두
48초 동안 8스텝을 완주하며 실제 `-Y`로 232.58/231.57 mm 이동했고, 최대 착지력은
42.42/42.75 N이었다. 이는 정방향 **준정적 스텝 시뮬레이션**의 성공이며 아직 자연스러운
연속 보행 성공은 아니다. 설계, 실패한 전이학습, 최종 영상과 수치는
[`docs/KHR3HV_V13_CORRECTED_FORWARD_EXPERIMENT.md`](docs/KHR3HV_V13_CORRECTED_FORWARD_EXPERIMENT.md)에 기록했다.

V14에서는 첫 속도 커리큘럼으로 주기를 6.00초에서 4.52초/step으로 줄였다. 초기
`kp=90` 참조와 두 번의 전이학습은 발 접촉 해제 조건에 실패했지만, 60회 참조 탐색에서
`kp=110`이 짧아진 주기의 추종 지연을 해결했다. 25,000-step 미세조정 정책이 좌·우 선행
모두 36.16초 동안 8스텝을 성공해 228.68/229.29 mm 전진했고 최대 착지력은
46.53/46.81 N이었다. 평균 속도는 약 6.33 mm/s로 V13보다 약 31% 빨라졌지만 아직
준정적 단계다. 실패 계보, 정면·사선 영상과 다음 3.40초 후보는
[`docs/KHR3HV_V14_DYNAMIC_FORWARD_STAGE1_EXPERIMENT.md`](docs/KHR3HV_V14_DYNAMIC_FORWARD_STAGE1_EXPERIMENT.md)에 기록했다.

V15에서는 주기를 3.40초/step으로 다시 줄였다. 고정 `kp` 탐색에서는 발 접촉 해제와
50 N 착지 제한을 동시에 만족시키지 못했으나, 스윙 `kp=130`·착지 `kp=70`의 phase별
gain scheduling으로 참조와 V14 정책이 모두 양쪽 엄격 조건을 통과했다. 최종 정책은
27.2초 동안 8스텝으로 202.53/202.23 mm 전진했고 최대 착지력은 44.99/46.09 N이었다.
평균 속도 약 7.44 mm/s는 V14보다 약 17.5% 빠르며, 최대 관절 토크 9.16 N·m와 포화율
0%를 확인했다. 별도 PPO 미세조정은 착지 충격을 악화시켜 채택하지 않았다. 전체 탐색,
실패 checkpoint와 영상은
[`docs/KHR3HV_V15_DYNAMIC_FORWARD_STAGE2_EXPERIMENT.md`](docs/KHR3HV_V15_DYNAMIC_FORWARD_STAGE2_EXPERIMENT.md)에 기록했다.

V16에서는 주기를 2.56초/step으로 줄였다. 직접 비례 압축은 착지력이 60–84 N으로 실패해
112회 참조 탐색으로 phase를 `20/8/10/21/5`, 스윙/착지 `kp=130/80`으로 조정했다.
advance 마지막 12%에서 gain을 부드럽게 보간하며, V15 정책이 양쪽 모두 20.48초 동안
8스텝을 성공했다. 전진량은 186.88/184.55 mm, 평균 속도는 9.125/9.011 mm/s, 최대
착지력은 47.35/47.66 N이고 토크 포화는 없었다. 미세조정 정책은 다시 착지 충격을
악화시켜 채택하지 않았다. 오른쪽 전진 기준 여유가 4.55 mm로 작으므로 다음 단계는 추가
압축보다 전진 여유와 강건성 회복이 우선이다. 상세 결과는
[`docs/KHR3HV_V16_DYNAMIC_FORWARD_STAGE3_EXPERIMENT.md`](docs/KHR3HV_V16_DYNAMIC_FORWARD_STAGE3_EXPERIMENT.md)에 기록했다.

V17에서는 2.56초 주기를 유지하고 보폭을 35 mm에서 42 mm로 늘려 전진 여유를 회복했다.
35/38/40/42 mm 탐색에서 38/40 mm는 착지 50 N 제한을 넘었지만 42 mm는 참조와 정책,
좌·우 선행 모두 성공했다. 최종 정책은 239.07/242.67 mm 전진해 강화된 200 mm 기준에
39.07/42.67 mm 여유를 확보했고 평균 속도는 11.67/11.85 mm/s였다. phase별 220 mm
몸통 목표 보상도 구현했지만 PPO 미세조정은 전진 여유와 착지를 악화시켜 채택하지 않았다.
최대 착지력이 48.53/49.05 N으로 제한에 가까우므로 다음 단계는 속도 압축보다 물성·초기
오차 강건성 검증이 우선이다. 상세 결과는
[`docs/KHR3HV_V17_FORWARD_MARGIN_EXPERIMENT.md`](docs/KHR3HV_V17_FORWARD_MARGIN_EXPERIMENT.md)에 기록했다.

V18–V20에서는 같은 40개 복합 물성·초기 오차에서 V17 12/40, V19 25/40, V20
30/40으로 강건성을 높였다. V21은 unload/landing 2개 출력 head와 직전 스텝 메모리,
접촉 조건부 lift gate를 추가해 184,320 step 학습했지만 최고 학습본이 29/40에 그쳐
채택하지 않았다. 최종 V21 archive는 V20 동작과 512 step 전체가 수치적으로 동일한 초기
이식본이며, 실패 계보와 정면 비교 영상은
[`docs/KHR3HV_V21_PHASE_SPLIT_EXPERIMENT.md`](docs/KHR3HV_V21_PHASE_SPLIT_EXPERIMENT.md)에 기록했다.

V22에서는 V20 잔여 실패가 모두 첫 스텝에서만 생긴다는 사실을 확인하고, 800회 국소
counterfactual sweep으로 첫 lift의 지지측 hip-roll에 좌우 대칭 0.008 rad 보정을 찾았다.
V17/V20 정책과 42 mm 보폭은 그대로 유지하면서 기존 복합 40회가 30/40→40/40으로,
미사용 seed 300회는 228/300→280/300으로 개선됐다. 대표 회복·경계 회귀 영상과 전체
결과는
[`docs/KHR3HV_V22_COUNTERFACTUAL_STANCE_EXPERIMENT.md`](docs/KHR3HV_V22_COUNTERFACTUAL_STANCE_EXPERIMENT.md)에 기록했다.

하드웨어 검토용으로 배포 구성(V22)의 관절별 토크 프로파일을 500 Hz로 추출했다. 8스텝
20.48초 동안 최대 토크는 9.01/9.05 N·m로 24 N·m 한계의 38%였고 포화는 없었다. 부하는
hip-roll(RMS 4.1 N·m)에 집중되며 여유가 가장 적은 관절은 7 N·m 한계의 55%를 쓰는
ankle_roll이다. 최대 관절 속도는 13 rpm으로 정격의 33%에 그친다. 대신 AC 토크 에너지의
39–67%가 제어 주기와 같은 25 Hz에 몰려 있어 실기 이식 시 목표 보간이나 제어 주파수
상향 검토가 필요하다. 추출 방법, 관절별 표와 산출물은
[`docs/TORQUE_PROFILE_V22.md`](docs/TORQUE_PROFILE_V22.md)에 기록했다.

```bash
python3 -m pytest -q
python3 scripts/evaluate_khr3hv_v1.py --steps 500
python3 scripts/evaluate_single_support_v4.py --side right
python3 scripts/evaluate_single_support_v4.py --side left
python3 scripts/evaluate_two_step_v6.py --first-side left
python3 scripts/evaluate_two_step_v6.py --first-side right
python3 scripts/evaluate_four_step_v7.py --first-side left
python3 scripts/evaluate_four_step_v7.py --first-side right
python3 scripts/evaluate_eight_step_v8.py --first-side left
python3 scripts/evaluate_eight_step_v8.py --first-side right
python3 scripts/evaluate_fast_eight_step_v9.py --first-side left
python3 scripts/evaluate_fast_eight_step_v9.py --first-side right
python3 scripts/evaluate_soft_landing_v10.py --first-side left
python3 scripts/evaluate_soft_landing_v10.py --first-side right
python3 scripts/evaluate_robust_soft_landing_v11.py
python3 scripts/sweep_soft_landing_v11_asymmetry.py
python3 scripts/train_curriculum_soft_landing_v12.py
python3 scripts/evaluate_corrected_forward_v13.py --first-side left
python3 scripts/evaluate_corrected_forward_v13.py --first-side right
python3 scripts/search_dynamic_forward_v14_reference.py
python3 scripts/train_dynamic_forward_v14.py --timesteps 50000 --learning-rate 1e-5
python3 scripts/evaluate_dynamic_forward_v14.py --first-side left
python3 scripts/evaluate_dynamic_forward_v14.py --first-side right
python3 scripts/search_dynamic_forward_v15_reference.py
python3 scripts/train_dynamic_forward_v15.py --timesteps 50000 --learning-rate 1e-5
python3 scripts/evaluate_dynamic_forward_v15.py --first-side left
python3 scripts/evaluate_dynamic_forward_v15.py --first-side right
python3 scripts/search_dynamic_forward_v16_reference.py
python3 scripts/train_dynamic_forward_v16.py --timesteps 50000 --learning-rate 1e-5
python3 scripts/evaluate_dynamic_forward_v16.py --first-side left
python3 scripts/evaluate_dynamic_forward_v16.py --first-side right
python3 scripts/search_dynamic_forward_v17_stride.py
python3 scripts/train_forward_margin_v17.py --timesteps 50000 --learning-rate 1e-5
python3 scripts/evaluate_forward_margin_v17.py --first-side left
python3 scripts/evaluate_forward_margin_v17.py --first-side right
python3 scripts/train_phase_split_residual_v21.py
python3 scripts/calibrate_phase_split_residual_v21.py
python3 scripts/render_phase_split_residual_v21.py
python3 scripts/diagnose_counterfactual_v22.py
python3 scripts/sweep_counterfactual_v22.py --workers 4
python3 scripts/calibrate_stance_probe_v22.py --workers 4
python3 scripts/render_stance_probe_v22.py
python3 scripts/export_torque_profile_v22.py
```
