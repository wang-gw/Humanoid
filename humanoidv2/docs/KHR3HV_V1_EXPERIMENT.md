# KHR-3HV 방식 보행 강화학습 V1 실험 기록

- 실행일: 2026-08-04 (Asia/Seoul)
- 대상: CHIRO Humanoid v2, 자유 베이스 + 다리 10관절
- 판정: **시뮬레이션과 낙상 방지는 성공, 목표 속도의 전진 보행은 실패**
- 원 논문: [Reinforcement Learning of Bipedal Walking Using a Simple Reference Motion](https://doi.org/10.3390/app14051803)

## 1. 목적과 범위

Open Duck Playground나 Berkeley Humanoid의 환경·보상·물리 파라미터를 섞지 않고,
KHR-3HV 논문에 공개된 방법을 현재 로봇에 직접 적용하는 첫 기준 실험이다. 원 논문은
Webots를 사용했지만 KHR-3HV용 환경 소스와 완전한 Webots 모델을 공개하지 않았고, 현재
로봇 자산은 MuJoCo MJCF이므로 시뮬레이터만 MuJoCo를 사용했다. 로봇의 질량, 관성,
관절 위치와 CAD는 기존 `URDF_F_v2_footprint_contact.xml`을 그대로 출발점으로 삼았다.

이번 V1은 논문의 고정 기준궤적 실험에 해당한다. Bayesian optimization, domain
randomization 및 실제 로봇 전이는 포함하지 않았다.

## 2. 논문에서 직접 적용한 요소

| 항목 | V1 구현 |
|---|---|
| 제어 관절 | 양쪽 다리 10관절 |
| 물리/정책 주기 | MuJoCo 500 Hz, 정책 및 IK 기준 25 Hz |
| 보행 주기 | 50 정책 단계 = 2.0초 |
| 기준 운동 | 논문의 세 사인파 기반 좌우 무게이동 및 교대 발 들기 |
| 고정 파라미터 | `w=20 mm`, `delta_w=5 mm`, `h=25 mm`, `delta_h=5 mm` |
| IK | 매 보행 위상별 30회 제한 damped Gauss-Newton, update gain 0.5 |
| 행동 | 10차원 `[-1, 1]`, LPF 계수 0.2, IK 관절각에 잔차로 더함 |
| 관측 | 관절각 10 + 선형가속도 3 + 각속도 3의 최근 5프레임 + 위상 sin/cos = 82 |
| 내부 제어 | `Kp=18.5`, `Ki=0`, `Kd=0.009`의 PD 토크 제어 |
| 보상 | 자세 0.10 + 속도 0.75 + 대칭 0.10 + 생존 0.05 + 토크 페널티 |
| PPO | actor/critic 각각 64-64, Tanh, SB3 PPO 기본 continuous-control 설정 |
| 에피소드 | 최대 500단계 = 20초, 몸통 높이 저하 시 종료 |

논문 본문의 상태 정의를 합하면 82차원이지만 네트워크 그림에는 84로 표기되어 있다.
재현 가능한 수식 정의를 우선해 82차원을 사용했다. PPO 구현은
[Stable-Baselines3 PPO 문서](https://stable-baselines3.readthedocs.io/en/master/modules/ppo.html)의
연속 제어 기본값(`3e-4`, `2048`, `64`, `10`, `0.99`, `0.95`, `0.2`)을 명시적으로 고정했다.

## 3. 현재 로봇 때문에 필요한 최소 변경

1. 로봇 좌표계에서 `+y`가 전진, `x`가 좌우이므로 논문의 전진 `x`와 좌우 `y`를 교환했다.
2. AK45-36과 AK45-10의 속도 차이를 잔차 행동 크기에 반영했고 피크 토크를 각각
   24 Nm와 7 Nm로 제한했다. PID 이득은 논문 값을 유지했다.
3. 원본 발 CAD 메시가 의도한 발바닥 패드보다 약 46 mm 아래까지 내려와 초기 자세에서
   바닥과 깊게 겹쳤다. 학습 중 CAD 메시는 시각화 전용으로 바꾸고 기존 8개 sole box만
   접촉에 사용했다. 원본 XML 파일 자체는 변경하지 않았다.
4. KHR의 20 mm 초기 COM 하강은 현재 로봇의 비대칭 hip pitch 관절 제한과 발 들기 여유를
   동시에 만족하지 못해 약 10 mm의 얕은 웅크림으로 시작했다.
5. 논문의 실제 IMU 대신 자유 베이스 속도 차분 가속도와 베이스 각속도를 사용했다.

## 4. 학습 설정

- 난수 시드: 7
- 병렬 환경: 4개
- 요청 학습량: 500,000 environment steps
- 실제 학습량: 507,904 steps (PPO rollout 경계 때문에 초과)
- 실행 시간: 1,177.26초 (19분 37초)
- 수집 에피소드: 1,274개
- 최종 최근 20 에피소드 평균 보상: 168.716
- 최종 최근 20 에피소드 평균 길이: 500.0/500
- 평가: 학습 노이즈 없이 deterministic action, 500단계

## 5. 결과

| 평가 | 생존 | 총 보상 | 20초 순전진 | 평균 전진속도 | 최종 몸통 높이 |
|---|---:|---:|---:|---:|---:|
| 기준 IK 궤적만 | 500/500 | 17.299 | 0.0376 m | 0.00186 m/s | 0.3158 m |
| 최종 PPO 정책 | 500/500 | 195.767 | 0.0195 m | 0.00111 m/s | 0.3320 m |

학습 정책은 20초 동안 자세를 유지했고 기준 대비 자세·대칭·토크 항을 개선했다. 그러나
목표 `0.15 m/s`의 0.74% 수준에 불과하므로 전진 보행 성공으로 볼 수 없다. 영상상 결과는
안정적인 제자리 스텝/균형 동작에 가깝다.

평균 보상 구성은 다음과 같다.

| 평가 | 자세 | 속도 | 대칭 | 토크 페널티 |
|---|---:|---:|---:|---:|
| 기준 IK 궤적만 | 0.0779 | 0.00276 | 0.7932 | -0.1046 |
| 최종 PPO 정책 | 0.6417 | 0.34834 | 0.9539 | -0.0793 |

체크포인트 비교에서도 학습 보상은 계속 증가했지만 전진거리는 증가하지 않았다. 5만 단계
정책의 순전진은 3.77 cm, 최종 정책은 1.95 cm였다. 즉 더 오래 학습하면 해결되는 형태라기보다
보상식의 허점을 이용한 것으로 판단한다.

### 보행 실패의 직접 원인

논문 속도 보상은 전진속도가 양수이고 자세 보상이 0.8보다 크면 지수 보상을 준다. 정지에
아주 가까운 속도에서도 부호만 양수이면 속도항은 대략 `exp(-0.675)=0.509`가 된다. 따라서
정책은 0.15 m/s에 접근하는 것보다 작은 양의 흔들림을 만들면서 자세와 대칭을 유지하는
안전한 해를 선택했다. 실제 최종 평균 속도가 0.0011 m/s인데 속도 보상 평균이 0.348인 것이
이 현상을 뒷받침한다.

## 6. 산출물

- 학습 정책: [`results/khr3hv_v1/ppo_khr3hv_final.zip`](../results/khr3hv_v1/ppo_khr3hv_final.zip)
- 학습 정책 영상: [`results/khr3hv_v1/trained_policy.mp4`](../results/khr3hv_v1/trained_policy.mp4)
- 기준궤적 영상: [`results/khr3hv_v1/reference_only.mp4`](../results/khr3hv_v1/reference_only.mp4)
- 평가 수치: `*_summary.json`, 프레임별 값: `*_trajectory.json`
- 체크포인트 비교: [`checkpoint_comparison.json`](../results/khr3hv_v1/checkpoint_comparison.json)
- 5만 단계별 정책: `results/khr3hv_v1/checkpoints/`

두 MP4는 각각 640×480, 25 fps, 501프레임(초기 프레임 포함)이며 디코딩 검사를 통과했다.

## 7. 물리 모델의 현재 한계

- 기존 MJCF의 질량·관성을 재산정하지 않고 사용했으므로 CAD/실물과의 일치 검증이 필요하다.
- CAD 메시 충돌을 끄고 발바닥 패드만 접촉시켰다. 몸통·다리의 바닥 충돌과 self-collision은
  없으며, 낙상은 몸통 높이로 판정한다.
- Webots와 MuJoCo의 접촉, 모터, 마찰 모델은 같지 않다. 따라서 이것은 논문 알고리즘의
  현재 로봇 적용 실험이지 KHR-3HV 결과의 bit-for-bit 재현이 아니다.
- 단일 seed 결과이므로 분산과 재현 성공률은 아직 측정하지 않았다.

## 8. V2 권고안

KHR 방식의 구조(IK 기준 + 잔차 PPO)는 유지하되 다음 순서가 합리적이다.

1. 성공 조건을 `20초 생존 + 평균속도 0.12~0.18 m/s + 순전진 2.4 m 이상`으로 별도 정의한다.
2. 속도 보상을 `exp(-K(v_cmd-v)^2)`만 쓰거나, 0.05 m/s 이하에는 보상을 주지 않도록 바꿔
   제자리 흔들림 해를 제거한다. 이는 논문식에서 의도적으로 벗어나는 변경이므로 V2로 분리한다.
3. 몸통·허벅지·종아리의 단순 convex collision과 올바른 발바닥 높이를 MJCF에 명시한다.
4. CAD 질량, 링크 COM, 관성, 감속비, 토크-속도 곡선을 실측/도면과 대조한다.
5. 위 조건을 고정한 뒤 3개 이상의 seed와 기준궤적 파라미터 탐색을 실행한다.
6. 시뮬레이션 보행이 확인된 다음에만 domain randomization과 실제 로봇 제한을 추가한다.

이 순서는 외부 로봇의 환경이나 물리를 가져오는 것이 아니라, 이번 실험에서 확인된 현재
로봇 모델과 보상식의 결함을 하나씩 제거하는 방향이다.

## 9. 재실행

```bash
cd /home/king0519/projects/Humanoidv2
python3 -m pip install -r requirements.txt
python3 -m pytest -q
python3 scripts/train_khr3hv_v1.py --timesteps 500000 --n-envs 4
python3 scripts/evaluate_khr3hv_v1.py --steps 500
python3 scripts/evaluate_khr3hv_v1.py --reference-only --steps 500
python3 scripts/compare_checkpoints.py
```
