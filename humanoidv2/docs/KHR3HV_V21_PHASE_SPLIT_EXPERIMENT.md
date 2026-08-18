# KHR-3HV 방식 V21 위상 분리 보정기 실험

## 결론

V21은 V20의 단일 10축 보정 출력을 `unload/advance`용 10축과 `land/settle`용 10축으로
분리하고, 직전 스텝 결과 4개를 관측에 추가했다. V17 기본 정책, 42 mm 보폭,
2.56초/step, 200 mm 전진 기준 및 50 N 착지 제한은 바꾸지 않았다.

구조와 정책 이식은 정상 작동했지만 강건성 개선에는 실패했다. 184,320 step 학습 후
V18과 동일한 복합 오차 40회에서 체크포인트 성공 수는 27, 26, 25회로 감소했다. 게이트
강도를 별도로 재보정해도 가장 좋은 학습본은 29/40이어서 V20의 30/40을 넘지 못했다.

따라서 최종 V21 archive는 학습본이 아니라 **V20 동작을 정확히 보존하는 초기 이식본**을
채택했다. 배포 시 조건부 lift gate는 0이다. 최종 수치는 V20과 동일한 30/40(75%), 양쪽
성공 12/20, 낙상 0이다. V21은 성공한 새 제어기가 아니라, 실패 원인을 보존한 구조 실험이다.

## 변경한 구조

### 1. 20차원 위상 분리 출력

정책 출력은 다음 두 헤드로 나뉜다.

| 출력 | 차원 | 적용 구간 |
|---|---:|---|
| unload head | 10 | lift, hold, advance 전반 |
| landing head | 10 | land, settle |

advance 진행률 50%부터 두 헤드를 smoothstep으로 혼합하고, land 진입 시 landing head만
사용한다. 합성된 10축 보정에만 기존 V20 gate와 관절당 ±0.025 rad 제한을 적용한다.
따라서 기본 로봇 구동 차원은 계속 10이며, 42 mm 발 목표도 바뀌지 않는다.

### 2. 직전 스텝 메모리

V20의 90개 관측 뒤에 다음 4개를 붙여 총 94개 관측을 사용한다.

| 메모리 | 정규화 |
|---|---:|
| 직전 스텝 최대 착지력 | 100 N |
| advance 중 5 N 미만 swing-force 비율 | 0–1 |
| settle 중 양발 접촉 비율 | 0–1 |
| 실제 스텝 길이 | 42 mm |

각 스텝의 마지막 settle sample에서만 갱신되므로 현재 스텝의 미완성 통계가 다음 판단에
섞이지 않는다.

### 3. 접촉 조건부 lift gate

lift 중 swing foot force가 5 N을 넘을 때만 보정할 수 있게 했다. force 5–15 N과 lift
진행률 25–100%를 각각 smoothstep으로 변환하고, 두 값을 곱해 gate를 만든다. curriculum
상한은 0 → 0.20 → 0.35였다. 최대 0.35에서도 lift 관절 보정 한계는
`0.025 × 0.35 = 0.00875 rad`이다.

## V20에서 V21로의 무손실 이식

V20의 첫 은닉층 90개 입력 weight를 그대로 복사하고 새 메모리 4개 column은 0으로
초기화했다. V20의 10개 출력 weight, bias 및 log standard deviation은 두 V21 헤드에
동일하게 복제했다.

조건부 lift gate를 0으로 둔 nominal 512 step 비교 결과는 다음과 같다.

| 비교 항목 | 최대 차이 |
|---|---:|
| V20 action과 V21 unload head | 0 |
| V20 action과 V21 landing head | 0 |
| 두 V21 head | 0 |
| qpos | 0 |
| 전진거리 | 0 m |
| 최대 착지력 | 0 N |

따라서 이후 성능 차이는 네트워크 크기 변경 오류가 아니라 학습과 lift gate 개입에서 생긴다.

## 학습

PPO 설정은 learning rate 5e-5, 4개 병렬 환경, 1024 rollout step, batch 256,
7 epochs, clip 0.10, target KL 0.02를 사용했다.

| 단계 | domain | lift gate 상한 | 누적 실제 step |
|---|---|---:|---:|
| 0 | 전체 균등 범위 | 0 | 40,960 |
| 1 | 고마찰 집중 범위 | 0.20 | 102,400 |
| 2 | 전체 균등 범위 | 0.35 | 184,320 |

학습 중 모든 최근 episode 길이는 512 step으로 낙상은 발생하지 않았다. 그러나 완주가
50 N 제한 통과를 의미하지 않으므로 checkpoint는 별도 40회 검증으로 선택했다.

## 0.35 gate 공통 평가

| 후보 | nominal | 복합 성공 | 양쪽 성공 sample | 최악 착지력 | 최소 전진 |
|---|---:|---:|---:|---:|---:|
| 초기 이식 | 2/2 | 29/40 | 11/20 | 78.87 N | 247.54 mm |
| stage 0 | 2/2 | 27/40 | 10/20 | 79.95 N | 247.18 mm |
| stage 1 | 2/2 | 26/40 | 10/20 | 79.58 N | 248.90 mm |
| stage 2 | 2/2 | 25/40 | 8/20 | 79.99 N | 252.19 mm |

초기 이식본도 gate 0.35에서는 V20보다 한 번 적게 통과했다. 즉 학습 전부터 조건부 lift
개입이 자동 개선이 아님을 확인했다.

## 배포 gate 재보정

| 후보 | lift gate | 복합 성공 | 양쪽 성공 sample | 최악 착지력 |
|---|---:|---:|---:|---:|
| 초기 이식 | 0 | **30/40** | **12/20** | 78.63 N |
| stage 0 | 0 | 29/40 | 12/20 | **73.59 N** |
| stage 0 | 0.10 | 28/40 | 11/20 | 76.76 N |
| stage 1 | 0.10 | 29/40 | 12/20 | 77.92 N |
| stage 1 | 0.20 | 28/40 | 11/20 | 78.42 N |
| stage 2 | 0.10 | 29/40 | 12/20 | 77.72 N |
| stage 2 | 0.20 | 27/40 | 10/20 | 77.34 N |

stage 0은 최악 충격을 5.05 N 낮췄지만 50 N 통과 수는 한 번 줄였다. 최종 선택 기준은
nominal 2/2 이후 통과 수를 최우선으로 하므로 초기 이식+gate 0을 채택했다.

## 최종 선택본

| 지표 | V20 | V21 선택본 |
|---|---:|---:|
| 복합 성공 | 30/40 | 30/40 |
| 양쪽 성공 sample | 12/20 | 12/20 |
| 낙상 | 0 | 0 |
| impact 실패 | 10 | 10 |
| swing unload 실패 | 8 | 8 |
| double support 실패 | 0 | 0 |
| step length 실패 | 1 | 1 |
| 최악 착지력 | 78.63 N | 78.63 N |
| 최소 전진거리 | 246.54 mm | 246.54 mm |

nominal은 왼쪽 선행 265.57 mm/46.90 N, 오른쪽 선행 268.10 mm/45.87 N으로 모두
성공했다.

## 대표 사례와 실패 해석

seed 106 우측에서 stage 0 학습본은 충격을 78.63→73.33 N으로 줄였다. 그러나 swing
unload 비율은 0.1, 최소 보폭은 16.50 mm이고 양발 접촉 비율도 1.0→0.8로 떨어져 여전히
실패했다. 충격만 낮추는 보정이 발 분리와 스텝 안정화를 회복하지 못했다.

seed 120 좌측에서는 선택본이 49.83 N으로 통과했지만 stage 0은 50.14 N으로 경계선을
넘었다. PPO의 연속 충격 벌점은 일부 큰 peak를 낮췄지만 `8개 스텝 모두 50 N 이하`라는
hard criterion과 정확히 정렬되지 않았다.

직전 스텝 메모리는 다음 스텝부터만 유효하므로 첫 실패를 예방할 수 없고, 집계값 4개만으로
어느 관절·어느 순간을 고쳐야 하는지 알기 어렵다. 또한 하나의 episode reward에 unload,
접촉 회복, 충격 및 전진을 함께 넣으면 성공 경계 부근에서 서로 교환되는 현상이 생겼다.

## 영상과 결과

- [선택본 nominal 좌·우 정면 비교](../results/khr3hv_v21_phase_split/v21_selected_nominal_left_right_front.mp4)
- [seed 106 선택본/학습본 비교](../results/khr3hv_v21_phase_split/v21_seed106_right_selected_vs_learned_front.mp4)
- [seed 120 경계 성공/실패 비교](../results/khr3hv_v21_phase_split/v21_seed120_left_selected_vs_learned_front.mp4)
- [무손실 이식 검증](../results/khr3hv_v21_phase_split/transfer_verification.json)
- [학습 checkpoint 결과](../results/khr3hv_v21_phase_split/training_summary.json)
- [gate 재보정 결과](../results/khr3hv_v21_phase_split/calibration_summary.json)
- [복합 평가 원본](../results/khr3hv_v21_phase_split/calibration_results.json)
- [배포 설정](../results/khr3hv_v21_phase_split/deployment_config.json)
- [최종 V21 archive](../results/khr3hv_v21_phase_split/ppo_phase_split_v21_final.zip)
- [V21 환경](../humanoidv2/phase_split_residual_v21_env.py)
- [학습 코드](../scripts/train_phase_split_residual_v21.py)
- [gate 보정 코드](../scripts/calibrate_phase_split_residual_v21.py)
- [영상 코드](../scripts/render_phase_split_residual_v21.py)

## 다음 단계

V22에서는 PPO 학습량을 더 늘리지 않는다. 먼저 실패한 10개 실행에서 각 스텝·관절·위상별
작은 보정 sweep을 실행해 50 N, unload 0.9, double support 0.9, 보폭 20 mm를 동시에
개선하는 보정 방향이 실제로 존재하는지 확인한다.

1. seed 106 우측을 포함한 실패 실행을 스텝 단위로 재생한다.
2. lift/advance/land 구간별로 관절 하나씩 제한된 counterfactual offset을 적용한다.
3. 한 지표를 개선하면서 이미 통과한 지표를 깨는 offset은 폐기한다.
4. 유효한 offset이 존재할 때만 그 결과를 teacher label로 사용한다.
5. 정책은 전체 episode PPO가 아니라 실패 순간의 제한된 보정량을 모방하도록 학습한다.
6. 동일 40회에서 V20의 30/40을 넘어야만 새 정책으로 채택한다.
