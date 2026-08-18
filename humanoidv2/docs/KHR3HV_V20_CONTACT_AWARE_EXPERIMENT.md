# KHR-3HV 방식 V20 접촉 인지 보정기 실험

## 결론

V20은 V19의 착지 보정기를 접촉 인지형으로 확장했다. V17 기본 정책은 계속 고정했고,
42 mm 보폭, 2.56초/step, 200 mm 전진 및 50 N 착지 기준도 바꾸지 않았다.

V18과 동일한 복합 오차 40회에서 V20은 **30/40(75.0%)**를 통과했다. V17의 12/40,
V19의 25/40보다 개선됐고 양쪽 선행을 모두 통과한 parameter sample도 4→10→12개로
늘었다. nominal 좌·우는 모두 성공했으며 낙상은 없었다.

목표 32/40(80%)에는 2회 미달했다. 추가 안정화 학습을 80k 더 진행했지만 성공 수가
30→28→26으로 후퇴해, 최종 정책은 안정화 80k checkpoint로 되돌렸다.

## V19에서 변경한 구조

### 1. 기본 정책 관측과 보정기 관측 분리

V17 정책은 기존 82개 관측만 계속 받는다. V20 보정기는 여기에 8개 접촉 정보를 붙인
90개 관측을 사용한다.

| 추가 관측 | 정규화 |
|---|---:|
| 좌·우 발 접촉력 | 100 N |
| 좌·우 sole 높이 | 0.05 m |
| 좌·우 sole 수직속도 | 0.5 m/s |
| 현재 correction gate | 0–1 |
| 현재 phase 진행률 | 0–1 |

V19의 82입력 네트워크를 90입력으로 확장하면서 기존 weight는 그대로 복사했고 새 8개
입력 column만 0으로 초기화했다.

`early_gate_blend=0`에서 이식 직후 V19와 V20을 512 step 비교한 결과는 다음과 같다.

- 최대 정책 action 차이: 0
- 최대 qpos 차이: 0
- 최종 전진거리 차이: 0
- 최대 착지력 차이: 0

따라서 네트워크 확장 자체는 V19 동작을 정확히 보존한다.

### 2. 보정 시점 앞당김

V19는 advance 후반부터 보정했다. V20의 조기 gate는 hold 후반부터 켜지고 advance 전체에
작용한다. curriculum에서 조기 gate를 0%, 50%, 100%로 열었으나 최종 보정 평가 결과
75% blend가 양쪽 동시 성공 조합을 더 잘 보존했다.

최종 배치는 다음과 같다.

- lift: 보정 없음
- hold 후반: 조기 gate의 75%
- advance 전반: 조기 gate의 75%
- advance 후반: 기존 V19 gate와 조기 gate의 혼합
- land: 100%
- settle: 기존과 동일하게 1→0
- 관절 목표 보정 한계: 각 관절 ±0.025 rad(약 ±1.43°)

최종 평가에서 실제 최대 보정은 0.01360 rad(약 0.78°)였다.

### 3. 보상 추가

V19의 착지력·하강속도·보정 크기 보상에 다음 항목을 추가했다.

- hold/advance에서 swing foot force가 5 N을 넘는 unload 실패 벌점
- settle에서 한쪽 접촉력이 10 N 아래로 내려가는 contact recovery 벌점

## 학습 과정

1. V19 weight 이식 및 동작 보존 확인
2. stage 0: 기존 gate, 중간 domain, 32,768 step
3. stage 1: 조기 gate 50%, 전체 domain, 누적 81,920 step
4. stage 2: 조기 gate 100%, 고마찰 집중, 누적 163,840 step
5. gate 75%, 전체 균등 domain 안정화 80,000 step
6. 추가 80,000 step은 성능이 후퇴해 폐기

선택된 V20 정책은 총 243,840 step 학습 상태다. 전체로는 323,840 step까지 탐색했지만
마지막 80k는 최종 정책에 포함하지 않았다.

### checkpoint 결과

| 후보 | 복합 성공 | 양쪽 성공 sample | 최악 착지력 |
|---|---:|---:|---:|
| V20 stage2, gate 75% | 26/40 | 9/20 | 74.67 N |
| 안정화 +40k | 27/40 | 10/20 | 77.02 N |
| 안정화 +80k | **30/40** | **12/20** | 78.63 N |
| 안정화 +120k | 28/40 | 10/20 | 72.92 N |
| 안정화 +160k | 26/40 | 10/20 | 73.60 N |

추가 학습은 최악 충격을 낮췄지만 경계 조건의 통과 수를 줄였다. 이번 선택 기준은 nominal
양쪽 성공 이후 전체 성공 수와 양쪽 동시 성공 sample 수를 우선했으므로 +80k를 채택했다.

## 최종 비교

| 지표 | V17 | V19 | V20 |
|---|---:|---:|---:|
| 복합 성공 | 12/40 (30.0%) | 25/40 (62.5%) | **30/40 (75.0%)** |
| 양쪽 모두 성공한 sample | 4/20 | 10/20 | **12/20** |
| 낙상 | 0 | 0 | 0 |
| impact 실패 | 28 | 12 | **10** |
| swing unload 실패 | 10 | 11 | **8** |
| double support 실패 | 3 | 3 | **0** |
| step length 실패 | 0 | 0 | 1 |
| 최소 전진거리 | 236.16 mm | 247.22 mm | **246.54 mm** |
| 최악 착지력 | 82.64 N | 83.60 N | **78.63 N** |

V20은 성공률뿐 아니라 V19에서 개선하지 못했던 최악 충격과 double-support 실패를 줄였다.
다만 seed 106 우측처럼 고마찰에서 swing unload가 깨지면 보폭도 16.33 mm로 줄고 착지력이
78.63 N까지 올라간다.

## nominal 결과

| 선행 발 | 전진거리 | 착지력 | 최소 보폭 | 결과 |
|---|---:|---:|---:|---|
| 왼쪽 | 265.57 mm | 46.90 N | 33.54 mm | 성공 |
| 오른쪽 | 268.10 mm | 45.87 N | 31.09 mm | 성공 |

actuator torque saturation은 두 경우 모두 0이었다.

## 대표 개선 사례

seed 113 우측 선행은 같은 물리 조건에서 다음과 같이 개선됐다.

| 제어기 | 전진거리 | 착지력 | 결과 |
|---|---:|---:|---|
| V19 | 256.02 mm | 56.91 N | 실패 |
| V20 | 256.68 mm | 49.46 N | 성공 |

## 영상과 결과

- [V19/V20 동일 조건 전후 비교](../results/khr3hv_v20_contact_aware/v20_before_after_seed113_comparison.mp4)
- [V20 nominal 좌·우 비교](../results/khr3hv_v20_contact_aware/v20_nominal_left_right_comparison.mp4)
- [V20 잔여 최악 실패](../results/khr3hv_v20_contact_aware/v20_remaining_failure_seed106_right.mp4)
- [V17/V19/V20 최종 요약](../results/khr3hv_v20_contact_aware/robustness_summary.json)
- [V20 복합 40회 원본](../results/khr3hv_v20_contact_aware/robustness_results.json)
- [초기 curriculum 학습](../results/khr3hv_v20_contact_aware/training_summary.json)
- [전체 안정화 checkpoint 비교](../results/khr3hv_v20_contact_aware/stabilization_all_summary.json)
- [배치 설정](../results/khr3hv_v20_contact_aware/deployment_config.json)
- [최종 V20 정책](../results/khr3hv_v20_contact_aware/ppo_contact_aware_v20_final.zip)
- [V20 환경](../humanoidv2/contact_aware_residual_v20_env.py)
- [학습 코드](../scripts/train_contact_aware_residual_v20.py)
- [안정화 학습 코드](../scripts/continue_contact_aware_residual_v20.py)
- [평가 코드](../scripts/evaluate_contact_aware_residual_v20.py)

## 다음 단계

V20은 목표 80%에 근접했지만 같은 네트워크를 더 학습하면 성공률이 후퇴했다. 다음 단계는
학습량을 늘리는 것이 아니라 남은 10개 실패의 구조를 분리해야 한다.

1. unload 보정과 landing 보정을 하나의 10축 출력으로 공유하지 않고 phase별 head로 분리한다.
2. step별 최대 충격과 unload 실패 이력을 recurrent state 또는 별도 저차원 memory로 전달한다.
3. 고마찰에서 발이 충분히 떨어지지 않는 경우에만 lift/hold 높이 보정을 허용한다.
4. seed 106처럼 보폭까지 무너지는 조건에는 보정 한계를 착지 관절각이 아니라 swing 높이·
   진행 목표 단위로 제한한다.
5. 동일 40회에서 32/40 이상과 최악 착지력 감소를 동시에 만족해야 속도 압축을 재개한다.
