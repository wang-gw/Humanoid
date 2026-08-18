# KHR-3HV 방식 V18 강건성 검증

## 결론

V18은 새 정책을 학습한 버전이 아니라, 확정된 V17 정책과 보행 목표를 고정한 채 모델 오차를
가해 실패 경계를 찾는 검증 버전이다. 결론은 다음과 같다.

- 42 mm 명목 보폭과 200 mm 전진 기준은 충분한 여유를 유지했다.
- 단일요인 114회 중 89회(78.1%)가 전체 기준을 통과했다.
- 복합 오차 40회 중 12회(30.0%)만 통과했고, 좌·우 선행을 모두 통과한 조합은 20개 중
  4개였다.
- 모든 복합 실패는 50 N 착지 한계를 넘었다. 낙상과 전진거리 실패는 없었다.
- 따라서 V17을 더 빠르게 만들기 전에 착지 충격 강건화를 먼저 해야 한다.

## 고정한 것과 바꾼 것

V18에서 고정한 항목은 다음과 같다.

| 항목 | 고정값 |
|---|---:|
| 정책 | V17 최종 정책(실제로는 V16 정책과 byte-identical) |
| 명목 발 보폭 | 42 mm/step |
| 몸통 shaping 목표 | 27.5 mm/step, 총 220 mm |
| 성공 최소 전진거리 | 200 mm |
| 착지력 한계 | 50 N |
| step 주기 | 2.56 s |
| 전체 episode | 8 step, 20.48 s |

V18 환경은 위 조건을 바꾸지 않고 다음 오차만 추가한다.

- 바닥 및 sole 마찰계수 배율
- 전체 로봇 질량·관성 배율
- 좌우 다리 질량·관성 비대칭
- 초기 10개 관절 각도와 속도 오차
- swing/landing PD gain의 공통 배율
- residual action의 control-step 지연

좌우 질량 비대칭 `a`는 왼쪽 다리에 `1+a`, 오른쪽 다리에 `1-a`를 적용한다. 제어 1 step은
40 ms다. 각도와 속도 오차는 표시한 값을 표준편차로 하는 seed 고정 정규분포다.

## 단일요인 결과

총 114회에서 89회가 통과했다. 실패 25회는 모두 착지력 초과였고 그중 5회는 swing unload
기준도 함께 깨졌다. 전진거리는 모든 경우 234.84 mm 이상이었다.

### 양쪽 선행이 모두 통과한 이산 검증점

아래 범위는 수학적으로 보장된 연속 허용구간이 아니라, 실제로 시험한 점 중 좌·우 선행이
모두 통과한 구간이다.

| 변수 | 양쪽 통과 검증점/구간 | 처음 확인된 경계 실패 |
|---|---|---|
| 전체 질량 배율 | 0.98, 0.99, 1.00 | 0.97에서 한쪽, 1.01에서 한쪽 |
| 좌우 질량 비대칭 | -1.0%, -0.75%, -0.5%, 0% | -1.25%에서 한쪽, +0.5%에서 한쪽 |
| motor gain 배율 | 0.99, 1.00, 1.01, 1.02 | 0.98에서 한쪽, 1.03에서 한쪽 |
| 제어 지연 | 0, 40, 80 ms | 80 ms까지 실패 없음 |
| 초기 관절각 오차 | 0.0035 rad(약 0.20°)까지 10/10 | 0.00525 rad에서 9/10 |
| 초기 관절속도 오차 | 0.015 rad/s까지 10/10 | 시험 범위 안 실패 없음 |

마찰은 단조롭지 않았다. 배율 0.75, 0.80, 0.85, 0.95, 1.00, 1.05에서는 양쪽 모두
통과했지만 0.90과 1.10에서는 한쪽이 실패했고, 1.15 이상에서는 양쪽이 실패했다. 이는
마찰이 커질수록 단순히 충격이 커지는 문제가 아니라 발 접촉 시점과 기준 궤적의 위상이
재정렬되는 문제임을 보여준다. 따라서 마찰의 안전영역을 하나의 연속 구간으로 해석하면 안
된다.

## 복합 오차 결과

20개 parameter sample에 좌·우 선행을 각각 적용해 총 40회를 평가했다.

| 오차 | sampling 범위 |
|---|---:|
| 마찰 배율 | 0.80–1.20 |
| 전체 질량 배율 | 0.98–1.02 |
| 좌우 질량 비대칭 | -1.0–+1.0% |
| 초기 관절각 오차 표준편차 | 0.0035 rad |
| 초기 관절속도 오차 표준편차 | 0.010 rad/s |
| motor gain 배율 | 0.95–1.05 |
| 제어 지연 | 0–80 ms |

결과는 다음과 같다.

| 지표 | 결과 |
|---|---:|
| 전체 성공 | 12/40 (30.0%) |
| 양쪽 모두 성공한 sample | 4/20 (20.0%) |
| 낙상 | 0/40 |
| 착지력 실패 | 28/40 |
| swing unload 동시 실패 | 10/40 |
| double support 동시 실패 | 3/40 |
| 최소 전진거리 | 236.16 mm |
| 최대 착지력 | 82.64 N |

전진거리 최솟값도 200 mm보다 36.16 mm 앞섰다. 즉 V17의 42 mm 보폭 선택은 유지할 수
있다. 실패한 28회 모두 착지 충격 기준을 넘었으므로 다음 버전에서 보폭이나 몸통 진행 목표를
먼저 줄일 근거는 없다.

## 대표 영상

비교 영상은 왼쪽부터 복합 오차 통과, 질량 +1% 경계 실패, 큰 복합 실패 순서다.

- [3개 조건 동시 비교](../results/khr3hv_v18_robustness/v18_success_boundary_severe_comparison.mp4)
- [복합 오차 통과: 246.76 mm, 49.84 N](../results/khr3hv_v18_robustness/v18_combined_success_right_seed101.mp4)
- [질량 +1% 경계 실패: 244.27 mm, 50.23 N](../results/khr3hv_v18_robustness/v18_boundary_mass101_failure_right.mp4)
- [복합 오차 큰 실패: 239.70 mm, 82.64 N](../results/khr3hv_v18_robustness/v18_combined_severe_failure_right_seed106.mp4)

정량 원본은 다음 파일에 있다.

- [단일요인 요약](../results/khr3hv_v18_robustness/single_factor_summary.json)
- [단일요인 전체 결과](../results/khr3hv_v18_robustness/single_factor_results.json)
- [경계 탐색 요약](../results/khr3hv_v18_robustness/boundary_summary.json)
- [복합 오차 요약](../results/khr3hv_v18_robustness/combined_summary.json)
- [복합 오차 전체 결과](../results/khr3hv_v18_robustness/combined_results.json)
- [V18 환경 코드](../humanoidv2/robust_forward_margin_v18_env.py)
- [평가 코드](../scripts/evaluate_robust_forward_margin_v18.py)

## 재현 방법

```bash
PYTHONPATH=. python3 scripts/evaluate_robust_forward_margin_v18.py
PYTHONPATH=. python3 scripts/evaluate_robust_forward_margin_v18.py --boundary
PYTHONPATH=. python3 scripts/evaluate_robust_forward_margin_v18.py --combined
```

개별 영상은 같은 스크립트에 `--video`와 원하는 오차 인자를 전달해 생성한다.

## 다음 단계: V19

V19는 42 mm 보폭, 2.56 s/step, 200 mm 성공 기준을 유지해야 한다. 최적화 대상은 전진이
아니라 착지 phase의 충격 민감도다.

1. V17 정책을 보존한 채 landing phase에만 작용하는 작은 residual 보정기를 둔다.
2. 착지 직전 발 수직속도와 접촉 예상시간을 관측/보상에 명시한다.
3. V18 복합 오차 범위로 domain randomization하되 쉬운 범위부터 단계적으로 넓힌다.
4. nominal 좌·우 성공을 먼저 유지하고, 이후 복합 40회 성공률과 최악 착지력을 기준으로
   checkpoint를 선택한다.
5. 복합 오차에서 양쪽 모두 통과율이 충분히 올라가기 전에는 주기를 1.92 s로 줄이지 않는다.

V19의 우선 목표는 복합 오차 40회 성공률을 30%에서 최소 80%로 높이고, 최악 착지력을
50 N 아래로 만드는 것이다. 이 두 조건을 동시에 만족하지 못하면 속도 향상 단계로 넘어가지
않는다.
