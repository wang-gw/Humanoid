# KHR-3HV 방식 V19 착지 보정기 실험

## 결론

V19는 V17 정책을 고정하고, 착지 직전에만 작동하는 작은 관절 목표 보정기를 별도로 학습한
버전이다. V18과 동일한 복합 오차 40회에서 성공률을 **30.0%에서 62.5%로 높였다.** 전진
최솟값은 247.22 mm였고 낙상은 없었다. 따라서 제한 보정기 구조가 유효하다는 것은
확인됐다.

그러나 목표였던 80%에는 도달하지 못했고 최악 착지력도 82.64 N에서 83.60 N으로 개선되지
않았다. V19는 성공한 최종 강건화 버전이 아니라, nominal 보행을 보존하면서 중간 난이도
오차를 상당 부분 복구한 첫 번째 제한 보정기 버전이다.

## 제어 구조

V17이 전체 보행을 담당하고 V19는 다음 구간에만 관절 목표 보정을 더한다.

```text
V17 관측 ──> 고정 V17 PPO ──> 기존 residual action ──┐
                                                    ├─> 관절 목표 ─> PD 제어
V17 관측 ──> V19 PPO ──> gate × ±0.025 rad 보정 ─────┘
```

gate는 다음과 같다.

- lift/hold 및 advance 전반부: 0
- advance 후반 0.2초: smoothstep으로 0→1
- land: 1
- settle: smoothstep으로 1→0

따라서 gate가 0인 구간에는 V19가 어떤 출력을 내더라도 V17 관절 목표가 정확히 유지된다.
보정 한계는 각 관절당 ±0.025 rad(약 ±1.43°)다. 최종 평가에서 실제 사용한 최대 보정은
0.01130 rad(약 0.65°)였다.

## nominal 보존 검사

학습 전에 V19 보정 출력을 0으로 고정하고 V17과 512 control-step 전체를 비교했다.

| 비교값 | 좌측 선행 | 우측 선행 |
|---|---:|---:|
| 최대 qpos 차이 | 0 | 0 |
| 최대 observation 차이 | 0 | 0 |
| 최종 전진거리 차이 | 0 | 0 |
| 최대 착지력 차이 | 0 | 0 |

즉 V19 wrapper 자체는 보정이 없을 때 V17과 수치적으로 동일하다.

최종 학습된 V19 nominal 결과는 다음과 같다.

| 선행 발 | 전진거리 | 최대 착지력 | 최대 보정 | 결과 |
|---|---:|---:|---:|---|
| 왼쪽 | 261.11 mm | 47.66 N | 0.01003 rad | 성공 |
| 오른쪽 | 265.09 mm | 45.89 N | 0.01001 rad | 성공 |

42 mm 명목 보폭, 2.56초/step, 200 mm 성공 기준은 변경하지 않았다.

## 시행착오: action 비율과 실제 관절각

첫 시도에서는 보정 한계를 기존 residual action의 12%로 정의했다. 그러나 V17 자체가
`0.10 × action_scale`만 사용하므로 실제 관절 목표 변화는 약 0.001 rad(0.06°)에 불과했다.
고충격 조건에서 10개 관절의 ±방향을 모두 최대값으로 시험해도 82.64 N을 82.15 N까지만
낮출 수 있었다.

그래서 한계를 실제 단위인 ±0.025 rad로 다시 정의했다. 같은 단일 관절 민감도 시험에서
82.64 N을 57.51 N까지 낮추는 방향이 확인돼, 여러 관절 조합을 학습할 물리적 권한이
생겼다. 모호한 normalized action 비율보다 실제 관절각 한계를 명시해야 한다는 것이 이
실험의 중요한 설계 교훈이다.

## 학습

기본 V17 정책의 파라미터는 업데이트하지 않았다. 별도의 2×128 MLP PPO 보정기만
학습했다.

- 3단계 전체-domain curriculum: 122,880 step
- 고마찰 집중 추가 학습: 160,000 step
- 실제 관절각 보정기 총학습: 282,880 step
- 고마찰 집중 분포: 75%는 마찰 1.05–1.20, 25%는 0.80–1.05
- 최종 checkpoint 선택: nominal 좌·우 성공 → 복합 성공 수 → 양쪽 동시 성공 sample 수 →
  최악 착지력 → 최소 전진거리

고마찰 추가 학습은 25/40에서 성공 수를 더 높이지 못했다. 120k checkpoint는 23/40으로
후퇴했고 160k checkpoint가 25/40과 조금 더 낮은 최악 충격을 보여 최종 모델로 선택됐다.

## V18 대비 최종 강건성 결과

동일한 seed 101–120의 복합 parameter 20개에 좌·우 선행을 각각 적용했다.

| 지표 | V17 단독 | V17 + V19 |
|---|---:|---:|
| 성공 | 12/40 (30.0%) | **25/40 (62.5%)** |
| 양쪽 모두 성공한 parameter sample | 4/20 | **10/20** |
| 낙상 | 0 | 0 |
| 착지력 실패 | 28 | **12** |
| swing unload 실패 | 10 | 11 |
| double support 실패 | 3 | 3 |
| 최소 전진거리 | 236.16 mm | **247.22 mm** |
| 최악 착지력 | 82.64 N | 83.60 N |

V19는 경계 근처의 충격 실패를 많이 복구했다. 예를 들어 seed 105 우측 선행은 동일한
조건에서 다음과 같이 바뀌었다.

| 제어기 | 전진거리 | 최대 착지력 | 결과 |
|---|---:|---:|---|
| V17 | 251.27 mm | 50.86 N | 실패 |
| V17 + V19 | 266.46 mm | 43.90 N | 성공 |

반면 seed 106처럼 contact schedule 자체가 무너지는 조건에서는 오른쪽 선행 착지력이
82.64→83.60 N으로 오히려 높아졌다. 이 조건은 단순한 착지 미세 보정보다 swing unload 및
double-support 타이밍을 함께 고쳐야 한다.

## 영상과 결과 파일

- [동일 조건 V17/V19 전후 비교](../results/khr3hv_v19_landing_residual_joint/v19_before_after_seed105_comparison.mp4)
- [nominal 좌·우 비교](../results/khr3hv_v19_landing_residual_joint/v19_nominal_left_right_comparison.mp4)
- [V19 잔여 실패 seed 106](../results/khr3hv_v19_landing_residual_joint/v19_remaining_failure_seed106_right.mp4)
- [최종 40회 비교 요약](../results/khr3hv_v19_landing_residual_joint/robustness_comparison_summary.json)
- [최종 40회 전체 결과](../results/khr3hv_v19_landing_residual_joint/robustness_comparison_results.json)
- [초기 curriculum 학습 요약](../results/khr3hv_v19_landing_residual_joint/training_summary.json)
- [고마찰 추가 학습 요약](../results/khr3hv_v19_landing_residual_joint/hard_training_summary.json)
- [최종 V19 보정 정책](../results/khr3hv_v19_landing_residual_joint/ppo_landing_residual_v19_final.zip)
- [V19 환경 코드](../humanoidv2/landing_residual_v19_env.py)
- [학습 코드](../scripts/train_landing_residual_v19.py)
- [고마찰 추가 학습 코드](../scripts/continue_landing_residual_v19.py)
- [평가 코드](../scripts/evaluate_landing_residual_v19.py)

## 재현 방법

```bash
PYTHONPATH=. python3 scripts/train_landing_residual_v19.py
PYTHONPATH=. python3 scripts/continue_landing_residual_v19.py
PYTHONPATH=. python3 scripts/evaluate_landing_residual_v19.py
```

## 다음 단계: V20

V19의 포화는 학습량 부족만의 문제가 아니다. 잔여 실패 15회 중 11회는 swing unload,
3회는 double support까지 함께 실패했다. 착지 구간에만 개입하는 V19는 이미 늦게
행동한다.

V20에서는 다음 변경이 필요하다.

1. gate 시작을 advance 후반이 아니라 hold 말기까지 앞당기되 전체 swing을 건드리지는 않는다.
2. 관측에 좌·우 발 접촉력, swing foot 높이 및 수직속도를 직접 추가한다.
3. 착지력뿐 아니라 unload와 double-support 회복을 보정기 보상에 명시한다.
4. V19 최종 정책을 초기값으로 사용하고 V17 기본 정책은 계속 고정한다.
5. 동일 40회에서 32/40 이상, nominal 양쪽 성공, 최악 충격 감소를 동시에 만족해야 다음
   속도 단계로 넘어간다.
