# KHR-3HV 방식 V11 저충격 보행 강건성 실험

## 결론

V10 정책은 명목 조건에서는 좌우 시작 모두 성공하지만, 물리값과 초기 자세를 바꾼 78회
검증에서는 **51회 성공(65.4%)**했다. 실패 27회의 원인은 낙상 18회, 발 분리 조건 실패
7회, 50 N 착지 제한 초과 2회였다.

따라서 “착지 충격을 낮추면 계속 넘어진다”가 아니라, **현재 정책의 가장 큰 문제는 착지
충격보다 초기 균형과 좌우 물성 오차에 대한 강건성 부족**이다. V10의 부드러운 착지 자체는
명목 조건과 여러 교란 조건에서 그대로 유지됐다.

## 검증 방법

V10의 6초/step, 8스텝, 50 N 제한과 모든 엄격 성공 조건을 그대로 사용했다. 성공 기준을
느슨하게 바꾸지 않았다. seed는 다음 초기 오차를 실제로 샘플링한다.

- mild 초기 오차: 관절각 표준편차 0.0035 rad(약 0.20°), 관절속도 0.01 rad/s
- moderate 초기 오차: 관절각 0.0087 rad(약 0.50°), 관절속도 0.02 rad/s
- 마찰: floor와 sole pad 마찰계수에 0.7 또는 1.3 배율
- 전체 질량/관성: 명목값의 95% 또는 105%
- 다리 비대칭 `a`: 왼쪽 질량/관성 × `(1+a)`, 오른쪽 × `(1-a)`

질량을 바꿀 때 질량과 회전관성을 같은 비율로 조정하고 MuJoCo 상수를 다시 계산했다.
모델 형상, 관절축, 제어 주기와 성공 판정은 바꾸지 않았다.

## 78회 강건성 결과

| 시나리오 | 조건 | 성공/실행 | 성공률 | 최악 착지력 |
|---|---|---:|---:|---:|
| nominal | 교란 없음 | 2/2 | 100% | 47.45 N |
| initial mild | 0.20° 초기 오차 | 8/10 | 80% | 48.08 N |
| initial moderate | 0.50° 초기 오차 | 6/10 | 60% | 50.11 N |
| friction low | 마찰 ×0.7 + mild | 6/6 | 100% | 45.49 N |
| friction high | 마찰 ×1.3 + mild | 3/6 | 50% | 52.44 N |
| mass light | 질량/관성 ×0.95 + mild | 5/6 | 83.3% | 49.90 N |
| mass heavy | 질량/관성 ×1.05 + mild | 3/6 | 50% | 46.39 N |
| left heavy | 왼쪽 +3%, 오른쪽 -3% + mild | 0/6 | 0% | 43.80 N |
| right heavy | 왼쪽 -3%, 오른쪽 +3% + mild | 1/6 | 16.7% | 47.18 N |
| combined left heavy | 마찰 ×0.7, 질량 ×1.05, 비대칭 +3%, moderate | 9/10 | 90% | 46.33 N |
| combined right heavy | 마찰 ×0.7, 질량 ×1.05, 비대칭 -3%, moderate | 8/10 | 80% | 47.19 N |

![시나리오별 성공률](../results/khr3hv_v11_robustness/robustness_success_rates.png)

낮은 마찰에서 성공률이 높아진 것은 이 정책에서 발의 미세한 slip이 횡방향/회전 하중을
풀어준 결과로 추정된다. 반대로 높은 마찰은 우측 시작 첫 체중이동을 구속해 낙상 또는 충격
증가로 이어졌다. 다만 이는 현재 시뮬레이션에서 나온 추론이며, 실제 로봇의 마찰을 낮추라는
결론은 아니다.

## 좌우 질량 비대칭 경계

비대칭만 따로 72회 추가 평가했다. `a < 0`은 오른쪽 다리가, `a > 0`은 왼쪽 다리가 더
무겁다는 의미다.

| 비대칭 a | 초기 오차 없음 | mild 초기 오차 |
|---:|---:|---:|
| -3.0% | 0/2 | 1/6 |
| -2.0% | 0/2 | 1/6 |
| **-1.0%** | **2/2** | **6/6** |
| **-0.5%** | **2/2** | **6/6** |
| 0.0% | 2/2 | 4/6 |
| +0.5% | 1/2 | 3/6 |
| +1.0% | 1/2 | 3/6 |
| +2.0% | 1/2 | 3/6 |
| +3.0% | 0/2 | 0/6 |

![좌우 질량 비대칭 성공률](../results/khr3hv_v11_robustness/asymmetry_success_rates.png)

대칭 모델인데도 작은 오른쪽-heavy 오차(-0.5~-1.0%)가 mild 초기 오차 성공률을 오히려
높였다. 이는 모델을 일부러 비대칭으로 만들라는 뜻이 아니라, **V9/V10 정책 자체에 시작
방향 편향이 남아 있음**을 보여준다. 실제 로봇 측정값 없이 -1%를 고정 보정값으로 쓰면
시뮬레이션 정책에 물리 모델을 억지로 맞추게 되므로 채택하지 않았다.

## 대표 사례

### 강한 복합 교란 성공

- 마찰 ×0.7, 전체 질량 ×1.05
- 왼쪽 다리 +3%, 오른쪽 -3%
- 0.50° 초기 관절 오차, seed 7, 좌측 시작
- 48초 8스텝 성공, 249.03 mm 전진, 최대 착지력 46.28 N

### 비대칭 낙상

- 왼쪽 다리 +3%, 오른쪽 -3%
- mild 초기 오차, seed 7, 우측 시작
- 첫 step 중 3.72초에서 낙상

### 낙상하지 않았지만 충격 제한 실패

- moderate 초기 오차, seed 29, 우측 시작
- 48초 8스텝 보행은 완주
- 최대 착지력 50.112 N으로 제한을 0.112 N 초과

이 사례들은 낙상과 착지 충격 실패가 같은 현상이 아님을 보여준다.

## 영상과 결과 파일

- 강건 성공/충격 실패 비교: [`robust_success_vs_impact_failure.mp4`](../results/khr3hv_v11_robustness/robust_success_vs_impact_failure.mp4)
- 강한 복합 교란 성공: [`robust_combined_left_heavy_left_seed7.mp4`](../results/khr3hv_v11_robustness/robust_combined_left_heavy_left_seed7.mp4)
- 비대칭 낙상: [`robust_left_heavy_right_seed7.mp4`](../results/khr3hv_v11_robustness/robust_left_heavy_right_seed7.mp4)
- 50 N 초과 완주: [`robust_initial_moderate_right_seed29.mp4`](../results/khr3hv_v11_robustness/robust_initial_moderate_right_seed29.mp4)
- 78회 전체 요약: [`robustness_summary.json`](../results/khr3hv_v11_robustness/robustness_summary.json)
- 78회 개별 결과: [`robustness_results.json`](../results/khr3hv_v11_robustness/robustness_results.json)
- 비대칭 sweep 요약: [`asymmetry_sweep_summary.json`](../results/khr3hv_v11_robustness/asymmetry_sweep_summary.json)
- V11 환경: [`robust_soft_landing_v11_env.py`](../humanoidv2/robust_soft_landing_v11_env.py)

## 재현 명령

```bash
python3 scripts/evaluate_robust_soft_landing_v11.py
python3 scripts/sweep_soft_landing_v11_asymmetry.py
python3 scripts/plot_soft_landing_v11_robustness.py

python3 scripts/evaluate_robust_soft_landing_v11.py \
  --video-scenario combined_left_heavy --first-side left --seed 7
python3 scripts/evaluate_robust_soft_landing_v11.py \
  --video-scenario left_heavy --first-side right --seed 7
python3 scripts/evaluate_robust_soft_landing_v11.py \
  --video-scenario initial_moderate --first-side right --seed 29
```

## 한계와 다음 단계

- 시나리오당 표본이 2~10회라 성공률을 실제 하드웨어 확률로 해석하면 안 된다.
- 마찰·질량 범위는 stress test이며 실제 제작 공차를 측정한 값이 아니다.
- 센서 측정 잡음, 지면 높이 변화, actuator 지연은 아직 포함하지 않았다.
- 다음 V12는 작은 domain randomization부터 시작하는 curriculum과 매우 낮은 learning rate,
  KL 제한, 양방향 checkpoint 검증을 사용해야 한다. 이전처럼 전체 정책을 한 번에 크게
  업데이트하면 다시 조기 낙상할 가능성이 높다.

