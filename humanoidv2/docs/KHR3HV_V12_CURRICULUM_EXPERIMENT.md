# KHR-3HV 방식 V12 단계형 Domain Randomization 실험

## 결론

V11에서 확인한 초기 자세·좌우 물성 민감도를 줄이기 위해 V10 정책을 작은 교란부터
단계적으로 PPO 전이학습했다. 그러나 **가장 약한 1단계부터 우측 시작 명목 성능이
깨졌고**, 교란 범위를 늘릴수록 낙상이 증가했다.

자동 검증은 모든 학습 checkpoint를 배제하고 V10 초기 정책을 최종 선택했다. 선택된 V12
정책과 V10 정책의 모든 parameter 차이는 정확히 0이다. 따라서 V12는 성능 개선 버전이
아니라, **vanilla PPO로 기존 정책 전체를 미세조정하는 접근이 부적합하다는 실패 실험**이다.

## 이전 실패와 달리 적용한 안전장치

V10의 첫 전이학습보다 update를 크게 제한했다.

| 항목 | V10 전이학습 | V12 |
|---|---:|---:|
| learning rate | 1e-4 | **1e-5** |
| PPO epochs | 10 | **2** |
| clip range | 0.2 | **0.05** |
| target KL | 없음 | **0.002** |
| 학습 방식 | 고정 충격 페널티 | 3단계 domain randomization |
| 안전 선택 | checkpoint 비교 | 명목 성공 우선 + 40회 검증 |

최종 정책은 학습 checkpoint가 명목 양방향을 모두 유지할 때만 선택될 수 있도록 했다.

## Curriculum 범위

각 reset마다 범위 안에서 마찰, 전체 질량/관성, 좌우 다리 비대칭과 초기 관절 오차를
새로 샘플링했다.

| 단계 | 학습량 | 마찰 | 전체 질량 | 다리 비대칭 | 관절각 초기 오차 |
|---|---:|---:|---:|---:|---:|
| 0 | 9,600 | ±2% | ±0.5% | ±0.1% | 0.001 rad |
| 1 | 19,200 | ±10% | ±1% | ±0.5% | 0.0025 rad |
| 2 | 24,000 | ±20% | ±2% | ±1% | 0.0035 rad |

누적 학습량은 52,800 step이다. V10의 50 N 제한과 다른 엄격 성공 조건은 바꾸지 않았다.

## 학습 중 변화

| checkpoint | 누적 step | 최근 평균 episode 길이 | 최근 평균 reward |
|---|---:|---:|---:|
| stage 0 | 9,600 | 1,174.0 | 668.83 |
| stage 1 | 28,800 | 1,151.9 | 658.76 |
| stage 2 | 52,800 | 1,092.6 | 625.06 |

교란 범위가 커질수록 episode 길이와 reward가 모두 감소했다. 다만 학습 로그만으로 정책을
고르지 않고 각 checkpoint를 동일한 40회 검증 세트에서 다시 실행했다.

## 40회 checkpoint 검증

검증 세트는 명목 양방향, mild/moderate 초기 오차, 높은 마찰, +5% 전체 질량,
오른쪽-heavy 1%, 왼쪽-heavy 0.5%를 포함한다.

| 정책 | 명목 성공 | 전체 성공 | 낙상 | 비낙상 충격 실패 |
|---|---:|---:|---:|---:|
| **V10 초기 정책** | **2/2** | **28/40** | **10** | 2 |
| stage 0 | 1/2 | 24/40 | 11 | 3 |
| stage 1 | 1/2 | 24/40 | 14 | 1 |
| stage 2 | 1/2 | 16/40 | 19 | 4 |

![V12 checkpoint 검증](../results/khr3hv_v12_curriculum/checkpoint_validation.png)

### 방향별 붕괴

- stage 0: 좌측 시작 명목 보행은 유지했지만 우측 시작 착지력이 62.98 N으로 증가
- stage 1: 우측 시작 4.60초 부근 낙상
- stage 2: 우측 시작 4.40초에서 낙상
- 모든 단계의 좌측 시작 명목 보행은 유지

이 결과는 V11에서 발견한 좌우 정책 편향이 PPO update 과정에서도 반복됐음을 보여준다.
작은 평균 update라도 여러 번 누적되면서 우측 시작의 좁은 안정 영역을 벗어났다.

## 최종 선택과 V11 결과 재사용

최종 선택 archive는 V10 정책을 다시 저장한 것이다.

- `state_dict` key: 완전히 동일
- 모든 parameter의 최대 절대 차이: **0.0**
- 따라서 V11의 78회 성공률 65.4%와 72회 비대칭 sweep 결과도 수치적으로 동일

동일한 정책을 같은 결정론적 환경에서 78회 다시 실행하는 것은 새 정보를 만들지 않으므로
재실행하지 않았다. 대신 학습 checkpoint의 실패 영상과 40회 검증 결과를 보존했다.

## 영상과 결과 파일

- V10/거부된 stage 0 비교: [`v10_vs_v12_stage0_nominal.mp4`](../results/khr3hv_v12_curriculum/v10_vs_v12_stage0_nominal.mp4)
- stage 0 우측 시작 62.98 N 완주: [`robust_nominal_right_seed7.mp4`](../results/khr3hv_v12_curriculum/stage0_video/robust_nominal_right_seed7.mp4)
- stage 2 우측 시작 낙상: [`robust_nominal_right_seed7.mp4`](../results/khr3hv_v12_curriculum/stage2_video/robust_nominal_right_seed7.mp4)
- 학습 요약: [`training_summary.json`](../results/khr3hv_v12_curriculum/training_summary.json)
- 160회 checkpoint 검증 원본: [`validation_comparison.json`](../results/khr3hv_v12_curriculum/validation_comparison.json)
- 정책 동일성: [`policy_equivalence.json`](../results/khr3hv_v12_curriculum/policy_equivalence.json)
- 최종 선택 정책: [`ppo_curriculum_final.zip`](../results/khr3hv_v12_curriculum/ppo_curriculum_final.zip)
- curriculum 환경: [`curriculum_soft_landing_v12_env.py`](../humanoidv2/curriculum_soft_landing_v12_env.py)

## 재현 명령

```bash
python3 scripts/train_curriculum_soft_landing_v12.py
python3 scripts/plot_curriculum_soft_landing_v12.py

python3 scripts/evaluate_robust_soft_landing_v11.py \
  --model results/khr3hv_v12_curriculum/ppo_curriculum_stage0.zip \
  --output results/khr3hv_v12_curriculum/stage0_video \
  --video-scenario nominal --first-side right --seed 7
```

## 다음 설계 판단

학습률을 더 낮추는 것만으로는 누적 정책 drift를 막기 어렵다. 다음 시도는 기존 V10 정책을
완전히 고정하고, 그 출력 위에 크기가 제한된 작은 보정기만 학습하는 구조가 적절하다.

```text
final action = frozen V10 action + bounded correction
```

이 구조라면 보정기가 실패해도 V10 동작에서 멀리 벗어날 수 없고, 보정 gain을 단계적으로
0에서 늘릴 수 있다. 다음 버전에서는 전체 PPO fine-tuning을 반복하지 않는 것이 핵심이다.

