# KHR-3HV 방식 V22 첫 스텝 지지측 보정 실험

## 결론

V22는 V21처럼 정책을 다시 학습하지 않고, V20의 잔여 실패 10개를 스텝·위상·관절별로
counterfactual 재생해 공통 원인을 찾았다. 10개 실패는 모두 **첫 번째 스텝 하나에서만**
발생했으며 2–8번째 스텝은 통과했다. 따라서 V21의 직전 스텝 메모리는 이 실패들을 원리상
예방할 수 없었다.

첫 lift 동안 지지측 hip-roll 목표에 0.008 rad(약 0.46°)의 좌우 대칭 offset을 넣자 기존
복합 40회가 V20의 30/40에서 **40/40**으로 개선됐다. 선택에 사용하지 않은 300회 추가
실행에서도 V20 228/300(76.0%) 대비 V22 280/300(93.3%)으로 +52회 개선됐다.

V22는 새 PPO가 아니다. V17 기본 정책과 V20 접촉 보정 정책을 그대로 고정하고 첫 스텝의
참조 관절 목표만 작게 보정한다. 42 mm 보폭, 2.56초/step, 200 mm 전진 및 50 N 기준도
변경하지 않았다.

## 첫 스텝 진단

V20의 실패 실행은 seed 103R, 104R, 106L/R, 108R, 110L, 115L/R, 117L, 120R이다.
최종 episode 집계가 아니라 스텝별 수치를 다시 계산한 결과, 모든 실행에서 실패한 스텝은
1번뿐이었다.

| 실패 기준 | 실행 수 |
|---|---:|
| 착지력 50 N 초과 | 10 |
| advance 언로딩 0.9 미만 | 8 |
| 보폭 20 mm 미만 | 1 |
| 이후 스텝의 신규 실패 | 0 |

첫 스텝은 정지된 대칭 양발 지지 자세에서 시작하지만 이후 스텝은 이미 좌우 하중 이동이
만들어진 상태에서 시작한다. 이 차이가 첫 swing foot 분리를 어렵게 만들고, 늦은 분리가
짧은 보폭과 큰 착지 충격으로 이어진 것으로 판단했다.

## 1차 counterfactual sweep

첫 스텝에만 다음 800개 실험을 수행했다.

- phase: lift, hold, advance, land
- joint: 좌·우 10개 관절
- offset: ±0.006 rad
- 실패 실행: 10개
- 각 후보는 전체 8스텝을 끝까지 재평가

41개 counterfactual이 원래 실패를 완전 성공으로 바꿨고, 10개 실행 중 8개에는 적어도
하나의 성공 보정이 존재했다. 가장 일관된 두 방향은 다음과 같다.

| 첫 swing foot | 보정 관절 | 부호 | 10개 실패 중 회복 |
|---|---|---:|---:|
| 오른쪽 | 왼쪽 stance hip-roll | 음수 | 오른쪽 선행 4/6 |
| 왼쪽 | 오른쪽 stance hip-roll | 양수 | 왼쪽 선행 4/4 |

두 결과는 하나의 좌우 대칭 규칙이다. 10개 모두에서 `right_hip_roll +0.006` 방향은
violation을 줄였고, 반대 선행에서는 mirrored `left_hip_roll -0.006`이 같은 역할을 했다.

## 최종 보정

배포 규칙은 첫 스텝 lift에서만 적용된다.

| 첫 swing foot | stance joint | 최대 offset |
|---|---|---:|
| 왼쪽 | right_hip_roll | +0.008 rad |
| 오른쪽 | left_hip_roll | −0.008 rad |

offset은 lift 앞뒤 25%에서 smoothstep으로 0↔최대값을 보간한다. hold부터는 완전히 0이며
2–8번째 스텝에는 적용하지 않는다. V20의 관절 보정 한계 0.025 rad의 32%에 해당한다.

## 진폭 탐색

### 1차 전체 40회

| 진폭 | 복합 성공 | 양쪽 성공 sample | 최악 충격 | 최소 전진 |
|---:|---:|---:|---:|---:|
| 0 | 30/40 | 12/20 | 78.63 N | 246.54 mm |
| 0.004 | 37/40 | 18/20 | 70.95 N | 248.34 mm |
| 0.005 | 38/40 | 18/20 | 75.74 N | 247.82 mm |
| 0.006 | 38/40 | 18/20 | 66.46 N | 248.16 mm |
| 0.007 | 39/40 | 19/20 | 60.68 N | 247.81 mm |
| **0.008** | **40/40** | **20/20** | **49.97 N** | 247.12 mm |
| 0.010 | 39/40 | 19/20 | 50.54 N | 246.18 mm |
| 0.012 | 38/40 | 18/20 | 50.66 N | 245.73 mm |

0.008을 넘기면 하중 이동이 과해져 새로운 경계 충격 실패가 생겼다. 0.0075/0.0085 세밀
탐색에서도 각각 39/40이므로 최초 40회에서의 통과 구간은 좁다.

### 미사용 seed 121–170

| 지표 | V20 | V22 0.008 |
|---|---:|---:|
| 성공 | 83/100 | **91/100** |
| 양쪽 성공 sample | 38/50 | **43/50** |
| impact 실패 | 17 | **9** |
| unload 실패 | 6 | **3** |
| double-support 실패 | 2 | **0** |
| 최악 충격 | 84.34 N | **81.00 N** |

상태 전이는 성공 유지 80, 성공→실패 3, 실패→성공 11, 실패 유지 6이다.

### 최종 미사용 seed 171–270 감사

| 지표 | V20 | V22 0.008 |
|---|---:|---:|
| 성공 | 145/200 | **189/200** |
| 성공률 | 72.5% | **94.5%** |
| 양쪽 성공 sample | 64/100 | **91/100** |
| impact 실패 | 55 | **11** |
| unload 실패 | 29 | **2** |
| double-support 실패 | 4 | **0** |
| 최악 충격 | 81.39 N | **66.06 N** |
| 최소 전진 | 243.00 mm | **247.84 mm** |

상태 전이는 성공 유지 142, 성공→실패 3, 실패→성공 47, 실패 유지 8이다. 순증은
44/200이다. 0.0075와 0.0085도 같은 감사에서 각각 189/200, 192/200으로 개선을 유지해
0.008 한 점에만 의존하는 결과는 아니었다. 다만 최초 40회를 모두 통과한 값은 0.008이므로
배포값은 바꾸지 않았다.

## nominal 결과

| 선행 발 | V20 전진/충격 | V22 전진/충격 | V22 결과 |
|---|---:|---:|---|
| 왼쪽 | 265.57 mm / 46.90 N | **273.65 mm / 44.53 N** | 성공 |
| 오른쪽 | 268.10 mm / 45.87 N | **276.10 mm / 42.70 N** | 성공 |

언로딩, 양발 접촉 및 보폭 기준도 모두 통과했고 낙상은 없다.

## 대표 사례

### seed 106 우측: 기존 최악 실패 회복

| 지표 | V20 | V22 |
|---|---:|---:|
| 최대 충격 | 78.63 N | **49.76 N** |
| 최소 언로딩 비율 | 0.1 | **0.9** |
| 최소 보폭 | 16.33 mm | **23.38 mm** |
| 결과 | 실패 | **성공** |

### seed 134 우측: 미사용 조건 회복

V20은 79.58 N, 언로딩 0.4, double support 0.8로 실패했다. V22는 48.87 N,
언로딩 1.0, double support 1.0으로 성공했다.

### seed 188 좌측: 경계 회귀

V20의 47.88 N 성공이 V22에서 50.047 N 실패가 됐다. 최종 감사 200회에서 이런
성공→실패가 3건 존재한다. 따라서 V22가 모든 조건에서 V20을 지배한다고 주장할 수는 없다.
전체적으로는 실패→성공 47건이 더 커 성공률이 크게 개선됐다.

## 정책 학습을 추가하지 않은 이유

counterfactual sweep에서 한 개의 해석 가능한 대칭 보정이 바로 발견됐고, 새 seed에서도
93% 이상의 성공률을 보였다. 이를 teacher label로 다시 근사 학습하면 현재 정확한 보정
규칙에 정책 오차를 추가하게 된다. 그래서 V22에서는 V17/V20 정책을 고정하고 결정론적
참조 보정 자체를 채택했다.

## 영상과 결과

- [V22 nominal 좌·우 정면 비교](../results/khr3hv_v22_counterfactual/v22_nominal_left_right_front.mp4)
- [seed 106 V20 실패/V22 성공](../results/khr3hv_v22_counterfactual/v22_seed106_right_v20_vs_v22_front.mp4)
- [seed 134 미사용 조건 회복](../results/khr3hv_v22_counterfactual/v22_seed134_right_holdout_recovery_front.mp4)
- [seed 188 경계 회귀](../results/khr3hv_v22_counterfactual/v22_seed188_left_boundary_regression_front.mp4)
- [첫 스텝 실패 진단](../results/khr3hv_v22_counterfactual/baseline_failure_diagnostics.json)
- [800회 counterfactual 요약](../results/khr3hv_v22_counterfactual/first_pass_summary.json)
- [최초 40회 진폭 보정](../results/khr3hv_v22_counterfactual/stance_probe_calibration_summary.json)
- [seed 121–170 holdout](../results/khr3hv_v22_counterfactual/stance_probe_holdout_121_170_summary.json)
- [seed 171–270 최종 감사](../results/khr3hv_v22_counterfactual/stance_probe_final_audit_171_270_summary.json)
- [진폭 허용오차 감사](../results/khr3hv_v22_counterfactual/stance_probe_tolerance_audit_171_270_summary.json)
- [배포 설정](../results/khr3hv_v22_counterfactual/deployment_config.json)
- [V22 환경](../humanoidv2/counterfactual_probe_v22_env.py)
- [진단 코드](../scripts/diagnose_counterfactual_v22.py)
- [counterfactual sweep 코드](../scripts/sweep_counterfactual_v22.py)
- [전체 회귀 평가 코드](../scripts/calibrate_stance_probe_v22.py)
- [영상 코드](../scripts/render_stance_probe_v22.py)

## 다음 단계

V22는 현재 2.56초/step 준정적 보행의 강건성을 충분히 회복했다. 다음 단계에서는 속도를
즉시 줄이기 전에 다음 두 작업이 우선이다.

1. 성공→실패로 바뀐 경계 3건과 잔여 실패 8건에서 first-lift 관측으로 보정 필요 여부를
   구분할 수 있는지 검사한다.
2. 실제 로봇 적용을 고려해 hip-roll offset ±10%, 센서 지연 및 바닥 기울기 교란을 추가한다.
3. 이 조건에서도 기준 성공률 90% 이상을 유지하면 주기를 2.56초에서 소폭 줄이는 V23
   속도 탐색을 시작한다.
