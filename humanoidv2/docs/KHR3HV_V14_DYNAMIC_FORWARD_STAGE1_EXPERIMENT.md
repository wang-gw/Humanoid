# KHR-3HV 방식 V14 동적 보행 커리큘럼 1단계

## 결론

V14는 V13의 정방향, 대칭 관성 모델, 35 mm 보폭과 10% PPO 잔차 제약을 유지하면서 한
스텝 주기를 6.00초에서 **4.52초**로 줄였다. 최종 결정론적 정책은 좌측·우측 선행 모두
36.16초 동안 낙상 없이 8스텝을 완료했고 엄격 접촉·보폭·착지 조건을 통과했다.

평균 전방속도는 6.32/6.34 mm/s로 V13의 약 4.84 mm/s보다 약 31% 증가했다. 그러나 목표
0.15 m/s의 약 4.2%이며, 여전히 준정적 스텝에 가까워 자연스러운 동적 보행 성공으로
분류하지 않는다.

## V13에서 유지한 조건

- 모델: `URDF_F_v2_footprint_contact_symmetric_inertia.xml`
- 해부학적 전방: 월드 `-Y`
- 보폭: 35 mm
- lateral sway: 90 mm
- lift: 30 mm
- PPO 행동: 참조 관절 목표에 관절 가동범위의 10% 이내 잔차 보정
- 착지 제한: 각 스텝 `land + settle` 피크 50 N 이하
- 상위 제어 주기: 25 Hz

V13 정책과 결과 파일은 변경하지 않았다. V14는 별도 환경과 결과 폴더를 사용한다.

## 주기 압축

25 Hz 제어에서는 정확히 4.50초를 정수 control-step으로 표현할 수 없다. 가장 가까운
113-step, 4.52초를 사용했다.

| phase | control steps | 시간 |
|---|---:|---:|
| lift | 45 | 1.80 s |
| hold | 15 | 0.60 s |
| advance | 23 | 0.92 s |
| land | 23 | 0.92 s |
| settle | 7 | 0.28 s |
| 합계 | 113 | 4.52 s |

8스텝 에피소드는 904 control-step, 36.16초다. V13의 1,200-step, 48초보다 24.7% 짧다.

## 실패와 원인 분석

### 1. `kp=90` 직접 압축

압축 참조와 V13 정책 모두 8스텝을 완주하고 약 230–239 mm 전진했으나, 가장 나쁜
`advance` 구간의 5 N 미만 접촉 표본 비율이 각각 69.6%, 65.2%였다. 성공 기준 90%를
통과하지 못했다. 최대 착지력은 44.5–46.7 N으로 제한 안이었다.

### 2. 첫 106,496-step 전이학습

모든 최근 rollout은 904/904 스텝을 버텼지만 어떤 결정론적 checkpoint도 엄격 성공하지
못했다. 접촉 해제율은 최고 73.9%였고 후반 정책은 최대 착지력이 50 N을 넘었다. 최종
선택기는 실패한 학습 정책 대신 초기 V13 정책을 보존했다.

### 3. V14 전용 unload 보상 학습

스윙 발 5 N 초과 접촉에 직접 페널티를 주고 6 mm clearance 보상을 추가해 다시
106,496-step 학습했다. 접촉 해제율은 여전히 최고 73.9%였고 착지 충격도 악화됐다. 단순
보상 부족이 근본 원인이 아니었다.

### 4. phase 배분 탐색

총 113-step을 유지한 7개 phase 배분을 양쪽 순서로 평가했다. hold를 30-step까지 늘리면
접촉 해제율은 73.3%로 일부 개선됐지만 land 시간이 줄어 최대 착지력이 55 N 이상으로
증가했다. 원래 45/15/23/23/7 배분이 가장 안전했다.

## 참조 물리 탐색과 해결

lift 30–50 mm, sway 90–110 mm, `kp` 90/110 조합을 양쪽 순서로 총 60회 평가했다. 선택된
`lift=30 mm`, `sway=90 mm`, **`kp=110`** 참조만 양쪽에서 모든 엄격 조건을 통과했다.

| 참조 결과 | 좌측 선행 | 우측 선행 |
|---|---:|---:|
| 전방 진행량 | 226.21 mm | 226.18 mm |
| 최소 5 N 미만 비율 | 91.30% | 91.30% |
| 최대 착지력 | 44.63 N | 44.26 N |
| 최소 보폭 | 31.88 mm | 31.06 mm |

lift 높이를 키운 조합은 오히려 충격과 접촉을 악화했다. 짧아진 주기에서 관절이 기존
`kp=90` 목표를 제때 추종하지 못한 것이 주원인이며, `kp=110`, `kd=0.44`가 이를 해결했다.

## 최종 PPO 선택

`kp=110` 환경에서 V13 정책을 직접 평가하자 이미 양쪽 모두 성공했다. 이 성공 정책을 초기
checkpoint로 보존하고 learning rate `1e-5`로 57,344-step 미세조정했다.

- 초기 정책: 양쪽 성공, 최악 착지 47.10 N
- 25,000-step: 양쪽 성공, 최악 착지 46.81 N
- 50,000-step: 왼쪽·오른쪽 접촉 지표 회귀
- last: 왼쪽 실패

따라서 양쪽 엄격 성공을 유지하면서 최악 착지력이 가장 낮은 **25,000-step checkpoint**를
최종 정책으로 선택했다.

## 최종 결과

| 항목 | 좌측 선행 | 우측 선행 |
|---|---:|---:|
| 엄격 성공 | 성공 | 성공 |
| 낙상 | 없음 | 없음 |
| 시간 / 스텝 수 | 36.16 s / 8 | 36.16 s / 8 |
| 월드 Y 변위 | -228.68 mm | -229.29 mm |
| 전방 진행량 | 228.68 mm | 229.29 mm |
| 평균 전방속도 | 6.324 mm/s | 6.341 mm/s |
| 최소 보폭 | 31.40 mm | 32.04 mm |
| 최소 5 N 미만 비율 | 95.65% | 100.00% |
| 최대 착지력 | 46.53 N | 46.81 N |

성공 조건은 904 control-step 완주, 매 advance 평균 접촉력 5 N 미만, 5 N 미만 표본 90%
이상, settle 양발 접촉 90% 이상, 매 보폭 20 mm 이상, 전방 진행 180 mm 이상, 착지 피크
50 N 이하이다.

## 영상과 산출물

- [정면 좌우 비교 영상](../results/khr3hv_v14_dynamic_forward_stage1/dynamic_forward_v14_world_fixed_front_side_by_side.mp4)
- [사선 좌우 비교 영상](../results/khr3hv_v14_dynamic_forward_stage1/dynamic_forward_v14_world_fixed_side_by_side.mp4)
- [정면 좌측 선행 영상](../results/khr3hv_v14_dynamic_forward_stage1/dynamic_forward_v14_left_first_trained_world_fixed_front.mp4)
- [정면 우측 선행 영상](../results/khr3hv_v14_dynamic_forward_stage1/dynamic_forward_v14_right_first_trained_world_fixed_front.mp4)
- [좌측 정량 결과](../results/khr3hv_v14_dynamic_forward_stage1/dynamic_forward_v14_left_first_trained_world_fixed_summary.json)
- [우측 정량 결과](../results/khr3hv_v14_dynamic_forward_stage1/dynamic_forward_v14_right_first_trained_world_fixed_summary.json)
- [타이밍 탐색](../results/khr3hv_v14_dynamic_forward_stage1/timing_search.json)
- [참조 물리 탐색](../results/khr3hv_v14_dynamic_forward_stage1/reference_search.json)
- [첫 `kp=90` 실패 비교](../results/khr3hv_v14_dynamic_forward_stage1/initial_kp90_checkpoint_comparison.json)
- [보상 보강 실패 비교](../results/khr3hv_v14_dynamic_forward_stage1/unload_shaping/checkpoint_comparison.json)
- [`kp=110` 최종 checkpoint 비교](../results/khr3hv_v14_dynamic_forward_stage1/checkpoint_comparison.json)
- [최종 정책](../results/khr3hv_v14_dynamic_forward_stage1/ppo_dynamic_forward_v14_final.zip)
- [환경 코드](../humanoidv2/dynamic_forward_v14_env.py)

## 재현 명령

```bash
python3 scripts/search_dynamic_forward_v14_timing.py
python3 scripts/search_dynamic_forward_v14_reference.py
python3 scripts/train_dynamic_forward_v14.py \
  --timesteps 50000 --n-envs 4 --seed 17 --learning-rate 1e-5 \
  --output results/khr3hv_v14_dynamic_forward_stage1/kp110_finetune
python3 scripts/evaluate_dynamic_forward_v14.py --first-side left
python3 scripts/evaluate_dynamic_forward_v14.py --first-side right
```

## 한계와 다음 단계

- 단일 seed, 평지, 명목 물성의 결정론적 평가다.
- 강성이 `kp=110`으로 증가했으므로 실제 하드웨어 적용 전 토크·온도·구동 지연 검증이 필요하다.
- 속도는 개선됐지만 목표 0.15 m/s와는 큰 차이가 있다.
- 다음 V15는 V14를 보존하고 주기를 약 25% 더 줄인 85-step, 3.40초/step을 첫 후보로 삼는다.
- V15에서도 양쪽 엄격 성공 checkpoint만 다음 단계로 전달하며, 실패하면 주기와 `kp`를 함께
  탐색한다.
