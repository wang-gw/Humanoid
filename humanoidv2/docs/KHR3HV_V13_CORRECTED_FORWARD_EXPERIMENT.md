# KHR-3HV 방식 V13 정방향 8스텝 실험

## 결론

V13은 로봇의 해부학적 전방을 월드 `-Y`로 바로잡았다. 선택된 결정론적 PPO 정책은 좌측
선행과 우측 선행 모두 48초 동안 낙상 없이 8스텝을 완료했다. 실제 월드 Y 변위는 각각
**-232.58 mm / -231.57 mm**이며, 전방 진행량은 같은 값을 양수로 보고한다.

이 결과는 6초/step의 **준정적 정방향 스텝 시뮬레이션 성공**이다. 속도는 약 4.84 mm/s로
아직 자연스러운 동적 보행이나 목표 0.15 m/s 보행 성공은 아니다.

## V1–V12 방향 오류

기존 환경은 월드 `+Y` 속도와 위치 증가를 전방으로 사용했다. 고정 정측면 카메라에서 초기
모델과 `+Y`로 강제 평행이동한 모델을 겹쳐 비교한 결과, 무릎·발·몸통이 향한 해부학적
전방은 `-Y`였고 `+Y` 이동은 후진이었다.

V13은 과거 결과 재현을 깨지 않도록 V7–V12의 기본 `forward_sign=+1`을 유지하고 새 환경만
`forward_sign=-1`을 선택한다. 다음 값에 모두 동일한 부호를 적용했다.

- 발 IK의 전후 목표
- 스윙 발 보폭
- 몸통 전방 진행량
- 온라인 재기준화 anchor 진행량
- 성공 조건의 최소 전방 거리
- target cache key

## 최종 환경

- 모델: `URDF_F_v2_footprint_contact_symmetric_inertia.xml`
- 전방: 월드 `-Y`
- 제어: 25 Hz 상위 제어 + PD 관절 추종
- 보폭: 35 mm
- lateral sway: 90 mm
- lift: 30 mm
- `kp/kd`: 90 / 0.36
- PPO 행동: 참조 관절 목표에 관절 범위의 10% 이내 잔차 보정
- 착지 제한: 매 스텝 `land + settle` 피크 50 N 이하

| phase | control steps | 시간 |
|---|---:|---:|
| lift | 60 | 2.4 s |
| hold | 20 | 0.8 s |
| advance | 30 | 1.2 s |
| land | 30 | 1.2 s |
| settle | 10 | 0.4 s |
| 합계 | 150 | 6.0 s |

`hold`는 발을 든 목표에서 접촉 과도응답이 가라앉을 때까지 기다린다. 단순히 lift 보간을
길게 하면 목표에 더 천천히 접근할 뿐 접촉 해제가 충분히 개선되지 않아 별도 phase로
분리했다.

## 참조와 학습 과정

72개 참조 조합을 탐색했다. 선택한 35 mm/90 mm/`kp=90` 참조는 양쪽 모두 약 244 mm를
`-Y`로 이동하고 43 N 미만 착지를 보였지만, 5 N 미만 스윙 접촉 표본 비율이 엄격한 90%
기준에는 미달했다.

첫 학습은 V10 정책을 초기값으로 106,496스텝 전이했다. 학습 중 확률적 rollout은 길게
버텼지만 결정론적 평균 정책은 약 105스텝에서 낙상해 실패했다. V10의 후진 과제 행동
편향을 정방향 과제로 전이한 것이 원인이었다.

두 번째 학습은 신규 PPO를 초기화하고 행동 보정을 10%로 제한했다. 106,496스텝 동안 최근
에피소드 평균 1,200/1,200스텝을 유지했다. 원래 phase에서는 첫 스텝 접촉 해제율이 0.73에
머물렀지만 20-step hold를 추가하자 양쪽 모두 0.967 이상으로 개선됐다. 최종 타이밍에서
32,768스텝을 추가 미세조정했으나 최악 착지력이 조금 증가해 미세조정 전 정책을 선택했다.
실패 정책과 모든 checkpoint는 결과 폴더에 보존했다.

## 최종 결과

| 항목 | 좌측 선행 | 우측 선행 |
|---|---:|---:|
| 엄격 성공 | 성공 | 성공 |
| 낙상 | 없음 | 없음 |
| 시간 / 스텝 수 | 48.0 s / 8 | 48.0 s / 8 |
| 월드 Y 변위 | -232.58 mm | -231.57 mm |
| 전방 진행량 | 232.58 mm | 231.57 mm |
| 평균 전방속도 | 4.845 mm/s | 4.824 mm/s |
| 최소 보폭 | 26.56 mm | 30.94 mm |
| 최소 5 N 미만 비율 | 96.67% | 96.67% |
| 최대 착지력 | 42.42 N | 42.75 N |

성공 조건은 1,200 control-step 완주, 매 advance 평균 접촉력 5 N 미만, 5 N 미만 표본
90% 이상, settle 양발 접촉 90% 이상, 매 보폭 20 mm 이상, 전방 진행 180 mm 이상,
착지 피크 50 N 이하이다.

## 영상과 결과

- [좌우 선행 비교 영상](../results/khr3hv_v13_corrected_forward/corrected_forward_world_fixed_side_by_side.mp4)
- [좌측 선행 영상](../results/khr3hv_v13_corrected_forward/corrected_forward_left_first_trained_world_fixed.mp4)
- [우측 선행 영상](../results/khr3hv_v13_corrected_forward/corrected_forward_right_first_trained_world_fixed.mp4)
- [정면 좌우 선행 비교 영상](../results/khr3hv_v13_corrected_forward/corrected_forward_world_fixed_front_side_by_side.mp4)
- [정면 좌측 선행 영상](../results/khr3hv_v13_corrected_forward/corrected_forward_left_first_trained_world_fixed_front.mp4)
- [정면 우측 선행 영상](../results/khr3hv_v13_corrected_forward/corrected_forward_right_first_trained_world_fixed_front.mp4)
- [좌측 정량 결과](../results/khr3hv_v13_corrected_forward/corrected_forward_left_first_trained_world_fixed_summary.json)
- [우측 정량 결과](../results/khr3hv_v13_corrected_forward/corrected_forward_right_first_trained_world_fixed_summary.json)
- [참조 탐색 결과](../results/khr3hv_v13_corrected_forward/reference_search_summary.json)
- [첫 전이학습 실패 비교](../results/khr3hv_v13_corrected_forward/checkpoint_comparison.json)
- [신규 PPO 학습 비교](../results/khr3hv_v13_corrected_forward/scratch_residual/checkpoint_comparison.json)
- [최종 타이밍 미세조정 비교](../results/khr3hv_v13_corrected_forward/final_timing_finetune/checkpoint_comparison.json)
- [최종 정책](../results/khr3hv_v13_corrected_forward/ppo_corrected_forward_v13_final.zip)
- [환경 코드](../humanoidv2/corrected_forward_v13_env.py)

고정 카메라는 월드 `(0, -0.12, 0.18)`을 바라보며 로봇을 추적하지 않는다. 영상에는
`anatomical forward = world -Y`, 실제 `world dY`, 양수 전방 진행량을 함께 표시한다.
정면 버전은 해부학적 전방 `-Y` 쪽의 `azimuth=270°`, `elevation=-8°`, 거리 2.0 m에서
같은 월드 지점을 바라본다.

## 재현 명령

```bash
python3 scripts/search_corrected_forward_v13_reference.py
python3 scripts/train_corrected_forward_v13.py --from-scratch \
  --timesteps 100000 --n-envs 4 --seed 17 --learning-rate 3e-5 \
  --output results/khr3hv_v13_corrected_forward/scratch_residual
python3 scripts/evaluate_corrected_forward_v13.py --first-side left
python3 scripts/evaluate_corrected_forward_v13.py --first-side right
```

## 한계와 다음 단계

- 단일 seed, 평지, 명목 물성의 결정론적 평가다.
- 8회 발 교대는 성공했지만 매우 느린 준정적 동작이다.
- 다음 단계는 전방 부호를 다시 바꾸지 못하도록 축 회귀 테스트를 유지하면서 step 주기를
  점진적으로 줄이고, 성공 checkpoint만 대상으로 다중 seed와 물성 무작위화를 평가하는 것이다.
