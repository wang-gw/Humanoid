# KHR-3HV 방식 V15 동적 보행 커리큘럼 2단계

## 결론

V15는 V14의 정방향 8스텝을 4.52초/step에서 **3.40초/step**으로 압축했다. 최종 구성은
스윙 구간 `kp=130`과 착지 구간 `kp=70`을 분리한 gain scheduling을 사용한다. V14 PPO
정책을 이 제어기 위에 그대로 적용했을 때 좌측·우측 선행 모두 27.2초 동안 낙상 없이
8스텝을 완료하고 모든 엄격 성공 조건을 통과했다.

평균 전방속도는 7.446/7.435 mm/s로 V14보다 약 17.5%, V13보다 약 54% 증가했다. 다만 목표
0.15 m/s의 약 5%이고 한 스텝이 3.4초이므로, 아직 자연스러운 동적 보행 성공은 아니다.

## V14에서 유지한 조건

- 대칭 관성 MuJoCo 모델
- 해부학적 전방: 월드 `-Y`
- 보폭 35 mm, lateral sway 90 mm, lift 30 mm
- PPO 잔차 범위: 관절 가동범위의 10%
- 25 Hz 상위 제어
- 50 N 착지 제한
- 스윙 접촉 및 clearance 보상

V13과 V14 환경·정책·결과는 수정하지 않았다. 공통 상위 환경에는 과거 버전과 같은 고정
gain을 기본으로 반환하는 phase gain hook과 읽기 전용 토크 계측만 추가했다.

## 3.40초 주기

| phase | control steps | 시간 |
|---|---:|---:|
| lift | 32 | 1.28 s |
| hold | 11 | 0.44 s |
| advance | 15 | 0.60 s |
| land | 21 | 0.84 s |
| settle | 6 | 0.24 s |
| 합계 | 85 | 3.40 s |

8스텝은 총 680 control-step, 27.2초다.

## 초기 압축 실패

V14의 `kp=110`과 비례 압축 phase `34/11/17/17/6`을 사용했을 때 참조와 V14 정책은 모두
680-step을 완주했다. 그러나 다음 두 조건을 통과하지 못했다.

- 참조 접촉 해제율: 70.6–76.5%, 최대 착지력 53.34–54.28 N
- V14 정책 접촉 해제율: 양쪽 76.5%, 최대 착지력 51.16–51.27 N

낙상이나 전진 실패가 아니라 짧아진 시간 안에 발을 떼려면 강성이 더 필요하고, 그 강성을
착지까지 유지하면 충격 제한을 넘는 상충 관계였다.

## 고정 gain 탐색

9개 phase 배분과 `kp=100–140`을 좌·우 순서로 총 90회 평가했다. `kp=130–140`은 접촉
해제율을 94–100%까지 높였지만 최대 착지력이 58–65 N으로 증가했다. `kp=100–110`은
착지력이 더 낮지만 접촉 해제가 부족했다. 단일 고정 gain으로는 양쪽 엄격 성공 조합을
찾지 못했다.

## phase별 gain scheduling

스윙 추종과 착지 순응성을 분리했다.

- `lift / hold / advance`: `kp=130`, `kd=0.52`
- `land / settle`: `kp=70`, `kd=0.28`

3개 phase 배분, 스윙 `kp=120/130/140`, 착지 `kp=50–110`을 좌·우 순서로 총 126회
평가했다. 선택된 `32/11/15/21/6`, `130/70` 조합의 참조 결과는 다음과 같다.

| 참조 결과 | 좌측 선행 | 우측 선행 |
|---|---:|---:|
| 엄격 성공 | 성공 | 성공 |
| 전방 진행 | 206.30 mm | 206.27 mm |
| 최소 5 N 미만 비율 | 100% | 100% |
| 최대 착지력 | 44.39 N | 44.40 N |
| 최소 보폭 | 29.71 mm | 30.46 mm |

높은 스윙 강성으로 제시간에 발을 들고, 착지 시작과 동시에 낮은 gain으로 전환해 충격을
흡수한다. 보폭·lift·sway 등의 기하학적 목표는 변경하지 않았다.

## PPO 미세조정 결과

V14 최종 정책은 gain-scheduled V15에서 이미 양쪽 모두 성공했다. 이를 보존한 채 learning
rate `1e-5`로 57,344-step 미세조정했지만 학습 정책은 악화됐다.

| 정책 | 좌측 선행 | 우측 선행 | 최악 착지력 |
|---|---|---|---:|
| 초기 V14 정책 | 성공 | 성공 | 46.09 N |
| 25,000-step | 성공 | 실패 | 52.45 N |
| 50,000-step | 실패 | 실패 | 51.75 N |
| last | 실패 | 실패 | 53.72 N |

따라서 추가 학습 정책을 채택하지 않고 초기 V14 정책을 V15 최종 정책 파일로 복제했다.
학습 실패 checkpoint도 결과 폴더에 보존했다.

## 최종 결과

| 항목 | 좌측 선행 | 우측 선행 |
|---|---:|---:|
| 엄격 성공 | 성공 | 성공 |
| 낙상 | 없음 | 없음 |
| 시간 / 스텝 수 | 27.2 s / 8 | 27.2 s / 8 |
| 월드 Y 변위 | -202.53 mm | -202.23 mm |
| 전방 진행량 | 202.53 mm | 202.23 mm |
| 평균 전방속도 | 7.446 mm/s | 7.435 mm/s |
| 최소 보폭 | 27.90 mm | 27.89 mm |
| 최소 5 N 미만 비율 | 100% | 100% |
| 최대 착지력 | 44.99 N | 46.09 N |
| 최대 관절 토크 | 8.99 N·m | 9.16 N·m |
| 토크 한계 포화율 | 0% | 0% |

V14보다 8스텝 전진량은 약 11.7% 감소했지만 실행 시간은 24.8% 감소해 평균 속도는 약
17.5% 증가했다. 최대 관절 토크는 24 N·m 구동 한계보다 충분히 낮고, ankle-roll의 7 N·m
한계도 actuator별 clipping 계측에서 포화 표본이 없었다.

## 영상과 산출물

- [정면 좌우 비교 영상](../results/khr3hv_v15_dynamic_forward_stage2/dynamic_forward_v15_world_fixed_front_side_by_side.mp4)
- [사선 좌우 비교 영상](../results/khr3hv_v15_dynamic_forward_stage2/dynamic_forward_v15_world_fixed_side_by_side.mp4)
- [정면 좌측 선행 영상](../results/khr3hv_v15_dynamic_forward_stage2/dynamic_forward_v15_left_first_trained_world_fixed_front.mp4)
- [정면 우측 선행 영상](../results/khr3hv_v15_dynamic_forward_stage2/dynamic_forward_v15_right_first_trained_world_fixed_front.mp4)
- [좌측 정량 결과](../results/khr3hv_v15_dynamic_forward_stage2/dynamic_forward_v15_left_first_trained_world_fixed_summary.json)
- [우측 정량 결과](../results/khr3hv_v15_dynamic_forward_stage2/dynamic_forward_v15_right_first_trained_world_fixed_summary.json)
- [고정 gain 실패 탐색](../results/khr3hv_v15_dynamic_forward_stage2/fixed_gain_reference_search.json)
- [gain scheduling 탐색](../results/khr3hv_v15_dynamic_forward_stage2/reference_search.json)
- [PPO checkpoint 비교](../results/khr3hv_v15_dynamic_forward_stage2/checkpoint_comparison.json)
- [학습 요약](../results/khr3hv_v15_dynamic_forward_stage2/training_summary.json)
- [최종 정책](../results/khr3hv_v15_dynamic_forward_stage2/ppo_dynamic_forward_v15_final.zip)
- [환경 코드](../humanoidv2/dynamic_forward_v15_env.py)

## 재현 명령

```bash
python3 scripts/search_dynamic_forward_v15_reference.py
python3 scripts/train_dynamic_forward_v15.py \
  --timesteps 50000 --n-envs 4 --seed 17 --learning-rate 1e-5
python3 scripts/evaluate_dynamic_forward_v15.py --first-side left
python3 scripts/evaluate_dynamic_forward_v15.py --first-side right
```

## 한계와 다음 단계

- 단일 seed, 평지, 명목 물성의 결정론적 평가다.
- gain의 phase 경계 전환은 시뮬레이션에서는 성공했지만 실제 모터에서는 보간 전환과 지연
  검증이 필요하다.
- 속도는 목표 0.15 m/s의 약 5%에 불과하다.
- 다음 V16은 V15를 보존하고 약 25% 압축한 64-step, 2.56초/step을 첫 후보로 한다.
- 속도 증가 시 전진량 감소가 계속되면 주기 단축만으로는 한계이므로 보폭과 몸통 진행 목표를
  함께 재설계해야 한다.
