# 115. Left Mirror Sequence Precheck

## 목적

이전 `114` 단계에서 오른발 단일 step sequence가 성공했다.

이번 단계의 목적은 같은 흐름을 왼발에도 적용할 수 있는지 확인하는 것이다.

목표 sequence:

1. right weight shift
2. left clearance
3. left return
4. standing 복귀

## 먼저 확인한 것

오른발 sequence에서 사용한 pose들을 좌우 반전해서 왼발용 시작 pose 후보를 만들었다.

추가한 config:

- `configs/weight_shift_right065_mirrored_from_left065.json`
- `configs/left_foot_lift002_ik_mirrored_from_right002.json`
- `configs/left_foot_lift005_ik_mirrored_from_right005.json`
- `configs/weight_shift_right065_mirrored_swap_only.json`

시도한 mirror rule:

1. left/right joint를 swap하고 `*_roll` joint 부호를 반전
2. left/right joint만 swap하고 `*_roll` 부호는 유지

## 결과

### Rule 1: swap + roll sign flip

`weight_shift_right065_mirrored_from_left065.json`

| 항목 | 값 |
|---|---:|
| 완료 step | 27 / 250 |
| 종료 이유 | roll_limit |
| final roll | 0.555 rad |
| final pitch | 0.556 rad |

`left_foot_lift002_ik_mirrored_from_right002.json`

| 항목 | 값 |
|---|---:|
| 완료 step | 25 / 250 |
| 종료 이유 | pitch_limit |
| final roll | 0.492 rad |
| final pitch | 0.570 rad |

`left_foot_lift005_ik_mirrored_from_right005.json`

| 항목 | 값 |
|---|---:|
| 완료 step | 23 / 250 |
| 종료 이유 | pitch_limit |
| final roll | 0.403 rad |
| final pitch | 0.586 rad |

### Rule 2: swap only

`weight_shift_right065_mirrored_swap_only.json`

| 항목 | 값 |
|---|---:|
| 완료 step | 32 / 250 |
| 종료 이유 | roll_limit |
| final roll | 0.563 rad |
| final pitch | 0.191 rad |

## 해석

단순한 좌우 pose mirror는 실패했다.

이 결과는 중요한 의미가 있다.

- 현재 MuJoCo 모델의 좌우 joint axis/sign, contact geometry, COM offset이 완전한 수학적 mirror로 동작하지 않는다.
- 오른쪽에서 성공한 pose를 숫자만 좌우 교환해서 왼쪽에 쓰면 동역학적으로 안정하지 않다.
- 따라서 왼발 sequence는 오른쪽 policy를 그대로 mirror해서 붙이는 방식보다, 왼쪽 전용 pose search와 왼쪽 전용 RL task가 필요하다.

## 현재 결론

왼발 sequence로 바로 넘어가는 것은 아직 이르다.

먼저 해야 할 일:

1. `weight_shift_right` pose search
2. `left_unload` 또는 `left_clearance` task 추가
3. left foot clearance/contact 계산 추가
4. left return task 추가
5. 왼발 전용 curriculum 학습

## 다음 조치

다음 단계는 단순 mirror가 아니라 왼쪽 전용 환경 확장이다.

구체적으로:

- `UrdfFEnv`에 `left_unload`, `left_clearance`, `left_return` task 추가
- observation에 left/right clearance를 모두 넣는 새 환경 버전 또는 left task 전용 observation 구성
- `weight_shift_right` pose를 search하거나 RL로 학습

현재 판단:

> 오른발 step sequence는 성공했지만, 왼발은 단순 mirror로는 안 된다. 왼쪽 전용 pose/task를 만들어야 한다.
