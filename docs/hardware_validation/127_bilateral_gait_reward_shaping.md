# 127. Bilateral Gait Reward Shaping (E2E Walking v2→v6)

## 목적

doc 126(v2)에서 발 평평도(ankle twist)를 해결한 뒤, 남은 **좌우 보폭 비대칭**을
잡기 위한 reward shaping 실험 시리즈(v2→v6). 최종적으로 **진짜 양발 교번 보행(v5)**
을 달성했으나, 대칭↔장기안정의 상충을 확인했다.

## 진단: 외발 보행 로컬 옵티멈

phase별 측정으로 근본 원인 확정 — 각 발이 **자기 swing 차례**에 실제로 드는지:

| 버전 | L-swing 시 왼발 들림 | R-swing 시 오른발 들림 |
|---|---:|---:|
| v1~v4 | **2.5~3.5mm** (안 듦) | 10~12mm |

즉 **왼발이 자기 차례에도 안 들리는 외발 보행**. v1의 이 gait가 warm-start로
v2·v3·v4까지 대물림되어 로컬 옵티멈을 못 벗어났다. 원인:
- seed 서있는 자세부터 좌우 하중 비대칭 (왼발 과부하)
- 왼발 swing 단계에 체중이 오른발로 안 넘어가 왼발이 하중을 받은 채 → 못 듦

## 도입한 보상 항 (envs/urdf_f_env.py, walking task)

모두 walking task 분기에만 적용, 기본 weight=0 → 다른 task 무영향.

| 항 | 설명 | 게이트 |
|---|---|---|
| `flat_foot_penalty` (doc 126) | 접지한 발이 5° 초과 기울면 페널티 | 실제 contact |
| `swing_step_reward` | swing 발이 전진(+Y)하면 보상 (좌우 동일) | gait phase |
| `weight_shift_reward` | swing 반대쪽 stance 발에 체중 실리면 보상 | force_ratio |
| `swing_contact_penalty` | swing 발이 자기 차례에 땅에 붙어있으면 페널티 | contact |
| `foot_split_penalty` | 양발 앞뒤 gap(런지)이 deadzone 초과 시 페널티 | \|Ly-Ry\| |
| jitter (qpos/qvel) | reset 시 초기 상태 랜덤화 (강건성) | reset |

## 실험 경과

| 버전 | 방식 | 발 평평(R) | 왼발 들림 | split(런지) | 전진 | 안정성 |
|---|---|---:|---:|---:|---:|---|
| v1 | baseline | 24° | 2.5mm | 1.8cm | +13cm | ✓ 완주 |
| v2 | +flat_foot(warm) | 7.5° | 2.5mm | 12cm | +22cm | ✓ 완주 |
| v3 | +swing_step(warm) | 6.9° | 2.8mm | 20cm | +26cm | ✓ 완주 |
| v4 | +weight_shift(warm) | 6.5° | 3.5mm | 19cm | +24cm | ✓ 완주 |
| **v5** | **from-scratch 전체+swing_contact** | 8° | **15mm** ✓ | 15cm | **+28cm** | ✗ 1140step(57s) |
| v6 | +foot_split+jitter(warm from v5) | 11° | 15mm ✓ | **6cm** ↓ | +23cm | ✗ 360step(18s) |

### 핵심 발견

1. **warm-start로는 외발 보행을 못 벗어남** (v2~v4). 로컬 옵티멈이 v1부터 baked-in.
   → **from-scratch(v5)** + swing_contact 페널티 + ent_coef 0.02로 탈출, 양발 대칭 달성.

2. **연장 학습(v5 2.5M→5M)은 안정성 plateau** — 낙하 시점이 ~1140step에서 안 밀림.
   매번 정확히 동일 step 낙하 = deterministic 단일 궤적만 경험한 탓.

3. **런지 = 안정성** (v6의 역설). foot_split 페널티로 런지를 15→6cm 줄이자
   오히려 더 빨리(360step) 낙하. v5의 넓은 fore-aft 스탠스가 안정성을 제공하고
   있었고, 발을 모으니 tippier해짐. jitter+warm-start 2M로도 손실 회복 실패.

## 최종 결론

- **원래 목표(ankle twist)**: 해결 (24°→7~8°, doc 126)
- **양발 대칭**: 해결 (왼발 들림 2.5→15mm, v5)
- **대칭+평평+장기안정 동시**: reward shaping 5회로 미달성. **대칭↔안정 상충** 확인.

**채택: v5** (사용자 결정, 대칭 우선). 진짜 양발 교번 보행 + 최다 전진(+28cm),
57초/1140step 완주 후 낙하는 **알려진 한계**로 명시.

## 향후 과제 (장기 안정)

- 런지를 유지하되 회복 정책 학습: DummyVecEnv 단일 궤적 → **SubprocVecEnv + 강한 jitter/push**로 상태 분포 확장
- lateral(roll) vs pitch 낙하 원인 분리 후 targeted 보상 (측방 발 배치 등)
- gait_period_steps 튜닝 (현재 400 env step/cycle이 느릴 가능성)
- 커리큘럼(doc 123)의 명시적 weight-shift 스테이지와 결합

## 산출물

| 파일 | 설명 |
|---|---|
| `envs/urdf_f_env.py` | walking 보상 6개 항 + jitter reset |
| `scripts/train_e2e_walking.py` | 전체 보상 CLI + `find_latest_checkpoint` 숫자정렬 버그 수정 |
| `outputs/train/urdf_f/e2e_walk_v5/` | **채택 정책** (양발 대칭 보행) |
| `outputs/train/urdf_f/e2e_walk_v{2,3,4,6}/` | 실험 정책 (보존) |
| `outputs/e2e_walk_v5_tracking.mp4` | v5 보행 영상 |

## 부수 버그 수정

`find_latest_checkpoint`가 체크포인트를 **문자열 정렬**하여 `ppo_950000`을
`ppo_2500000`보다 최신으로 오판(‘9’>‘2’). **숫자 정렬**로 수정. resume 시
초기 정책으로 되돌아가던 문제 해결.
