# 05. 보행 RL 학습 — 스탠딩 성공, 보행 미달 진단

## 목적

`04`의 시드 자세에서 gait clock 기반 end-to-end 보행 정책(구모델 `124` 방식)을 학습한다.

## 학습 설정

```bash
python3 scripts/train_e2e_walking.py \
  --env gait_v2 \
  --run-name e2e_walk_v2model_v1 \
  --train-dir outputs/train/urdf_f_v2 \
  --swing-step-weight 0.5 --swing-contact-weight 0.5 \
  --total-timesteps 2000000 --n-envs 8
```

PPO 하이퍼파라미터는 구모델과 동일(n_steps=2048, batch=256, n_epochs=10, lr=3e-4,
net_arch=[256,256], n_envs=8).

## 결과: 스탠딩은 완벽, 보행은 미달

체크포인트별 롤아웃 (deterministic, 1000스텝 = 50초):

| steps | 생존 | 전진 | 최대 발 들림 |
|---|---:|---:|---:|
| 200k | 1000/1000 | +5.4 cm | 7 mm |
| 400k | 1000/1000 | +5.8 cm | — |
| 600k | 1000/1000 | +3.7 cm | — |
| 800k | 1000/1000 | +6.9 cm | — |
| 1.0M | 1000/1000 | +5.5 cm | 6.5 mm |

- ✅ **안정 스탠딩 완벽 학습**: 학습 전 24스텝 → 학습 후 50초 낙하 없음
- ❌ **보행 미달**: 50초에 3~7cm만 이동(사실상 제자리), 발 들림 6~7mm(스텝 아님)

## 진단: 서있기 지역최적

- 가만히 서있으면 안전하게 보상을 받음(접촉+직립, 벌점 없음). 발을 들면 낙하 위험
  (fall_penalty=20)이 있어 걷기를 회피 → 지역최적에 갇힘.
- 물리 확인: 한 발을 그냥 굽혀선 발이 안 뜬다(제자리 z 변화 0mm). 지지발은 체중을
  받고 있어, **먼저 반대발로 체중을 옮긴 뒤** 들어야 함(이족보행 근본 협응 난제).
- 워암스타트(1M 스탠딩 정책 → 강한 보상 swing_contact 1.5 + weight_shift)도 실패:
  스탠딩 정책의 action 탐색 폭(std)이 붕괴돼 발 들기 협응을 탐색하지 못함.

## 영상

```bash
python3 scripts/render_urdf_f_v2_walk.py \
  --policy outputs/train/urdf_f_v2/e2e_walk_v2model_v1/checkpoints/ppo_1200000_steps.zip \
  --vecnorm outputs/train/urdf_f_v2/e2e_walk_v2model_v1/checkpoints/vecnorm_1200000.pkl \
  --out outputs/urdf_f_v2/e2e_walk_v2model_standing.mp4
```

`outputs/urdf_f_v2/e2e_walk_v2model_standing.mp4` — 50초 안정 스탠딩(+7.2cm). 낙하 없음.

## 판단

- 모델·접촉·물성·자세·학습 파이프라인 **전부 검증 완료**. RL이 새 모델을 안정적으로
  제어(스탠딩)함을 확인.
- 걷기는 단일 보상 수정으로 되지 않음. 구모델도 이 지점에서 v1→v5, 90+ 실험의
  커리큘럼(weight_shift → clearance → return → walking)을 거쳐 걸었다(`../123`, `../124`).

## 다음 조치 (걷기 커리큘럼 캠페인)

1. **weight_shift 단독 학습** — 체중 좌우 이동만 먼저 (high ent_coef로 탐색 확보)
2. **foot clearance 학습** — 체중 이동 위에 스윙발 들기 추가
3. **walking 통합** — swing_contact_penalty로 교대 걸음 (구모델 v5 돌파구 방식)
4. 관절 부호 방향 정밀 검증 — `audit_joint_sign_response.py`

보상 수정은 `rewards/urdf_f_v2_walking.py`에서만 하면 구모델은 안 깨진다(`06` 참조).
