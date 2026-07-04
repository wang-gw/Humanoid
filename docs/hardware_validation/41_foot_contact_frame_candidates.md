# Foot contact frame 후보 감사

## 목적

사용자 제공 foot contact box가 Fusion local frame 기준이므로, 현재 MJCF foot body frame에 어떻게 넣어야 하는지 후보를 비교한다.

## 실행 명령

```bash
python3 scripts/audit_foot_contact_frame_candidates.py
```

## 후보 비교

| 후보 | 좌우 z 차이 m | COM inside x | COM inside y | left world | right world |
| --- | ---: | --- | --- | --- | --- |
| `user_direct` | 0.050000 | `False` | `False` | `[-0.009749999999999995, 0.0385, 0.03499999999999998]` | `[-0.16425, 0.0385, 0.08499999999999998]` |
| `body_centered_user_size_bottom_40mm` | 0.000000 | `True` | `True` | `[0.067, -0.0215, 0.01999999999999998]` | `[-0.087, -0.0215, 0.01999999999999998]` |
| `body_centered_thin_contact` | 0.000000 | `True` | `True` | `[0.067, -0.0215, 0.004999999999999977]` | `[-0.087, -0.0215, 0.004999999999999977]` |

## 산출물

- JSON: `/home/king0519/projects/Humanoid/outputs/analysis/foot_contact_frame_candidates/foot_contact_frame_candidates.json`
- CSV: `/home/king0519/projects/Humanoid/outputs/analysis/foot_contact_frame_candidates/foot_contact_frame_candidates.csv`

## 판단

`user_direct` 해석은 좌우 contact center 높이가 `0.05 m` 차이 나므로 현재 MJCF body frame에는 직접 사용할 수 없다.

`body_centered_user_size_bottom_40mm`는 사용자 제공 foot size를 유지하면서 좌우 contact 높이를 맞추고 COM projection도 support AABB 안에 둔다. 따라서 다음 동역학 검증용 변환 후보로 사용한다.
