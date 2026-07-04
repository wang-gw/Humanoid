# STEP Assembly Transform 감사

## 목적

`URDF_F_.step` 안의 assembly occurrence와 transform을 읽어 각 부품이 CAD assembly에서 어디에 놓였는지 확인한다. 28번에서 확인한 foot collision 겹침이 STL 파일 자체 문제인지, URDF/MJCF의 geom local offset 문제인지 분리하기 위한 자료다.

## 실행 명령

```bash
python3 scripts/audit_step_assembly_transforms.py
```

## 파싱 기준

- STEP 안의 `NEXT_ASSEMBLY_USAGE_OCCURRENCE` 수: `23`
- STEP 안의 `ITEM_DEFINED_TRANSFORMATION` 수: `23`
- 이 Autodesk STEP export에서는 occurrence와 transform이 같은 순서로 나열되어 있으므로 같은 index로 매칭했다.
- 좌표 단위는 STEP/STL export 기준 `mm`로 기록했다.

## Foot 관련 Transform

| name | target origin mm | target axis | target refdir | STL center mm | STL size mm |
| --- | --- | --- | --- | --- | --- |
| `footJ_L` | `[76.7499999999994, -40.0000000000003, 64.9999999999935]` | `[3.66373598126303e-16, -3.95482965076532e-17, 0.999999999999999]` | `[1.0, -1.22464679914733e-16, -3.66373598126302e-16]` | `[0.0, 0.0, -0.6176776885986328]` | `[57.5, 43.0, 58.735355377197266]` |
| `foot_L` | `[76.7499999999994, -60.0000000000002, 24.9999999999935]` | `[-3.55271367880048e-16, -3.95482965076529e-17, 0.999999999999999]` | `[1.0, -1.22464679914733e-16, 3.5527136788005e-16]` | `[0.0, 0.0, 31.8630428314209]` | `[70.0, 120.0, 63.7260856628418]` |
| `footJ_R` | `[-77.2500000000004, -40.0000000000001, 64.9999999999994]` | `[3.66373598126302e-16, 2.59157491077089e-16, -0.999999999999998]` | `[-1.0, 1.40978720662957e-31, -3.66373598126301e-16]` | `[0.0, 0.0, 0.6176776885986328]` | `[57.5, 43.0, 58.735355377197266]` |
| `foot_R` | `[-77.2500000000004, -60.0, 24.9999999999996]` | `[3.56832406375606e-16, 1.48154393143694e-16, -0.999999999999997]` | `[-1.0, -1.04083408558607e-17, -3.6116942769837e-16]` | `[0.0, 0.0, -31.8630428314209]` | `[70.0, 120.0, 63.7260856628418]` |

## 좌우 Target Origin Delta

| left | right | distance mm | delta L-R mm |
| --- | --- | ---: | --- |
| `foot_L` | `foot_R` | 154.000000 | `[153.9999999999998, -1.9895196601282805e-13, -6.10000938650046e-12]` |
| `footJ_L` | `footJ_R` | 154.000000 | `[153.9999999999998, -1.9895196601282805e-13, -5.8975047068088315e-12]` |
| `thighJ_L` | `thighJ_R` | 100.000000 | `[99.9999999999998, -9.947598300641403e-14, 0.0]` |
| `calf_L` | `calf_R` | 100.152271 | `[100.1522710323332, -1.9895196601282805e-13, -1.0942358130705543e-12]` |

## 산출물

- JSON: `/home/king0519/projects/Humanoid/outputs/analysis/step_assembly/step_assembly_transforms_summary.json`
- transform CSV: `/home/king0519/projects/Humanoid/outputs/analysis/step_assembly/step_assembly_transforms.csv`
- 좌우 delta CSV: `/home/king0519/projects/Humanoid/outputs/analysis/step_assembly/step_left_right_pair_deltas.csv`

## 해석

STEP assembly 기준으로 좌우 foot target origin은 서로 분리되어 있다. 따라서 좌우 foot body가 분리되어 있다는 28번 감사 결과와 일치한다.

반면 기존 MJCF의 sole collision local offset은 좌우 foot body separation을 상쇄해서 두 sole collision이 같은 world 위치로 모이게 했다. 즉 현재까지의 증거로는 STL 파일 자체가 완전히 잘못된 것이라기보다, URDF/MJCF에서 foot visual/collision local origin을 잡는 방식이 문제일 가능성이 높다.

다만 STEP transform만으로 실제 발바닥 패드 접촉 중심과 standing pose를 확정할 수는 없다. 이 값은 Fusion 360에서 joint origin, foot link local frame, sole pad 위치를 확인해 최종 입력값으로 확정해야 한다.
