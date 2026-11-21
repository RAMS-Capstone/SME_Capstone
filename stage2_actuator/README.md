# 단계 2: 관절 2 액추에이터 (동하용 틀)

단계 1에서 만든 모터에 감속기를 붙여 **관절 2 액추에이터** 블록 하나로 만든다. 이 블록은 나중에 Gen3 모델의 `J2_Actuator` 자리에 그대로 들어간다. 포트 규격·부호·식은 Claude 문서 "동하에게: 관절 2 액추에이터를 Gen3에 꽂는 방법"과 같다.

## 동하가 할 일

1. **실행해서 결과 보기** (아래 "실행"). 시험 4개가 PASS인지 확인한다.
2. **블록과 식 대조하기**: `J2_Actuator.slx`를 열고 아래 대응표의 블록이 어느 식의 어느 항인지 짚어 본다.
3. **내 모터로 바꾸기** (교수님 피드백을 반영해 모터를 다듬었다면): `FOC_current_ctrl`, `PMSM_dq_manual` 자리에 다듬은 버전을 넣는다. 입출력(`iq_cmd, id, iq, wm → vd, vq, diag` / `vd, vq, wm → id, iq, Te`)은 유지한다. 그리고 시험을 다시 돌린다.
4. **결과 해석하기**: 아래 "결과를 어떻게 읽나"의 질문 세 개에 답한다.
5. **민혁에게 보내기**: `J2_Actuator.slx`, `stage2_test_results.csv`, `stage2_tests.png`, 바꾼 점 한 문단.

## 실행

이 폴더가 저장소의 `stage2_actuator/`에 있으면 단계 1 파일(`../stage1_motor/j2_params.m`, `../stage1_motor/results/J2_PMSM_Stage1.slx`)을 자동으로 찾는다.

```matlab
cd stage2_actuator
build_stage2_actuator      % -> J2_Actuator.slx   (단계 1 FOC·모터 + 모터축 + 감속기)
build_stage2_testbench     % -> J2_Actuator_TestBench.slx (1자유도 시험대)
test_stage2_actuator       % 시험 5회 -> stage2_test_results.csv, stage2_tests.png
```

다른 폴더에서 실행할 때는 `build_stage2_actuator('C:/.../J2_PMSM_Stage1.slx')`처럼 경로를 넘긴다. 이때 `j2_params.m`도 같은 폴더나 MATLAB 경로에 둔다.

## 블록과 식 대응표 (`J2_Actuator.slx`)

| 블록 | 식 | 의미 |
| --- | --- | --- |
| `torque_to_iq` | iq_cmd = τ_cmd / (s·N·Kt) | 관절 토크 명령을 모터 전류 명령으로 |
| `FOC_current_ctrl` | 단계 1 그대로 | 전류 PI, 디커플링, 원형 전압 제한 12.47 V, ±20.9 A |
| `PMSM_dq_manual` | 단계 1 그대로 | dq 전압 방정식 → Te |
| `shaft_torque`, `gear_reaction`, `inv_Jshaft`, `omega_m`, `theta_m` | (Jm + J_WG)·ω̇m = Te − (s/N)·τ_J | 모터축(회전자 + 웨이브 제너레이터) 운동 방정식 |
| `theta_to_joint`, `deflection`, `Kg` | Kg·(s·θm/N − q_J) | 감속기 비틀림 스프링 |
| `omega_to_joint`, `deflection_rate`, `cg` | cg·(s·ωm/N − q̇_J) | 감속기 감쇠 |
| `tau_gear` → `tau_J` | 두 항의 합 | 관절에 실제로 들어가는 토크 |
| `log_mux` → `j2_act_log` | — | iq_cmd, iq, id, vd, vq, \|v\|, ωm, Te, τ_J, 비틀림 |

- 파라미터는 모델 작업공간의 `act`(= `j2_params()` + Jr, 초기값)에 있다. 단계 1 블록의 `p.xxx`는 `act.xxx`로 바뀌어 있다(Gen3 변수와 겹치지 않게).
- FOC와 모터에 들어가는 속도는 **모터축 상태 ωm**이다. 감속기가 탄성이라 ωm ≠ s·N·q̇이다.

## 시험 (`test_stage2_actuator`)

| 시험 | 조건 | Python 기준값 (`verify_stage2.py`) |
| --- | --- | --- |
| T1 정적 | 관절 고정, 10 N·m 계단 | τ_J → 10.000 N·m, 비틀림 0.714 mrad, 순간 피크 18.6 N·m, iq −2.29 A |
| T2 부호 | T1을 s = +1로 | τ_J가 T1과 같다, iq만 +2.29 A |
| T3 전류 제한 | 관절 고정, 100 N·m | iq 명령 20.9 A에서 잘림, τ_J → 91.17 N·m |
| T4 진자 | 1.2 kg·m² + 중력, 0 → 0.8 rad, 이상 토크와 같은 명령 | 최대 각도 차이 22.3 mrad |
| T4b 진자 + Jr 보상 | 명령에 Jr·q̈_ref(0.249·q̈) 추가 | 0.08 mrad |

## 결과를 어떻게 읽나 (동하가 답해 볼 질문)

1. **T1에서 최종 토크는 10 N·m인데 왜 순간 피크는 18.6 N·m일까?** 감속기 스프링과 회전자 관성이 약 38 Hz 공진을 만든다. 감쇠비가 0.05로 작아서 계단 명령에 크게 출렁인다.
2. **T4에서 같은 토크 명령인데 왜 22 mrad 어긋날까?** 액추에이터에는 회전자 관성(관절 환산 0.249 kg·m², 부하의 약 21%)이 더 있다. 그래서 같은 토크로는 덜 가속된다.
3. **T4b에서 왜 0.08 mrad로 줄까?** 명령에 Jr·q̈를 미리 더하면 회전자를 가속하는 데 필요한 토크를 보충한다. Gen3에서 CTC의 `ctc_Jr` 옵션이 하는 일이 바로 이것이다.

## 파일

| 파일 | 내용 |
| --- | --- |
| `build_stage2_actuator.m` | `J2_Actuator.slx` 생성 |
| `build_stage2_testbench.m` | `J2_Actuator_TestBench.slx` 생성 (액추에이터 내용을 서브시스템으로 복사 — Gen3에 꽂을 때와 같은 방법) |
| `test_stage2_actuator.m` | 시험 5회, Python 기준과 자동 비교 |
| `find_stage1.m` | 단계 1 파일 찾기 |
| `verify_stage2.py`, `stage2_reference.json/.png` | MATLAB과 독립인 Python 기준 모델과 결과 |

## 결과 (2026-11-21, 민혁 PC, 틀 그대로 실행)

12개 항목 모두 통과했고, Python 기준과 표시 자릿수까지 같다(`results/stage2_test_results.csv`, `results/stage2_tests.png`).

- **감속기 공진 피크:** 10 N·m 계단에서 순간 18.6 N·m(1.86배)가 나온다. 100 N·m 명령(전류 제한으로 최종 91.2 N·m)에서는 그래프상 **순간 약 170 N·m**까지 출렁인다. 이는 CSG-17-100의 반복 피크 한계 70 N·m는 물론 순간 피크 한계 125 N·m도 넘는 값이다. 탄성 감속기에 토크를 계단처럼 걸면 감속기 한계를 넘을 수 있다는 뜻이다. Gen3에서는 CTC 명령이 부드러워서 괜찮을 것으로 보이지만, 던지기의 릴리스·감속 구간에서 감속기 토크(τ_J) 피크를 반드시 확인해야 한다.
- **입력 모양의 영향:** 처음 실행에서는 계단이 10 ms 경사로 들어가(입력 포트 선형 보간) 피크가 16.7 N·m였다. 공진 주기(약 26 ms)에 비해 짧은 경사만으로도 출렁임이 줄어든다.
- **회전자 관성 보상:** 진자에서 같은 명령이면 22.3 mrad 어긋나지만 Jr·q̈ 피드포워드를 넣으면 0.08 mrad로 준다. Gen3 CTC의 `ctc_Jr`가 하는 일이다.
