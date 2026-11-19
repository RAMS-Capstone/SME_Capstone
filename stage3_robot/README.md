# 단계 3 준비: Gen3에 액추에이터를 꽂을 자리 만들기

동하가 모터·액추에이터(단계 2)를 다듬는 동안 로봇 쪽을 먼저 준비한다. 액추에이터가 나오는 날 관절 2에 바로 꽂아 "이상 토크 대 액추에이터"를 비교하는 것이 목표다.

## 진행 순서

| 순서 | 할 일 | 상태 |
| --- | --- | --- |
| 1 | 관절 2 액추에이터 슬롯 (`build_stage3_slot.m`, `test_stage3_slot.m`) | **완료** (2026-11-19, 원본과 차이 0) |
| 2 | Gen3 속도 한계 안의 느린 point-to-point 궤적 (`make_p2p_trajectory.m`, `run_p2p_ideal.m`) | 스크립트 완료, MATLAB 실행 대기 |
| 3 | 동작 중 M(q) 관성 변화 그림 (`analyze_inertia_p2p.m`) | 스크립트 완료, MATLAB 실행 대기 |
| 4 | CTC에 반사 관성 옵션 추가 | 예정 |

## 1. 관절 2 액추에이터 슬롯

```
원본 :  CTC ──τ(7)──────────────────────────► Subsystem (Gen3 Multibody)
슬롯 :  CTC ──τ_cmd(7)──► J2_Actuator_Slot ──τ_applied(7)──► Subsystem
                              ▲ q(7), qd(7)  (로봇에서 되돌아옴)
```

- `J2_Actuator_Slot`은 관절 2만 꺼내(Selector) `J2_Actuator`를 거친 뒤 다시 넣는다(Assignment). 나머지 6개 관절은 그대로 지나간다.
- `J2_Actuator`는 지금 **이상 통과**다(τ_J = τ_cmd_J). 단계 2에서 내용물만 액추에이터(토크→iq, FOC, dq PMSM, 감속기, 회전자 관성)로 바꾼다. 포트 3개 입력(τ_cmd_J, q_J, qd_J), 1개 출력(τ_J)은 유지한다.
- `q_J`, `qd_J`는 지금 쓰지 않는다. 감속기 비틀림 스프링 τ = K(s·θm/N − q) + c(s·ωm/N − q̇)에 필요해서 미리 연결해 둔다.
- 로그: `tau_joint` = CTC 명령(원본과 같다), `tau_applied` = 실제로 관절에 들어간 토크(새로 추가).

### 실행

```matlab
cd stage3_robot
build_stage3_slot                            % -> Gen3_J2_slot.slx (원본 파일은 저장하지 않는다)
test_stage3_slot('baseline_input_decel.mat') % 원본과 슬롯 모델을 같은 입력으로 비교
```

- `baseline_input_decel.mat`(q_cmd, qd_cmd, qdd_cmd, t_release_plan)는 민혁 PC에만 있다. MATLAB 경로에 두거나 전체 경로를 넘긴다.
- 원본 `Gen3_throw_baseline.slx`는 인자 → `../models/original/` → 같은 폴더 → MATLAB 경로 순서로 찾는다. 폴더 밖에서 실행할 때는 `build_stage3_slot('C:/.../Gen3_throw_baseline.slx')`처럼 경로를 넘긴다.
- 통과 기준: q, qd, 토크의 최대 차이 < 1e-9.

### 결과 (2026-11-19, 민혁 PC)

| 항목 | 값 |
| --- | --- |
| 시간 격자 동일 | 예 |
| q, qd 최대 차이 (7개 관절) | 0 |
| CTC 토크 명령 최대 차이 | 0 |
| 관절에 들어간 토크 대 명령 | 0 |

슬롯을 넣어도 결과가 비트 단위로 같다(`results/stage3_slot_check.csv`, `.png`). 이제 `J2_Actuator` 안만 바꾸면 관절 2에 액추에이터가 들어간다.

## 2–3. 느린 point-to-point 동작과 관성 변화

교수님 피드백("던지기 전에 움직이는 것부터, 관절 사이 관성 변화")에 맞춘 기준 동작이다.

| 항목 | 값 |
| --- | --- |
| 동작 | q0에서 0.5 s 정지 → 3 s 이동 → 1 s 정지 → 3 s 복귀 → 1 s 정지 (총 8.5 s) |
| 움직이는 관절 | 관절 2 +0.8 rad, 관절 4 −0.8 rad (나머지 정지) |
| 궤적 | 구간마다 정지-정지 5차 다항식 (시작·끝 속도·가속도 0) |
| 피크 속도 | 1.875·0.8/3 = **0.50 rad/s** → Gen3 대형 액추에이터 한계 1.36 rad/s의 37% |
| 피크 가속도 | 0.51 rad/s² |
| 공 | 놓지 않는다(`t_release_plan` = 1e6) |

### 실행

```matlab
cd stage3_robot
run_stage3_p2p      % = make_p2p_trajectory + run_p2p_ideal + analyze_inertia_p2p
```

| 결과 파일 (`results/`) | 내용 |
| --- | --- |
| `p2p_ideal_metrics.csv` | 관절별 최대·RMS 추종 오차, 피크 속도(한계 대비 %), 피크·RMS 토크 |
| `p2p_ideal.png` | 관절 2·4의 기준 대 실제 각도, 오차, 속도, 토크 |
| `joint2_p2p_trace.csv` | `data/joint2_baseline_trace.csv`와 같은 열. `analysis/candidate_sizing_j2.py`로 요구량 재계산 |
| `p2p_inertia.csv`, `p2p_inertia.png` | M22 변화율, M23·M24 결합, 반사 관성 0.249 kg·m²를 더했을 때 모터가 보는 변화율 |

- `make_p2p_trajectory`는 시작 자세 q0, 관절 한계를 모델 안의 `q0`, `robotToTest`에서 읽는다. 속도가 한계를 넘거나 관절 한계 5° 안쪽으로 들어가면 멈춘다.
- 다른 동작을 보고 싶으면 `make_p2p_trajectory(struct('delta',[0 0.5 0 -1.0 0 0 0]','T_move',4))`처럼 바꾼다.
- 주의: CTC가 쓰는 `robotToTest`에는 공이 없을 수 있다. 공을 쥔 상태라면 이상 토크에서도 추종 오차가 조금 생긴다.
