# 단계 3 준비: Gen3에 액추에이터를 꽂을 자리 만들기

동하가 모터·액추에이터(단계 2)를 다듬는 동안 로봇 쪽을 먼저 준비한다. 액추에이터가 나오는 날 관절 2에 바로 꽂아 "이상 토크 대 액추에이터"를 비교하는 것이 목표다.

## 진행 순서

| 순서 | 할 일 | 상태 |
| --- | --- | --- |
| 1 | 관절 2 액추에이터 슬롯 (`build_stage3_slot.m`, `test_stage3_slot.m`) | **완료** (2026-11-19, 원본과 차이 0) |
| 2 | Gen3 속도 한계 안의 느린 point-to-point 궤적 | 예정 |
| 3 | 동작 중 M(q) 관성 변화 그림 (M22, M23, M24 대 반사 관성 0.25 kg·m²) | 예정 |
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
