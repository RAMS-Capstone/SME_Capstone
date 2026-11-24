# 단계 3 준비: Gen3에 액추에이터를 꽂을 자리 만들기

동하가 모터·액추에이터(단계 2)를 다듬는 동안 로봇 쪽을 먼저 준비한다. 액추에이터가 나오는 날 관절 2에 바로 꽂아 "이상 토크 대 액추에이터"를 비교하는 것이 목표다.

## 진행 순서

| 순서 | 할 일 | 상태 |
| --- | --- | --- |
| 1 | 관절 2 액추에이터 슬롯 (`build_stage3_slot.m`, `test_stage3_slot.m`) | **완료** (2026-11-19, 원본과 차이 0) |
| 2 | Gen3 속도 한계 안의 느린 point-to-point 궤적 (`make_p2p_trajectory.m`, `run_p2p_ideal.m`) | **완료** (2026-11-20) |
| 3 | 동작 중 M(q) 관성 변화 그림 (`analyze_inertia_p2p.m`) | **완료** (2026-11-20) |
| 4 | CTC에 반사 관성 옵션 추가 (`build_stage3_ctc_jr.m`, `test_stage3_ctc_jr.m`) | **완료** (2026-11-21) |
| 3a | CTC에 공(페이로드) 반영, 놓는 순간 전환 (`build_stage3_payload.m`, `test_stage3_payload.m`) | **완료** (2026-11-21) |
| 5 | 관절 2 슬롯에 단계 2 액추에이터를 넣고 이상 토크와 비교 (`build_stage3_actuator.m`, `run_stage3_actuator.m`) | **MATLAB 실행 대기** (2026-11-24) |

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

- `baseline_input_decel.mat`(q_cmd, qd_cmd, qdd_cmd, t_release_plan)는 이 폴더에 있다(2026-11-24 저장소에 추가). 인자를 생략하면 이 파일을 쓴다.
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

### 결과 (2026-11-20, 민혁 PC, 이상 토크)

| 관절 | 최대 추종 오차 | 피크 속도 (한계 대비) | 피크 토크 |
| --- | --- | --- | --- |
| 2 | 7.1 mrad | 0.505 rad/s (37%) | 11.3 N·m |
| 4 | 16.0 mrad | 0.513 rad/s (38%) | 0.16 N·m |
| 6 | 20.8 mrad | 0.027 rad/s | 0.02 N·m |

| 관성 (관절 2가 보는 것) | 값 |
| --- | --- |
| M22 (팔만) | 1.158–1.321 kg·m², 동작 중 **13%** 변화 |
| M22 + 반사 관성 0.249 | 모터가 보는 변화 **11%** (반사 관성은 전체의 16–18%) |
| M24 결합 | 0.39–0.47 kg·m², M22의 최대 **35%** |
| M23 결합 | 0.02 kg·m² 이하 (무시 가능) |

해석:

- **이상 토크인데도 정지 구간에 오차가 남는다 → 원인: 공 0.1 kg (확인됨).** q0(관절 2 = 0, 팔이 서 있는 자세)에서는 오차가 0이다. 팔을 기울인 q0+Δ에서는 관절 2가 −7 mrad, 관절 4가 +16 mrad로 일정하게 남는다. `diagnose_gravity_mismatch.m`으로 확인한 결과, CTC 모델(`robotToTest`)에는 실제 로봇이 쥔 공(0.1 kg, 엔드이펙터 원점)이 없다. 관절 2에서 모델이 빠뜨린 중력은 −0.2961 N·m이고, 공의 중력 토크는 −0.2965 N·m라서 차이가 0.0005 N·m(0.2%)다. 링크 질량 7개와 중력 벡터는 모델과 실제가 같다(`results/gravity_mismatch.csv`). CTC에 적분항이 없어서 이 차이가 일정 오차로 남는다. 관절 4는 직접 받는 차이가 0.002 N·m뿐인데도 오차가 가장 크다. CTC의 오차는 e = (M·Kp)⁻¹·Δ라서, 관절 2의 토크 부족이 질량 행렬 결합(M24)을 거쳐 관절 4 오차로도 나타나기 때문이다.
- **관성 결합이 크다.** 관절 4가 가속하면 그 가속도의 35%만큼 관절 2에 관성 토크가 걸린다(M24/M22). 교수님이 말한 "관절 사이 관성 변화"가 수치로 보인다.
- **감속비 100으로는 결합이 숨지 않는다.** 반사 관성(0.249 kg·m²)은 M22의 약 20%라 변화율을 13%에서 11%로만 줄인다. "고감속비면 관절이 분리되어 보인다"는 이 관절에서는 성립하지 않는다.
- **이 동작은 액추에이터 한계에 한참 못 미친다.** 관절 2 피크 11.3 N·m는 모터 iq 약 3.7 A(효율 0.7)이고, 모터 속도 약 480 rpm에서 역기전력은 약 1.5 V다. 전류 한계 20.9 A, 전압 한계 12.47 V보다 훨씬 작다. 그래서 이 동작에 액추에이터를 붙이면 차이는 주로 회전자 관성과 감속기 탄성에서 나올 것이다. 전류·전압 한계의 영향은 더 빠른 동작이나 18 V·10.4 A 사례에서 봐야 한다.

## 3a. 페이로드를 아는 CTC (공 0.1 kg)

진단 결과 정지 오차의 원인은 CTC 모델에 공이 없는 것이었다(`diagnose_gravity_mismatch.m`). 실제 로봇 제어기처럼 **페이로드 설정**을 넣는다.

- **현실성 근거:** 공을 놓는 건 제어기 자신이 내리는 명령이다. 그래서 제어기는 놓는 시점을 안다. 실제 로봇도 잡을 때 페이로드를 설정하고 놓을 때 해제한다.
- **구현:** `Gen3_J2_slot.slx`를 복사해 `Gen3_J2_payload.slx`를 만든다. CTC 안에서 `robotToTest`를 쓰는 블록 3개(중력, 질량 행렬, 속도 곱)에 공 포함 모델(`robotPayload`)을 쓰는 쌍둥이 블록을 붙인다. 그리고 Switch로 고른다.
- **놓는 신호 통일:** 전환 조건은 `t < t_release_plan`이다. 실제 로봇에서 공을 떼는 용접 조인트 Step과 같은 파라미터를 쓴다. 전환 여부는 `ctc_payload_on`으로 기록한다.
- **질량 오차 민감도:** 제어기가 0.12 kg(+20%)으로 알고 있는 경우도 같은 모델로 돌린다(`make_payload_robot`, 실행 시에만 덮어씀).

```matlab
cd stage3_robot
build_stage3_payload                            % -> Gen3_J2_payload.slx (Gen3_J2_slot.slx 필요)
test_stage3_payload('baseline_input_decel.mat') % 느린 동작 3사례 + 던지기 2사례
```

통과 기준: 페이로드 CTC의 느린 동작 최대 오차가 모든 관절에서 0.5 mrad 미만이어야 한다. 던지기에서는 `ctc_payload_on`이 `t_release_plan`에서 1 → 0이 되어야 한다. 결과는 `results/payload_check.csv`, `.png`에 저장된다.

### 결과 (2026-11-21, 민혁 PC)

| 동작 | CTC 설정 | 관절 2 최대 오차 | 관절 4 | 관절 6 | 전 관절 최대 |
| --- | --- | --- | --- | --- | --- |
| 느린 동작 | 공 모름 (기존) | 7.05 mrad | 15.98 | 20.84 | 20.84 mrad |
| 느린 동작 | 공 0.1 kg (정확) | 0.022 | 0.022 | 0.00003 | **0.022 mrad** |
| 느린 동작 | 공 0.12 kg (+20% 오설정) | 1.37 | 2.94 | 2.71 | 2.94 mrad |
| 던지기 | 공 모름 (기존 baseline) | 10.95 | 23.19 | **159.9** | 159.9 mrad |
| 던지기 | 공 0.1 kg, 놓을 때 전환 | 0.018 | 0.023 | 0.026 | **0.026 mrad** |

- **기준선 오차는 전부 공 때문이었다.** 페이로드를 알려 주자 느린 동작 오차가 약 1000배 줄었다(20.8 → 0.022 mrad). 던지기 오차는 약 6000배 줄었다(159.9 → 0.026 mrad).
- **기존 던지기 baseline의 손목(관절 6) 오차는 0.16 rad(약 9°)였다.** 지금까지 "이상 토크 baseline"이라고 부른 결과에 공 질량 불일치로 인한 큰 자세 오차가 섞여 있었다. 앞으로 이상 토크 기준선은 `Gen3_J2_payload`로 바꾼다.
- **질량 설정이 틀린 만큼 오차가 비례한다.** +20% 오설정에서 정지 구간 오차는 공을 몰랐을 때의 −0.19배(관절 2), −0.19배(관절 4)다. 설정 오차에 선형이고 부호가 반대다.
- **전환 시점 확인:** `ctc_payload_on`은 정확히 t = 2.1 s(`t_release_plan`)에 1 → 0이 되었다. 놓은 뒤 오차도 0.01 mrad 수준이다.
- **실행 시간:** 느린 동작(8.5 s)은 약 2분(페이로드 모델), 던지기(4 s)는 5–29초. `run_p2p_ideal`이 1 ms 출력 시각을 강제해 실행당 10분 이상 걸리던 문제는 출력 보간(`Refine 10`)으로 고쳤다.

## 4. CTC의 반사 관성 옵션

설계 결정 2번("CTC는 액추에이터를 안다")의 구현이다. 감속기가 달린 모터는 관절에 반사 관성 Jr = N²·(Jm + J_WG)를 더한다. 관절 2의 Jr은 100²·(1.7×10⁻⁵ + 0.79×10⁻⁵) = 0.249 kg·m²다. 실제 제어기도 데이터시트의 회전자 관성을 알기 때문에 질량 행렬 대각에 더한다.

```
τ = (M(q) + diag(ctc_Jr))·a + C(q,q̇)·q̇ + g(q),   a = q̈_ref + Kp·e + Kd·ė
```

- `Gen3_J2_payload.slx`를 복사해 `Gen3_J2_ctc.slx`를 만든다. **앞으로 단계 3의 작업 모델은 이것이다**(슬롯 + 페이로드 + Jr 옵션).
- `ctc_Jr`는 7×1 벡터이고 기본값 0(꺼짐)이다. 액추에이터(회전자 포함)를 슬롯에 넣을 때만 `[0 0.249 0 0 0 0 0]'`로 켠다.
- 검증 `test_stage3_ctc_jr`:
  1. 꺼짐 상태가 `Gen3_J2_payload` 결과와 완전히 같은지 확인한다.
  2. 켜짐·꺼짐 모두에서, Simulink CTC 출력이 MATLAB에서 재계산한 `inverseDynamics + ctc_Jr.*a`와 같은지 확인한다.
  3. 참고: 켜짐 상태로 회전자가 없는 로봇을 돌리면 "제어기가 없는 회전자를 있다고 믿는" 불일치가 생긴다. 그 오차 크기를 기록한다.

```matlab
build_stage3_ctc_jr      % -> Gen3_J2_ctc.slx
test_stage3_ctc_jr       % 느린 동작 2회 (약 4분)
```

### 결과 (2026-11-21, 민혁 PC)

| 확인 | 값 | 판정 |
| --- | --- | --- |
| 옵션 꺼짐 대 `Gen3_J2_payload` (q, CTC 토크) | 차이 0, 0 | 완전히 같음 |
| CTC 출력 재계산 오차, 꺼짐 / 켜짐 | 0.0047 / 0.0016 N·m (피크 11.3 N·m의 0.04%) | 통과 (기준 0.005 N·m) |
| Jr·a₂ 피크 (CTC가 더하는 항) | 0.079 N·m | 재계산 오차의 약 17배라 빠지면 검출된다 |
| 켜짐과 꺼짐의 관절 2 토크 차이 피크 | 0.024 N·m | |
| 켜짐 + 회전자 없는 로봇의 최대 오차 | 관절 2 2.5, 관절 4 6.0, 전체 6.2 mrad | 참고 |

- 재계산 오차는 기록 신호를 솔버 스텝 사이에서 보간한 데서 생긴다. 통과 여유가 크지 않다(0.0047 대 0.005). 다만 Jr 항이 빠졌다면 0.079 N·m 수준의 차이가 나므로 검사는 의미가 있다.
- **제어기가 회전자 관성을 잘못 알면, 느린 동작에서도 수 mrad 오차가 생긴다.** 이번에는 없는 회전자를 있다고 믿는 경우였고, 여기서도 관절 4 오차(6.0)가 관절 2(2.5)보다 크다. 관절 2의 토크 오차가 질량 행렬 결합을 타고 퍼지기 때문이다. 던지기는 관절 가속도가 약 20배(10 rad/s² 대 0.5)라서 같은 불일치가 수십 mrad 수준이 될 것으로 예상한다. 액추에이터를 붙일 때 `ctc_Jr`를 켜야 하는 근거다.

## 5. 관절 2에 실제 액추에이터 넣기 (이상 토크 대 액추에이터)

`Gen3_J2_ctc.slx`를 복사해 `Gen3_J2_act.slx`를 만들고, `J2_Actuator_Slot/J2_Actuator`(지금까지 이상 통과)의 내용물을 `stage2_actuator/J2_Actuator.slx`의 내용물로 바꾼다. 단계 2 시험대가 쓰는 것과 같은 `copyContentsToSubSystem`이라서 1자유도 시험대에서 확인한 블록이 그대로 로봇에 들어간다.

```
CTC ──τ_cmd(7)──► J2_Actuator_Slot ──τ_applied(7)──► Gen3
                    관절 2: τ_cmd → iq → FOC → dq PMSM → 모터축(Jm+J_WG) → 감속기 스프링 Kg, cg → τ_J
                                                                  ▲ q_J, q̇_J (로봇에서)
```

| 바꾼 것 | 내용 | 이유 |
| --- | --- | --- |
| 모터축 초기 각도 정렬 | θm0 = s·N·(q0₂ + τ0₂/Kg), ωm0 = s·N·q̇0₂ (`j2_actuator_init.m`) | θm0 = 0이면 감속기가 −Kg·q0₂만큼 감긴 채 시작한다(q0₂ = 0.1 rad면 1400 N·m). 첫 CTC 토크 τ0만큼 미리 비틀어 두면 τ_J(0) = τ0라서 시작 순간 출렁임이 없다 |
| CTC 반사 관성 | `ctc_Jr = [0 0.249 0 0 0 0 0]'` (켜짐이 기본) | 설계 결정 2. 비교용으로 꺼짐도 돌린다 |
| 솔버 | `ode45`, MaxStep 2e-4 s, RelTol 1e-5 (이 모델만) | 1 kHz 전류 루프(약 7000 rad/s)와 약 40 Hz 감속기 모드가 10 rad/s CTC 루프와 같이 있다. explicit 솔버로도 이 스텝이면 안정하고 Jacobian 계산이 없다. 이상 모델들은 원래 설정 그대로 |
| 속도 | 로보틱스 블록 'Code generation', 쓰지 않는 IK 체인 주석 처리 | 아래 "실행 시간" |
| 로그 | `j2_act_log`(iq_cmd, iq, id, vd, vq, \|v\|, ωm, Te, τ_J, 비틀림)를 결과 MAT에 함께 저장 | `run_p2p_ideal`에 모델 파라미터 인자(5번째)를 추가했다 |

τ0는 CTC가 t = 0에 내는 토크를 같은 식으로 다시 계산한 값이다(공을 쥔 상태면 `robotPayload`). 입력이 바뀌면 `run_stage3_actuator`가 실행마다 다시 계산한다.

### 실행

```matlab
cd stage3_robot
build_stage3_actuator    % -> Gen3_J2_act.slx (Gen3_J2_ctc.slx 필요. J2_Actuator.slx가 없으면 단계 2 빌드부터 한다)
run_stage3_actuator      % 느린 동작: ideal(재사용 가능) + act_jr + act_nojr
```

- 필요한 모델 순서: `build_stage3_slot` → `build_stage3_payload` → `build_stage3_ctc_jr` → `build_stage3_actuator`. 저장소에는 `Gen3_J2_slot.slx`만 있다. 새로 받은 폴더에서는 `run_stage3_actuator`, `time_check_actuator`, 단계 3 테스트들이 빠진 모델(`ensure_stage3_model`)과 `p2p_input.mat`(`ensure_p2p_input`)을 처음 실행할 때 알아서 만든다.
- `ideal`은 `test_stage3_ctc_jr`가 남긴 `results/p2p_ctc_off_out.mat`가 있으면 재사용한다. 없으면 `Gen3_J2_ctc`(ctc_Jr 꺼짐)로 한 번 돌린다.
- 스텝 크기 영향 확인: `run_stage3_actuator(struct('solver',struct('MaxStep','1e-4'),'cases',{{'act_jr'}}))`.
- **실행 시간:** 처음에는 로보틱스 블록이 'Interpreted execution'이라 한 번 실행에 수 시간이 걸렸다. `build_stage3_actuator`가 이 블록들을 'Code generation'으로 바꾸고(`set_robot_blocks`, 결과는 같고 C 컴파일러 필요), Scope에만 연결된 IK 체인을 주석 처리한다. `time_check_actuator`(2026-11-24, 민혁 PC, 0.05 s 구간)로 설정을 비교한 결과는 아래와 같다. 그래서 기본 설정을 **ode23t, MaxStep 1e-3, RelTol 1e-4**로 정했다. `run_stage3_actuator`는 같은 사례를 스텝 1/5(2e-4)로 처음 2 s만 다시 돌려 차이도 기록한다(`stage3_actuator_convergence.csv`, 기준 0.01 mrad).

  | 설정 | 스텝당 | 8.5 s 예상 |
  | --- | --- | --- |
  | act, ode45 2e-4 | 558 ms | 약 400분 |
  | act, ode45 2e-4, 가속기 (두 번째 실행) | 13 ms | 약 10분 |
  | **act, ode23t 1e-3** | 34 ms | **약 7분** |
  | 참고: ideal, 자체 솔버 / ode45 2e-4 | 334 / 62 ms (대부분 초기화 15 s) | — |

| 사례 | 모델 | CTC의 ctc_Jr(2) |
| --- | --- | --- |
| `ideal` | `Gen3_J2_ctc` (이상 통과) | 0 |
| `act_jr` | `Gen3_J2_act` (액추에이터) | 0.249 |
| `act_nojr` | `Gen3_J2_act` (액추에이터) | 0 (CTC가 회전자를 모름) |

통과 기준(실패하면 결과를 쓴 뒤 멈춘다): 처음 50 ms 동안 |τ_J − τ_cmd| < 0.5 N·m(정렬 확인), iq 명령 피크 < 20.9 A, \|v\| 피크 < 12.47 V, 감속기 토크 피크 < 70 N·m(CSG-17-100 반복 피크). 추종 오차와 이상 대비 편차는 기준 없이 기록한다. 결과는 `results/stage3_actuator_compare.csv`, `.png`, 사례별 `p2p_<사례>_metrics.csv`, `_out.mat`.

### 예상 (MATLAB 실행 전, `predict_actuator_1dof.py`)

관절 2만 떼어 중력·결합은 CTC가 완벽히 보상한다고 가정한 1자유도 계산이다(J_L = 1.25 kg·m², Kp = 100, Kd = 20).

| 사례 | 관절 2 최대 오차 | 관성 토크에 의한 감속기 비틀림 |
| --- | --- | --- |
| `ideal` | 0 | — |
| `act_nojr` | 약 1.0 mrad | 0.05 mrad |
| `act_jr` | 약 0.002 mrad | 0.05 mrad |

- Gen3에서는 이 위에 페이로드 CTC의 잔여 오차(0.022 mrad)가 더해지고, 관절 2의 오차가 M24 결합으로 관절 4에도 퍼질 것이다(3a·4절과 같은 모양).
- 중력 토크(피크 11 N·m)는 감속기를 약 0.8 mrad 비틀지만, CTC 명령에 이미 들어 있고 저주파라 τ_J가 따라가므로 추종 오차로는 거의 나타나지 않을 것으로 본다.
- 이 동작에서 전류(약 3.7 A)와 전압(약 1.5 V)은 한계에 한참 못 미친다. 그래서 차이는 회전자 관성(ctc_Jr로 보상)과 감속기 탄성에서만 나온다.

### 결과

MATLAB 실행 후 여기에 채운다.
