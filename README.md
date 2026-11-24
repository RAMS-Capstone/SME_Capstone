# SME Capstone — 로봇 관절 액추에이터 모델링 및 시뮬레이션 기반 성능 평가

스마트모빌리티공학 종합설계 · 팀: 민혁, 동하, 희수

PMSM 모터·인버터·감속기를 포함한 로봇 관절 액추에이터를 모델링한다. 그리고 실제 구동 제약(전류·전압 한계, 감속기 탄성, 회전자 관성)이 Kinova Gen3 로봇팔의 동작 성능에 주는 영향을 시뮬레이션으로 평가한다. 이상적인 토크 입력 모델과 액추에이터 모델을 비교해 관절 추종 오차, 전류·전압 제한, 공의 릴리스 속도와 착지 위치 차이를 분석한다.

## 진행 현황 (2026-11-24)

| 단계 | 내용 | 상태 |
| --- | --- | --- |
| 1 | 모터 직접 구현: dq PMSM + FOC 전류 제어 | **완료** — 3상 모델·Simscape 블록과 일치 확인 |
| 2 | 액추에이터: 감속기(1:100, 비틀림 강성)·회전자 관성·토크→전류 변환, 1자유도 진자 시험 | **완료** — 12개 항목 Python 기준과 일치 (`stage2_actuator/`) |
| 3 | Gen3에 한 관절씩 통합(관절 2 → 4 → 1), 느린 point-to-point 동작 | **진행 중** — 관절 2에 액추에이터 삽입, 이상 토크와 비교 (`stage3_robot/` 5절) |
| 4 | 던지기: 릴리스 속도·착지 위치 비교 | 예정 |

week11 교수님 피드백: 모터를 "딸깍"하지 말고 직접 구현할 것. 던지기 전에 움직이는 것부터 확인할 것.

## 폴더 구조

```
├── docs/                 설계 근거 문서 (사양 결정, 요구량 분석, 후보 비교)
├── models/original/      팀이 만든 원본 Simulink 모델
│   ├── Gen3_throw_baseline.slx   (민혁) Gen3 + Computed Torque Controller, 이상 토크 baseline
│   └── PMSM_FOC_.slx             (동하) Simscape PMSM 블록 기반 FOC → 단계 1에서 검증 기준으로 사용
├── stage1_motor/         단계 1: 직접 구현한 모터 (아래 실행 방법)
│   ├── verification/     MATLAB 없이 한 Python 검증
│   └── results/          run_stage1 실행 결과물을 여기에 둔다
├── stage2_actuator/      단계 2: 관절 2 액추에이터(모터 + 감속기)와 1자유도 시험대
├── stage3_robot/         단계 3: Gen3 관절 2 슬롯, 궤적, 관성 분석, 액추에이터 통합
├── analysis/             요구량·후보 계산 스크립트
├── data/                 baseline 던지기 궤적의 관절 2 기록 (CSV)
└── legacy/               이전 AI 대화에서 만든 검증용 모델 (참고만, 파라미터가 현재 사양과 다름)
```

## 단계 1 실행 방법

필요: MATLAB + Simulink + Simscape Electrical (R2025b 이상에서 확인, 실행은 R2026b)

```matlab
cd stage1_motor
run_stage1
```

`run_stage1`은 다음 순서로 실행한다.

1. `build_stage1_pmsm` → `J2_PMSM_Stage1.slx` 생성. 직접 구현한 모터와 FOC 모델이다. 라이브러리 모터 블록 없이 Gain·Sum·Product·Integrator만 쓴다.
2. `test_stage1_pmsm` → 시험 T1–T5를 Python 기준값과 자동 비교한다(`stage1_test_results.csv`, `stage1_tests.png`).
3. `build_stage1_simscape_ref` → `J2_PMSM_SimscapeRef.slx` 생성. 같은 제어기에 Simscape PMSM 블록을 붙인 비교 기준 모델이다.
4. `compare_stage1_simscape` → 직접 만든 모터와 Simscape 블록을 비교한다(`stage1_vs_simscape.csv/.png`).

파라미터는 `stage1_motor/j2_params.m` 한 곳에서만 바꾼다. 값마다 [M] 제조사 / [C] 변환·계산 / [A] 가정으로 출처를 표시했다.

### 단계 1 결과 요약 (2026-11-16 MATLAB 실행)

| 비교 | 결과 |
| --- | --- |
| 시험 9개 항목 대 Python 기준 | 전부 통과 (전류 상승 0.35 ms, 최고 속도 429.1 rad/s 등) |
| 직접 구현 모터 대 Simscape PMSM | iq 최대 차이 3.5×10⁻⁴ A, 속도 3.2×10⁻⁵ rad/s |
| dq 모델 대 독립 3상(abc) 모델 (Python) | 최대 2.5×10⁻⁶ A |

## 기준 사양

| 항목 | 선택 | 근거 문서 |
| --- | --- | --- |
| 모터 | maxon EC-i 52 180 W, 24 V 권선(#574741) | `docs/03_관절2_기준사양_결정.md` |
| 감속기 | Harmonic Drive CSG-17-100 (1:100, 강성 K2 1.4×10⁴ N·m/rad) | 〃 |
| 전원·드라이브 | DC 24 V (Gen3 공칭), 전압 한계 12.47 V, 전류 명령 제한 20.9 A / 10.4 A | 〃 |

### 설계 결정

| 결정 | 선택 |
| --- | --- |
| 회전자 관성 위치 | 액추에이터 안에 모터축 상태를 따로 두고 감속기 스프링으로 관절에 연결. 대수 루프가 없다 |
| CTC가 액추에이터를 아는지 | 안다: M(q) 대각에 반사 관성 N²·(Jm+J_WG) ≈ 0.25 kg·m²를 더한다 |
| 부호 규약 | iq>0 → Te>0 → ωm>0, 감속기 방향 s = −1 (하모닉 드라이브 출력 반전) |

## 협업 규칙 (제안)

- `.slx`는 바이너리라 Git에서 병합이 안 된다. **같은 모델을 두 사람이 동시에 고치지 않는다.** 고치기 전에 단톡에 알린다.
- 모델은 가능하면 `build_*.m` 스크립트로 다시 만든다. 그래서 스크립트가 원본이고 `.slx`는 결과물이다.
- `*.slxc`, `slprj/`는 MATLAB 캐시라 올리지 않는다(`.gitignore` 처리).
- 결과 그림·CSV는 각 단계의 `results/`에 둔다.
