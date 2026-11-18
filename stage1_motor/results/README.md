# 단계 1 실행 결과

`run_stage1` 실행 결과물이다(2026-11-16 민혁 PC, MATLAB R2026b, 모두 PASS).

- `J2_PMSM_Stage1.slx` — 직접 구현한 모터 + FOC 모델
- `J2_PMSM_SimscapeRef.slx` — Simscape PMSM 비교 기준 모델
- `stage1_test_results.csv` — 시험 T1–T5 (Simulink 대 Python 기준, 9개 항목 모두 Pass)
- `stage1_vs_simscape.png` — 직접 구현 대 Simscape 비교 (발표용 그림). iq 최대 차이 3.5e-4 A, 속도 3.2e-5 rad/s

아직 없는 것: `stage1_tests.png`, `stage1_vs_simscape.csv` (다시 실행하면 생긴다)

`*.slxc`와 `slprj/`는 캐시라 올리지 않는다.
