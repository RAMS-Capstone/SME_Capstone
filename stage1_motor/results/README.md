# 단계 1 실행 결과

`run_stage1` 실행 후 생기는 아래 파일을 이 폴더에 둔다(2026-11-16 민혁 PC에서 실행, 모두 PASS).

- `J2_PMSM_Stage1.slx` — 직접 구현한 모터 + FOC 모델
- `J2_PMSM_SimscapeRef.slx` — Simscape PMSM 비교 기준 모델
- `stage1_test_results.csv`, `stage1_tests.png` — 시험 T1–T5
- `stage1_vs_simscape.csv`, `stage1_vs_simscape.png` — 직접 구현 대 Simscape 비교 (발표용 그림)

`*.slxc`와 `slprj/`는 캐시라 올리지 않는다.
