# MATLAB 없이 한 단계 1 검증

이 폴더에서 실행한다(numpy, matplotlib 필요).

```bash
python verify_stage1.py .                      # dq 모델 vs 독립 3상(abc) 모델 → stage1_verification.json/.png
python check_wiring.py ../build_stage1_pmsm.m  # 빌드 스크립트의 블록·연결 목록을 그대로 실행해 결선 확인
```

- `stage1_verification.json`: `test_stage1_pmsm.m`이 비교하는 기준값이다.
- `wiring_check.log`: 결선 확인 결과(WIRING MATCHES REFERENCE)다.
