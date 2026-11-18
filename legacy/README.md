# legacy — 이전 AI 대화에서 만든 검증용 모델 (참고만)

다른 AI 도구에서 작업할 때 만든 파일이다. **현재 사양과 맞지 않으니 실행용으로 쓰지 않는다.**

| 파일 | 상태 |
| --- | --- |
| `build_gen3_pmsm.m` | 원본 모델 `PMSM_1DOF_position_compare_v1`이 필요한데 저장소에 없다. 파라미터는 예전 검증값(Kt=0.27, N=2, Imax=1 A)이다 |
| `Gen3_J2_PMSM_Actuator.slx`, `Gen3_J2_PMSM_Standalone.slx` | 위 스크립트의 결과물이다 |
| `test_gen3_pmsm.m` | 시험 항목(정·역토크, 포화, 전압 포화 복귀, 기어 전력 보존, 역방향 좌표)은 단계 2 시험을 설계할 때 참고한다 |

새 작업은 `stage1_motor/`부터 이어간다.
