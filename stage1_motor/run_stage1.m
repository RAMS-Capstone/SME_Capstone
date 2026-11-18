% RUN_STAGE1  One-click stage 1: build both models, run the tests, compare with Simscape.
% Put all stage-1 .m files in one folder, cd there, then run:  run_stage1
% Needs Simulink + Simscape Electrical (same toolboxes as Dong-ha's PMSM_FOC_.slx).
build_stage1_pmsm;              % -> J2_PMSM_Stage1.slx      (hand-built motor + FOC)
results = test_stage1_pmsm;     % -> stage1_test_results.csv, stage1_tests.png
build_stage1_simscape_ref;      % -> J2_PMSM_SimscapeRef.slx (library PMSM, same params)
cmp = compare_stage1_simscape;  % -> stage1_vs_simscape.csv/.png
