% RUN_STAGE3_P2P  Stage 3 items 2-3 in one go (needs Gen3_J2_slot.slx in this folder).
%   1) make_p2p_trajectory -> p2p_input.mat     slow motion, joint 2 +0.8 rad, joint 4 -0.8 rad
%   2) run_p2p_ideal       -> results/p2p_ideal*  ideal-torque reference run
%   3) analyze_inertia_p2p -> results/p2p_inertia* M22, M23, M24 vs reflected inertia
traj = make_p2p_trajectory();
res  = run_p2p_ideal();
R    = analyze_inertia_p2p();
