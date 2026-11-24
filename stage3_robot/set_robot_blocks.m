function n = set_robot_blocks(model, mode)
% SET_ROBOT_BLOCKS  How the Robotics System Toolbox blocks of a model run in simulation.
%   The Gen3 model's CTC blocks (Gravity Torque, Joint Space Mass Matrix, Velocity Product Torque,
%   and their payload twins) and the IK chain were set to 'Interpreted execution': every call runs
%   MATLAB code. With the actuator the solver takes 0.1-0.2 ms steps, i.e. ~50,000 steps x ~6
%   evaluations x 6 blocks for the slow motion, which makes one run take hours.
%   'Code generation' compiles the same algorithm once (needs a C compiler, see mex -setup) and
%   gives the same numbers.
%   mode : 'Code generation' (default) or 'Interpreted execution'
%   n    : number of blocks changed
if nargin < 2, mode = 'Code generation'; end
blks = find_system(model, 'LookUnderMasks','all', 'FollowLinks','on', 'RegExp','on', ...
                   'SimulateUsing','.');
n = 0;
for k = 1:numel(blks)
    try
        if ~strcmp(get_param(blks{k},'SimulateUsing'), mode)
            set_param(blks{k}, 'SimulateUsing', mode); n = n + 1;
        end
    catch
        % parameter exists but is not settable on this block: leave it
    end
end
fprintf('%s: %d robotics block(s) set to ''%s''\n', model, n, mode);
end
