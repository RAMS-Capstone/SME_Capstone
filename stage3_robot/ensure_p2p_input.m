function f = ensure_p2p_input()
% ENSURE_P2P_INPUT  Path of p2p_input.mat next to this file; makes it first if it is missing
%   (fresh checkout). The file is generated, not committed: make_p2p_trajectory with its defaults.
here = fileparts(mfilename('fullpath'));
f = fullfile(here,'p2p_input.mat');
if ~isfile(f)
    fprintf('p2p_input.mat not found -> make_p2p_trajectory (default slow motion)\n');
    make_p2p_trajectory();
end
end
