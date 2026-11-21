function [stage1Model, paramsDir] = find_stage1(here, stage1Model)
% FIND_STAGE1  Locates the stage-1 files the actuator reuses and puts j2_params.m on the path.
%   J2_PMSM_Stage1.slx : argument -> ../stage1_motor/results -> ../stage1_motor -> this folder -> MATLAB path
%   j2_params.m        : ../stage1_motor -> this folder -> MATLAB path
name = 'J2_PMSM_Stage1.slx';
if nargin < 2, stage1Model = ''; end
cands = {stage1Model, fullfile(here,'..','stage1_motor','results',name), ...
         fullfile(here,'..','stage1_motor',name), fullfile(here,name), which(name)};
found = '';
for k = 1:numel(cands)
    if ~isempty(cands{k}) && isfile(cands{k}), found = cands{k}; break; end
end
assert(~isempty(found), ['J2_PMSM_Stage1.slx not found. Pass its full path, e.g. ' ...
    'build_stage2_actuator(''C:/.../J2_PMSM_Stage1.slx'')']);
stage1Model = found;

pcands = {fullfile(here,'..','stage1_motor','j2_params.m'), fullfile(here,'j2_params.m'), which('j2_params')};
paramsDir = '';
for k = 1:numel(pcands)
    if ~isempty(pcands{k}) && isfile(pcands{k}), paramsDir = fileparts(pcands{k}); break; end
end
assert(~isempty(paramsDir), 'j2_params.m not found (stage1_motor folder). Put it next to this script.');
addpath(paramsDir);
end
