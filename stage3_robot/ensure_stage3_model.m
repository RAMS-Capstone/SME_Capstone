function ensure_stage3_model(name)
% ENSURE_STAGE3_MODEL  Builds a stage-3 model (and the ones it is copied from) if its .slx is missing.
%   Chain: Gen3_J2_slot -> Gen3_J2_payload -> Gen3_J2_ctc -> Gen3_J2_act
%   Only Gen3_J2_slot.slx is committed; a fresh checkout builds the rest on first use.
here = fileparts(mfilename('fullpath'));
chain = {'Gen3_J2_slot',    @build_stage3_slot;
         'Gen3_J2_payload', @build_stage3_payload;
         'Gen3_J2_ctc',     @build_stage3_ctc_jr;
         'Gen3_J2_act',     @build_stage3_actuator};
k = find(strcmp(chain(:,1), name));
assert(~isempty(k), 'Unknown stage-3 model %s', name);
for i = 1:k
    if ~isfile(fullfile(here,[chain{i,1} '.slx']))
        fprintf('%s.slx not found -> %s\n', chain{i,1}, func2str(chain{i,2}));
        chain{i,2}();
    end
end
end
