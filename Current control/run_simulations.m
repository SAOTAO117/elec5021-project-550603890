function run_simulations(selected)
%RUN_SIMULATIONS Run the saved test configurations without changing parameters.
root = fileparts(mfilename('fullpath'));
models = {'DevelopmentCurrentSPWM.slx'; ...
    'DevelopmentCurrentSVPWM.slx'; ...
    'Current step up test/CurrentStepUp.slx'; ...
    'Current step down test/CurrentStepDown.slx'; ...
    'DQStartup.slx'; ...
    'Nonzero q current test/DQNonzero.slx'; ...
    'Nonzero q current test/DQFixedFrame.slx'; ...
    'PLL test/PLLKnownInput.slx'; ...
    'Varying current reference test/PeriodicCurrent.slx'};
if nargin > 0
    keep = false(size(models));
    for j = 1:numel(models)
        [~, name] = fileparts(models{j});
        keep(j) = any(strcmp(name, selected));
    end
    models = models(keep);
    assert(~isempty(models), 'No matching model was selected.');
end
for k = 1:numel(models)
    file = fullfile(root, models{k});
    [folder, model] = fileparts(file);
    assert(~bdIsLoaded(model), 'Close the model %s before running this script.', model);
    load_system(file);
    cleanup = onCleanup(@() close_system(model, 0));
    output = fullfile(folder, 'Results');
    if ~isfolder(output), mkdir(output); end
    result = sim(model, 'ReturnWorkspaceOutputs', 'on');
    s = struct;
    if any(strcmp(result.who, 'logsout'))
        logs = result.logsout;
        for j = 1:logs.numElements
            e = logs.get(j);
            s.(matlab.lang.makeValidName(e.Name)) = struct( ...
                'time', e.Values.Time, 'data', e.Values.Data);
        end
    end
    settings = struct('model', model, 'stop_time_s', str2double(get_param(model, 'StopTime')));
    switch model
        case 'CurrentStepUp', settings.reference_before=1; settings.reference_after=1.2; settings.q_reference=0;
        case 'CurrentStepDown', settings.reference_before=1; settings.reference_after=.8; settings.q_reference=0;
        case 'DQStartup', settings.reference_before=1.2; settings.reference_after=1.2; settings.q_reference=0;
        case {'DQNonzero','DQFixedFrame'}, settings.reference_before=1; settings.reference_after=1; settings.q_reference=.2;
        case 'PLLKnownInput'
            workspace = get_param(model, 'ModelWorkspace');
            input = getVariable(workspace, 'pllInput');
            t = input(:,1);
            s = struct('time',t,'frequency',50+2*(t>=.2), ...
                'angle',2*pi*(50*t+2*max(t-.2,0))+(pi/9)*(t>=.4), ...
                'voltage',input(:,2:4),'estimated_frequency',result.frequency_Hz, ...
                'estimated_angle',result.angle_rad,'magnitude',result.magnitude_pu);
    end
    save(fullfile(output, [model '.mat']), 's', 'settings', 'result', '-v7.3');
    fprintf('Saved %s\n', fullfile(output, [model '.mat']));
    clear cleanup result s
end
end
