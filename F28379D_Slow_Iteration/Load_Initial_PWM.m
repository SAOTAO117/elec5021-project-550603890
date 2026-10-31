% Load the initial 50 Hz open-loop PWM cycle.

packageDirectory = fileparts(mfilename('fullpath'));
addpath(fullfile(packageDirectory, 'MATLAB'));

ports = string(serialportlist('available'));
assert(~isempty(ports), 'No serial port is available.');
if isscalar(ports)
    boardPort = ports(1);
else
    defaultPort = find(strcmpi("COM6", ports), 1);
    if isempty(defaultPort)
        defaultPort = 1;
    end
    [portIndex, selected] = listdlg( ...
        'PromptString', 'Select the F28379D serial port:', ...
        'SelectionMode', 'single', ...
        'ListString', cellstr(ports), ...
        'InitialValue', defaultPort);
    if ~selected
        fprintf('PWM load cancelled: no serial port was selected.\n');
        return;
    end
    boardPort = ports(portIndex);
end
fprintf('F28379D serial port: %s\n', boardPort);

answer = questdlg( ...
    ['Keep the inverter DC bus at 0 V. This loads a 50 Hz cycle ', ...
     'with 20 kHz PWM and 0.10 modulation.'], ...
    'Load initial PWM', 'Continue', 'Cancel', 'Cancel');
if ~strcmp(answer, 'Continue')
    fprintf('PWM load cancelled.\n');
    return;
end

time = (0:399).' * 0.00005;
phaseA = 0.10 * sin(2 * pi * 50 * time);
phaseB = 0.10 * sin(2 * pi * 50 * time - 2 * pi / 3);
phaseC = 0.10 * sin(2 * pi * 50 * time + 2 * pi / 3);
phaseReference = [phaseA, phaseB, phaseC];
commonMode = -0.5 * (max(phaseReference, [], 2) + ...
    min(phaseReference, [], 2));
svpwmReference = phaseReference + commonMode;
activeDuty = 0.5 + 0.5 * svpwmReference;
activeDuty = min(max(activeDuty, 0.05), 0.95);
activeDutyEncoded = int16(round(activeDuty * 8192));

uploadStatus = upload_pwm_sequence(boardPort, activeDutyEncoded, ...
    BaudRate=115200);
assert(uploadStatus.loaded, 'The initial PWM cycle was not activated.');

stateDirectory = fullfile(packageDirectory, 'State');
if ~isfolder(stateDirectory)
    mkdir(stateDirectory);
end
save(fullfile(stateDirectory, 'Active_PWM_Sequence.mat'), ...
    'activeDuty', 'activeDutyEncoded', 'uploadStatus');

sequenceTable = table(time, activeDuty(:, 1), activeDuty(:, 2), ...
    activeDuty(:, 3), double(activeDutyEncoded(:, 1)), ...
    double(activeDutyEncoded(:, 2)), ...
    double(activeDutyEncoded(:, 3)), ...
    'VariableNames', {'Time_s', 'DutyA', 'DutyB', 'DutyC', ...
    'DutyA_Q13', 'DutyB_Q13', 'DutyC_Q13'});
writetable(sequenceTable, fullfile(stateDirectory, ...
    'Active_PWM_Sequence.csv'));

fprintf('\nInitial PWM loaded.\n');
fprintf('Fundamental: 50 Hz; PWM: 20 kHz; modulation: 0.10.\n');
fprintf('Dead time remains 1.5 microseconds in the ePWM hardware.\n');

