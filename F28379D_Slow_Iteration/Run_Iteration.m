% Process the latest LVDAC export and apply one bounded PWM update.

packageDirectory = fileparts(mfilename('fullpath'));
matlabDirectory = fullfile(packageDirectory, 'MATLAB');
addpath(matlabDirectory);

textFile = fullfile(packageDirectory, 'Data', 'Measured_Output.txt');
csvFile = fullfile(packageDirectory, 'Data', 'Measured_Output.csv');
assert(~(isfile(csvFile) && isfile(textFile)), ...
    'Both input formats exist. Keep only the current export in Data.');
if isfile(csvFile)
    measuredFile = csvFile;
elseif isfile(textFile)
    measuredFile = textFile;
else
    error(['Save the latest LVDAC oscilloscope export as ', ...
        'Data\\Measured_Output.txt or Data\\Measured_Output.csv.']);
end

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
        fprintf('Iteration cancelled: no serial port was selected.\n');
        return;
    end
    boardPort = ports(portIndex);
end
fprintf('F28379D serial port: %s\n', boardPort);

answer = questdlg( ...
    ['Confirm that the selected file is the newest 0.20 s LVDAC ', ...
     'record, E1=V34, E2=V45 and E3=V53, and that the DC bus has ', ...
     'been reduced to 0 V while the next PWM cycle is calculated.'], ...
    'Run one slow iteration', 'Continue', 'Cancel', 'Cancel');
if ~strcmp(answer, 'Continue')
    fprintf('Iteration cancelled.\n');
    return;
end

report = run_slow_iteration(boardPort, measuredFile, ...
    PackageDirectory=packageDirectory);

fprintf(['\nAfter checking the report, restore the same DC input and ', ...
    'record the next 0.20 s output.\n']);

