function report = send_data(portName, options)
%SEND_DATA Compare F28379D and Simulink using measured LabVolt waveforms.

arguments
    portName (1,1) string
    options.FilePath (1,1) string = ""
    options.ModelFile (1,1) string = ""
    options.ResultsDirectory (1,1) string = ""
    options.VoltageRangePeak (1,1) double ...
        {mustBeFinite, mustBePositive} = 80
    options.CurrentRangePeak (1,1) double ...
        {mustBeFinite, mustBePositive} = 0.04
    options.ControllerRateHz (1,1) double ...
        {mustBeFinite, mustBePositive} = 20000
    options.BaudRate (1,1) double ...
        {mustBeInteger, mustBePositive} = 115200
    options.TransferRateHz (1,1) double ...
        {mustBeFinite, mustBePositive} = 10
    options.ResponseTimeoutSeconds (1,1) double ...
        {mustBeFinite, mustBePositive} = 0.25
    options.ExpectedFrequencyHz (1,1) double ...
        {mustBeFinite, mustBePositive} = 50
    options.FrequencyToleranceHz (1,1) double ...
        {mustBeFinite, mustBePositive} = 2
    options.IdReferencePerUnit (1,1) double ...
        {mustBeFinite} = 0.253
    options.IdTolerancePerUnit (1,1) double ...
        {mustBeFinite, mustBePositive} = 0.05
    options.IqTolerancePerUnit (1,1) double ...
        {mustBeFinite, mustBePositive} = 0.05
    options.ReferenceHistoryFile (1,1) string = ""
    options.MinimumSourceDurationSeconds (1,1) double ...
        {mustBeFinite, mustBeNonnegative} = 0
    options.RepeatLastCycleToSeconds (1,1) double ...
        {mustBeFinite, mustBeNonnegative} = 0
    options.CreateIterationDirectory (1,1) logical = true
    options.DryRun (1,1) logical = false
end

scriptDir = fileparts(mfilename('fullpath'));
packageDir = fileparts(scriptDir);
if strlength(options.FilePath) == 0
    dataCandidates = [ ...
        string(fullfile(packageDir, 'Data', 'Measured_Output.csv')), ...
        string(fullfile(packageDir, 'Data', 'Measured_Output.txt'))];
    availableData = dataCandidates(isfile(dataCandidates));
    assert(~isempty(availableData), ...
        ['No measured output file was found. Export the LVDAC record as ', ...
         'Data/Measured_Output.csv or Data/Measured_Output.txt.']);
    assert(isscalar(availableData), ...
        'Both measurement formats exist. Pass FilePath explicitly.');
    options.FilePath = availableData(1);
end
if strlength(options.ModelFile) == 0
    options.ModelFile = string(fullfile(packageDir, 'Model', ...
        'IterativeController.slx'));
end
if strlength(options.ResultsDirectory) == 0
    options.ResultsDirectory = string(fullfile(packageDir, 'Results'));
end

assert(isfile(options.FilePath), ...
    'LabVolt data file not found: %s', options.FilePath);
assert(isfile(options.ModelFile), ...
    'Controller model not found: %s', options.ModelFile);
if options.CreateIterationDirectory
    options.ResultsDirectory = nextIterationDirectory( ...
        options.ResultsDirectory);
elseif ~isfolder(options.ResultsDirectory)
    mkdir(options.ResultsDirectory);
end

inputFrameBytes = 16;
resultFrameBytes = 16;
serialUtilisation = (inputFrameBytes + resultFrameBytes) * 10 * ...
    options.TransferRateHz / options.BaudRate;
assert(serialUtilisation <= 0.8, ...
    'The requested serial transfer uses %.1f%% of the line.', ...
    100 * serialUtilisation);

[sourceTimes, measured] = readLabVoltData(options.FilePath);
assert(sourceTimes(end) - sourceTimes(1) >= ...
    options.MinimumSourceDurationSeconds, ...
    ['The measured record is %.6f s. This run requires at least ', ...
     '%.6f s.'], sourceTimes(end) - sourceTimes(1), ...
    options.MinimumSourceDurationSeconds);
sourceSampleRateHz = 1 / median(diff(sourceTimes));
controllerTimes = makeControllerTimes(sourceTimes, ...
    options.ControllerRateHz);
measuredLineVoltages = interp1(sourceTimes, measured(:, 1:3), ...
    controllerTimes, 'linear');
phaseCurrents = interp1(sourceTimes, measured(:, 4:6), ...
    controllerTimes, 'linear');
repeatedCycleSamples = 0;
if options.RepeatLastCycleToSeconds > 0
    [controllerTimes, measuredLineVoltages, phaseCurrents, ...
        repeatedCycleSamples] = repeatLastMeasuredCycle( ...
        measuredLineVoltages, phaseCurrents, options.ControllerRateHz, ...
        options.ExpectedFrequencyHz, ...
        options.RepeatLastCycleToSeconds);
end
% E1=V34, E2=V45 and E3=V53. Recover the zero-sequence-free
% phase voltages required by the controller model.
phaseVoltages = lineToPhaseVoltages(measuredLineVoltages);

inputPerUnit = [phaseVoltages ./ options.VoltageRangePeak, ...
    phaseCurrents ./ options.CurrentRangePeak];
previousInputPerUnit = zeros(0, 6);
if strlength(options.ReferenceHistoryFile) > 0 && ...
        isfile(options.ReferenceHistoryFile)
    stored = load(options.ReferenceHistoryFile, 'inputHistoryPerUnit');
    assert(isfield(stored, 'inputHistoryPerUnit') && ...
        size(stored.inputHistoryPerUnit, 2) == 6, ...
        'The controller input-history file is invalid.');
    previousInputPerUnit = stored.inputHistoryPerUnit;
end
phaseAlignmentShiftSamples = 0;
if ~isempty(previousInputPerUnit) && ...
        options.RepeatLastCycleToSeconds > 0
    expectedCycleSamples = round(options.ControllerRateHz / ...
        options.ExpectedFrequencyHz);
    [inputPerUnit, phaseAlignmentShiftSamples] = ...
        alignPeriodicInput(inputPerUnit, previousInputPerUnit, ...
        expectedCycleSamples);
end

q13Limit = (2^15 - 1) / 2^13;
maximumInputMagnitude = max(abs(inputPerUnit), [], 'all');
assert(maximumInputMagnitude <= q13Limit, ...
    ['At least one input reaches %.6g p.u., outside the Q13 ', ...
     'range of +/-%.6g p.u.'], maximumInputMagnitude, q13Limit);

inputEncoded = encodeQ13(inputPerUnit);
inputPerUnit = double(inputEncoded) / 8192;
referenceInputPerUnit = [previousInputPerUnit; inputPerUnit];
referenceHistoryPerUnit = simulateReference(referenceInputPerUnit, ...
    options.ModelFile, 1 / options.ControllerRateHz);
referencePerUnit = referenceHistoryPerUnit( ...
    size(previousInputPerUnit, 1) + (1:size(inputPerUnit, 1)), :);
referenceEncoded = encodeQ13(referencePerUnit);

frameCount = size(inputEncoded, 1);
report = struct();
report.inputOrigin = "LabVolt 9063 measured line voltages and phase currents";
report.voltageInputMapping = "E1=V34, E2=V45, E3=V53";
report.dataFile = options.FilePath;
report.modelFile = options.ModelFile;
report.sourceRows = numel(sourceTimes);
report.sourceDurationSeconds = sourceTimes(end) - sourceTimes(1);
report.sourceSampleRateHz = sourceSampleRateHz;
report.controllerRows = frameCount;
report.controllerRateHz = options.ControllerRateHz;
report.repeatedCycleSamples = repeatedCycleSamples;
report.repeatedDurationSeconds = options.RepeatLastCycleToSeconds;
report.phaseAlignmentShiftSamples = phaseAlignmentShiftSamples;
report.transferRateHz = options.TransferRateHz;
report.expectedTransferSeconds = max(0, frameCount - 1) / ...
    options.TransferRateHz;
report.maximumInputMagnitudePerUnit = maximumInputMagnitude;
report.previousControllerRows = size(previousInputPerUnit, 1);
report.serialUtilisation = serialUtilisation;
report.dryRun = options.DryRun;
report.voltageRangePeak = options.VoltageRangePeak;
report.currentRangePeak = options.CurrentRangePeak;
report.expectedFrequencyHz = options.ExpectedFrequencyHz;
report.idReferencePerUnit = options.IdReferencePerUnit;
report.resultsDirectory = options.ResultsDirectory;

fprintf('LabVolt data: %d rows over %.6f s (%.3f Hz)\n', ...
    report.sourceRows, report.sourceDurationSeconds, ...
    report.sourceSampleRateHz);
fprintf('Controller inputs: %d rows at %.0f Hz virtual time\n', ...
    frameCount, options.ControllerRateHz);
fprintf('Serial transfer: %.0f frames/s, approximately %.1f s\n', ...
    options.TransferRateHz, report.expectedTransferSeconds);

if options.DryRun
    report.referencePerUnit = referencePerUnit;
    report.referenceEncoded = referenceEncoded;
    report.framesReturned = 0;
    return;
end

availablePorts = string(serialportlist('available'));
assert(any(strcmpi(portName, availablePorts)), ...
    'Serial port %s is not available. Available ports: %s', ...
    portName, strjoin(availablePorts, ', '));

device = serialport(portName, options.BaudRate, ...
    'Timeout', options.ResponseTimeoutSeconds);
cleanupObj = onCleanup(@() closeSerial(device));
flush(device);

boardEncoded = zeros(frameCount, 6, 'int16');
sentAt = nan(frameCount, 1);
returnedAt = nan(frameCount, 1);
scheduledAt = (0:frameCount - 1).' / options.TransferRateHz;
rxBuffer = uint8([]);
sequence = int16(0);
timer = tic;

for row = 1:frameCount
    waitUntil(timer, scheduledAt(row));
    sequence = nextSequence(sequence);
    payload = [sequence, inputEncoded(row, :)];
    frame = [uint8('S'), int16ToLittleEndian(payload), uint8('E')];
    assert(numel(frame) == inputFrameBytes, ...
        'Internal input-frame length error.');

    sentAt(row) = toc(timer);
    write(device, frame, 'uint8');
    [received, values, rxBuffer] = waitForResult(device, ...
        rxBuffer, sequence, options.ResponseTimeoutSeconds);
    assert(received, ...
        ['No result was returned for sequence %d. The controller ', ...
         'state must be reset before this test is repeated.'], sequence);
    boardEncoded(row, :) = values;
    returnedAt(row) = toc(timer);
end

elapsedSeconds = toc(timer);
clear cleanupObj;
closeSerial(device);

if strlength(options.ReferenceHistoryFile) > 0
    historyDirectory = fileparts(options.ReferenceHistoryFile);
    if ~isfolder(historyDirectory)
        mkdir(historyDirectory);
    end
    inputHistoryPerUnit = referenceInputPerUnit;
    save(options.ReferenceHistoryFile, 'inputHistoryPerUnit');
end

report.framesReturned = frameCount;
report.elapsedSeconds = elapsedSeconds;
report.achievedTransferRateHz = (frameCount - 1) / ...
    (sentAt(end) - sentAt(1));
report.meanRoundTripMilliseconds = ...
    1e3 * mean(returnedAt - sentAt);
report.maximumRoundTripMilliseconds = ...
    1e3 * max(returnedAt - sentAt);

report = compareResults(report, controllerTimes, measuredLineVoltages, ...
    phaseVoltages, phaseCurrents, boardEncoded, ...
    referenceEncoded, ...
    options.ResultsDirectory, options);

fprintf('Returned frames: %d/%d\n', report.framesReturned, frameCount);
fprintf('Mean/max round trip: %.3f/%.3f ms\n', ...
    report.meanRoundTripMilliseconds, ...
    report.maximumRoundTripMilliseconds);
fprintf('Maximum errors [angle id iq dutyA dutyB dutyC], Q13 counts:\n');
fprintf('  %s\n', strjoin(string(report.maximumAbsoluteErrorCounts), '  '));
fprintf('Board/Simulink calculation agreement: %s\n', report.result);
fprintf('Hardware-control assessment: %s\n', ...
    report.hardwareAssessment);
fprintf('Overall assessment: %s\n', report.overallAssessment);
end

function report = compareResults(report, timeSeconds, ...
        measuredLineVoltages, phaseVoltages, phaseCurrents, boardEncoded, ...
        referenceEncoded, ...
        resultsDirectory, options)
errorCounts = double(boardEncoded) - double(referenceEncoded);
errorCounts(:, 1) = mod(errorCounts(:, 1) + 4096, 8192) - 4096;
maximumAbsoluteErrorCounts = max(abs(errorCounts), [], 1);
rootMeanSquareErrorCounts = sqrt(mean(errorCounts.^2, 1));
toleranceCounts = [2, 2, 2, 16, 16, 16];
algorithmPassed = all(maximumAbsoluteErrorCounts <= toleranceCounts);

boardPerUnit = double(boardEncoded) / 8192;
referencePerUnit = double(referenceEncoded) / 8192;
boardAngleDegrees = 360 * mod(boardPerUnit(:, 1), 1);
referenceAngleDegrees = 360 * mod(referencePerUnit(:, 1), 1);
boardIdAmperes = boardPerUnit(:, 2) * options.CurrentRangePeak;
boardIqAmperes = boardPerUnit(:, 3) * options.CurrentRangePeak;

angleCycles = mod(double(boardEncoded(:, 1)), 8192) / 8192;
wrapIndices = find(diff(angleCycles) < -0.5);
assert(numel(wrapIndices) >= 2, ...
    ['The measured record does not contain two complete PLL cycles. ', ...
     'Record at least 0.10 s; 0.20 s is recommended.']);
cycleStart = wrapIndices(end - 1) + 1;
cycleEnd = wrapIndices(end);
cycleIndices = (cycleStart:cycleEnd).';
cycleSamples = wrapIndices(end) - wrapIndices(end - 1);
assert(cycleSamples <= 2048, ...
    'The selected PWM cycle has %d samples; the board limit is 2048.', ...
    cycleSamples);

measuredFrequencyHz = options.ControllerRateHz / cycleSamples;
lineVoltageRms = sqrt(mean( ...
    measuredLineVoltages(cycleIndices, :).^2, 1));
phaseVoltageRms = sqrt(mean(phaseVoltages(cycleIndices, :).^2, 1));
phaseCurrentRms = sqrt(mean(phaseCurrents(cycleIndices, :).^2, 1));
phaseCurrentPeak = max(abs(phaseCurrents(cycleIndices, :)), [], 1);
lineVoltageThdPercent = calculateCycleThd( ...
    measuredLineVoltages(cycleIndices, :), 50);
phaseCurrentThdPercent = calculateCycleThd( ...
    phaseCurrents(cycleIndices, :), 50);
idMeanPerUnit = mean(boardPerUnit(cycleIndices, 2));
iqMeanPerUnit = mean(boardPerUnit(cycleIndices, 3));
idRipplePerUnit = std(boardPerUnit(cycleIndices, 2), 0);
iqRipplePerUnit = std(boardPerUnit(cycleIndices, 3), 0);
dutyMinimum = min(boardPerUnit(cycleIndices, 4:6), [], 1);
dutyMaximum = max(boardPerUnit(cycleIndices, 4:6), [], 1);
dutySequenceEncoded = boardEncoded(cycleIndices, 4:6);

frequencyPassed = abs(measuredFrequencyHz - ...
    options.ExpectedFrequencyHz) <= options.FrequencyToleranceHz;
idPassed = abs(idMeanPerUnit - options.IdReferencePerUnit) <= ...
    options.IdTolerancePerUnit;
iqPassed = abs(iqMeanPerUnit) <= options.IqTolerancePerUnit;
hardwarePassed = frequencyPassed && idPassed && iqPassed;

report.maximumAbsoluteErrorCounts = maximumAbsoluteErrorCounts;
report.rootMeanSquareErrorCounts = rootMeanSquareErrorCounts;
report.toleranceCounts = toleranceCounts;
report.algorithmPassed = algorithmPassed;
report.measuredFrequencyHz = measuredFrequencyHz;
report.phaseVoltageRms = phaseVoltageRms;
report.lineVoltageRms = lineVoltageRms;
report.phaseCurrentRms = phaseCurrentRms;
report.phaseCurrentPeak = phaseCurrentPeak;
report.lineVoltageThdPercent = lineVoltageThdPercent;
report.phaseCurrentThdPercent = phaseCurrentThdPercent;
report.idMeanPerUnit = idMeanPerUnit;
report.iqMeanPerUnit = iqMeanPerUnit;
report.idMeanAmperes = idMeanPerUnit * options.CurrentRangePeak;
report.iqMeanAmperes = iqMeanPerUnit * options.CurrentRangePeak;
report.idRipplePerUnit = idRipplePerUnit;
report.iqRipplePerUnit = iqRipplePerUnit;
report.dutyMinimum = dutyMinimum;
report.dutyMaximum = dutyMaximum;
report.frequencyPassed = frequencyPassed;
report.idPassed = idPassed;
report.iqPassed = iqPassed;
report.hardwarePassed = hardwarePassed;
report.passed = algorithmPassed && hardwarePassed;
report.cycleStartSeconds = timeSeconds(cycleStart);
report.cycleEndSeconds = timeSeconds(cycleEnd);
report.pwmSequenceSamples = cycleSamples;
report.dutySequenceEncoded = dutySequenceEncoded;
if algorithmPassed
    report.result = "PASS";
else
    report.result = "REVIEW";
end
if hardwarePassed
    report.hardwareAssessment = "PASS";
else
    report.hardwareAssessment = "REVIEW";
end
if report.passed
    report.overallAssessment = "PASS";
else
    report.overallAssessment = "REVIEW";
end

resultTable = table(timeSeconds, ...
    measuredLineVoltages(:, 1), measuredLineVoltages(:, 2), ...
    measuredLineVoltages(:, 3), ...
    phaseVoltages(:, 1), phaseVoltages(:, 2), phaseVoltages(:, 3), ...
    phaseCurrents(:, 1), phaseCurrents(:, 2), phaseCurrents(:, 3), ...
    boardAngleDegrees, referenceAngleDegrees, ...
    boardPerUnit(:, 2), referencePerUnit(:, 2), ...
    boardIdAmperes, ...
    boardPerUnit(:, 3), referencePerUnit(:, 3), ...
    boardIqAmperes, ...
    boardPerUnit(:, 4), referencePerUnit(:, 4), ...
    boardPerUnit(:, 5), referencePerUnit(:, 5), ...
    boardPerUnit(:, 6), referencePerUnit(:, 6), ...
    errorCounts(:, 1), errorCounts(:, 2), errorCounts(:, 3), ...
    errorCounts(:, 4), errorCounts(:, 5), errorCounts(:, 6), ...
    'VariableNames', {'Time_s', ...
    'MeasuredV34_V', 'MeasuredV45_V', 'MeasuredV53_V', ...
    'ControllerVa_V', 'ControllerVb_V', 'ControllerVc_V', ...
    'MeasuredIa_A', 'MeasuredIb_A', 'MeasuredIc_A', ...
    'BoardAngle_deg', 'ReferenceAngle_deg', ...
    'BoardId_pu', 'ReferenceId_pu', 'BoardId_A', ...
    'BoardIq_pu', 'ReferenceIq_pu', 'BoardIq_A', ...
    'BoardDutyA', 'ReferenceDutyA', ...
    'BoardDutyB', 'ReferenceDutyB', ...
    'BoardDutyC', 'ReferenceDutyC', ...
    'AngleError_count', 'IdError_count', 'IqError_count', ...
    'DutyAError_count', 'DutyBError_count', 'DutyCError_count'});

sequenceTimeSeconds = (0:cycleSamples - 1).' / options.ControllerRateHz;
pwmSequenceTable = table(sequenceTimeSeconds, ...
    double(dutySequenceEncoded(:, 1)) / 8192, ...
    double(dutySequenceEncoded(:, 2)) / 8192, ...
    double(dutySequenceEncoded(:, 3)) / 8192, ...
    dutySequenceEncoded(:, 1), dutySequenceEncoded(:, 2), ...
    dutySequenceEncoded(:, 3), ...
    'VariableNames', {'Time_s', 'DutyA', 'DutyB', 'DutyC', ...
    'DutyA_Q13', 'DutyB_Q13', 'DutyC_Q13'});

csvFile = fullfile(resultsDirectory, ...
    'Measured_Output_and_Controller_Data.csv');
sequenceFile = fullfile(resultsDirectory, 'PWM_Sequence.csv');
figureFile = fullfile(resultsDirectory, ...
    'Measured_Output_and_Control.png');
comparisonFigureFile = fullfile(resultsDirectory, ...
    'Board_and_Simulink_Comparison.png');
matFile = fullfile(resultsDirectory, 'Iteration_Data.mat');
reportFile = fullfile(resultsDirectory, 'Iteration_Report.txt');
writetable(resultTable, csvFile);
writetable(pwmSequenceTable, sequenceFile);

figureHandle = figure('Color', 'white', 'Visible', 'off', ...
    'Position', [100 100 1400 1000]);
layout = tiledlayout(4, 1, 'TileSpacing', 'compact', ...
    'Padding', 'compact');

nexttile(layout);
plot(timeSeconds, measuredLineVoltages, 'LineWidth', 0.9);
grid on;
ylabel('Voltage (V)');
title('Measured line voltages');
legend('V_{34}', 'V_{45}', 'V_{53}', 'Location', 'best');

nexttile(layout);
plot(timeSeconds, phaseCurrents, 'LineWidth', 0.9);
grid on;
ylabel('Current (A)');
title('Measured phase currents');
legend('i_a', 'i_b', 'i_c', 'Location', 'best');

nexttile(layout);
plot(timeSeconds, boardIdAmperes, 'b-', 'LineWidth', 1.0);
hold on;
plot(timeSeconds, boardIqAmperes, 'r-', 'LineWidth', 1.0);
grid on;
ylabel('Current (A)');
title('F28379D dq currents');
legend('i_d', 'i_q', 'Location', 'best');

nexttile(layout);
plot(timeSeconds, boardPerUnit(:, 4:6), 'LineWidth', 0.9);
grid on;
xlabel('Virtual controller time (s)');
ylabel('Duty ratio');
title('F28379D SVPWM commands');
legend('A', 'B', 'C', 'Location', 'best');

exportgraphics(figureHandle, figureFile, 'Resolution', 300);
close(figureHandle);

comparisonFigure = figure('Color', 'white', 'Visible', 'off', ...
    'Position', [100 100 1400 850]);
comparisonLayout = tiledlayout(3, 1, 'TileSpacing', 'compact', ...
    'Padding', 'compact');
nexttile(comparisonLayout);
plot(timeSeconds, referenceAngleDegrees, 'k-', ...
    timeSeconds, boardAngleDegrees, 'r--', 'LineWidth', 1.0);
grid on;
ylabel('Angle (deg)');
title('PLL angle');
legend('Simulink', 'F28379D', 'Location', 'best');
nexttile(comparisonLayout);
plot(timeSeconds, referencePerUnit(:, 2:3), 'LineWidth', 1.0);
hold on;
plot(timeSeconds, boardPerUnit(:, 2:3), '--', 'LineWidth', 0.9);
grid on;
ylabel('Current (p.u.)');
title('dq current calculation');
legend('i_d Simulink', 'i_q Simulink', 'i_d F28379D', ...
    'i_q F28379D', 'Location', 'best');
nexttile(comparisonLayout);
plot(timeSeconds, errorCounts, 'LineWidth', 0.8);
grid on;
xlabel('Virtual controller time (s)');
ylabel('Q13 counts');
title('F28379D minus Simulink');
legend('Angle', 'i_d', 'i_q', 'Duty A', 'Duty B', 'Duty C', ...
    'Location', 'best');
exportgraphics(comparisonFigure, comparisonFigureFile, ...
    'Resolution', 300);
close(comparisonFigure);

report.csvFile = string(csvFile);
report.sequenceFile = string(sequenceFile);
report.figureFile = string(figureFile);
report.comparisonFigureFile = string(comparisonFigureFile);
report.matFile = string(matFile);
report.reportFile = string(reportFile);
save(matFile, 'report', 'resultTable', 'boardEncoded', ...
    'referenceEncoded', 'pwmSequenceTable');
writeTextReport(reportFile, report);
end

function reference = simulateReference(inputPerUnit, modelFile, sampleTime)
[~, sourceModel] = fileparts(modelFile);
sourceModel = char(sourceModel);
sourceWasLoaded = bdIsLoaded(sourceModel);
if sourceWasLoaded
    loadedFile = string(get_param(sourceModel, 'FileName'));
    assert(strcmpi(loadedFile, modelFile), ...
        'A different model named %s is already loaded: %s', ...
        sourceModel, loadedFile);
else
    load_system(modelFile);
end

referenceModel = 'ControllerVerificationReference';
if bdIsLoaded(referenceModel)
    close_system(referenceModel, 0);
end
new_system(referenceModel);
cleanupObj = onCleanup(@() closeReferenceModels(referenceModel, ...
    sourceModel, sourceWasLoaded));

sourceWorkspace = get_param(sourceModel, 'ModelWorkspace');
referenceWorkspace = get_param(referenceModel, 'ModelWorkspace');
variables = sourceWorkspace.whos;
for k = 1:numel(variables)
    referenceWorkspace.assignin(variables(k).name, ...
        sourceWorkspace.getVariable(variables(k).name));
end

sampleCount = size(inputPerUnit, 1);
timeSeconds = (0:sampleCount - 1).' * sampleTime;
referenceWorkspace.assignin('vabcInput', ...
    [timeSeconds, inputPerUnit(:, 1:3)]);
referenceWorkspace.assignin('iabcInput', ...
    [timeSeconds, inputPerUnit(:, 4:6)]);

add_block('simulink/Sources/From Workspace', ...
    [referenceModel '/Vabc'], 'VariableName', 'vabcInput', ...
    'Interpolate', 'off', 'OutputAfterFinalValue', ...
    'Holding final value', 'Position', [40 90 150 120]);
add_block('simulink/Sources/From Workspace', ...
    [referenceModel '/Iabc'], 'VariableName', 'iabcInput', ...
    'Interpolate', 'off', 'OutputAfterFinalValue', ...
    'Holding final value', 'Position', [40 170 150 200]);
add_block('simulink/Sources/Constant', ...
    [referenceModel '/Enable'], 'Value', '1', ...
    'OutDataTypeStr', 'boolean', 'Position', [70 245 120 275]);
add_block([sourceModel '/ControllerCore'], ...
    [referenceModel '/ControllerCore'], ...
    'Position', [260 70 500 310]);

for k = 1:3
    add_block('simulink/Sinks/Terminator', ...
        [referenceModel '/Terminator' num2str(k)], ...
        'Position', [570 140 + 60*k 590 160 + 60*k]);
end

controllerPorts = get_param([referenceModel '/ControllerCore'], ...
    'PortHandles');
voltagePorts = get_param([referenceModel '/Vabc'], 'PortHandles');
currentPorts = get_param([referenceModel '/Iabc'], 'PortHandles');
enablePorts = get_param([referenceModel '/Enable'], 'PortHandles');
add_line(referenceModel, voltagePorts.Outport, ...
    controllerPorts.Inport(1));
add_line(referenceModel, currentPorts.Outport, ...
    controllerPorts.Inport(2));
add_line(referenceModel, enablePorts.Outport, controllerPorts.Enable);
for k = 1:3
    terminatorPorts = get_param([referenceModel '/Terminator' ...
        num2str(k)], 'PortHandles');
    add_line(referenceModel, controllerPorts.Outport(k), ...
        terminatorPorts.Inport);
end

core = [referenceModel '/ControllerCore'];
children = find_system(core, 'SearchDepth', 1, 'Type', 'Block');
pll = children{find(contains(children, ...
    'Sinusoidal Measurement'), 1)};
pllPorts = get_param(pll, 'PortHandles');
demuxPorts = get_param([core '/Demux'], 'PortHandles');
dutyAPorts = get_param([core '/Saturation'], 'PortHandles');
dutyBPorts = get_param([core '/Saturation1'], 'PortHandles');
dutyCPorts = get_param([core '/Saturation2'], 'PortHandles');
loggedPorts = [pllPorts.Outport(2), demuxPorts.Outport(1), ...
    demuxPorts.Outport(2), dutyAPorts.Outport, dutyBPorts.Outport, ...
    dutyCPorts.Outport];
loggedNames = {'theta', 'id', 'iq', 'dutyA', 'dutyB', 'dutyC'};
for k = 1:numel(loggedPorts)
    set_param(loggedPorts(k), 'DataLogging', 'on', ...
        'DataLoggingNameMode', 'Custom', ...
        'DataLoggingName', loggedNames{k});
end

set_param(referenceModel, 'SolverType', 'Fixed-step', ...
    'Solver', 'FixedStepDiscrete', 'FixedStep', ...
    sprintf('%.15g', sampleTime), 'StartTime', '0', ...
    'StopTime', sprintf('%.15g', timeSeconds(end)), ...
    'SignalLogging', 'on', 'SignalLoggingName', 'logsout', ...
    'SaveOutput', 'off');
simulationOutput = sim(referenceModel);
logs = simulationOutput.logsout;

reference = zeros(sampleCount, 6);
for k = 1:6
    element = logs.get(loggedNames{k});
    values = squeeze(element.Values.Data);
    reference(:, k) = values(:);
end
reference(:, 1) = mod(reference(:, 1), 2*pi) / (2*pi);

clear cleanupObj;
closeReferenceModels(referenceModel, sourceModel, sourceWasLoaded);
end

function encoded = encodeQ13(values)
scaled = round(values * 8192);
scaled = min(max(scaled, -32768), 32767);
encoded = int16(scaled);
end

function [timeSeconds, channels] = readLabVoltData(filePath)
[~, ~, extension] = fileparts(filePath);
if strcmpi(extension, '.csv')
    data = readtable(filePath, 'VariableNamingRule', 'preserve');
    names = string(data.Properties.VariableNames);
    channelNames = ["E1_V", "E2_V", "E3_V", ...
        "I1_A", "I2_A", "I3_A"];
    timeName = "";
    if any(names == "Time_s")
        timeName = "Time_s";
    elseif any(names == "MeasuredTime_s")
        timeName = "MeasuredTime_s";
    end
    assert(strlength(timeName) > 0 && all(ismember(channelNames, names)), ...
        ['Measured CSV must contain Time_s or MeasuredTime_s, plus ', ...
         'E1_V, E2_V, E3_V, I1_A, I2_A and I3_A.']);
    timeSeconds = double(data{:, char(timeName)});
    channels = double(data{:, cellstr(channelNames)});
    assert(all(isfinite(timeSeconds)) && all(isfinite(channels), 'all'), ...
        'Measured CSV contains non-finite values.');
    assert(numel(timeSeconds) >= 2, ...
        'Measured CSV contains fewer than two samples.');
    assert(all(diff(timeSeconds) > 0), ...
        'Measured CSV time values must increase strictly.');
    return;
end

lines = readlines(filePath);
raw = nan(numel(lines), 7);
rowCount = 0;
for lineNumber = 1:numel(lines)
    values = sscanf(char(lines(lineNumber)), '%f').';
    if numel(values) >= 7
        rowCount = rowCount + 1;
        raw(rowCount, :) = values(1:7);
    end
end
raw = raw(1:rowCount, :);
assert(size(raw, 1) >= 2, ...
    'No complete LabVolt samples were found.');
timeSeconds = raw(:, 1) * 1e-3;
channels = raw(:, 2:7);
assert(all(diff(timeSeconds) > 0), ...
    'LabVolt time values must increase strictly.');
end

function sampleTimes = makeControllerTimes(timeSeconds, controllerRateHz)
samplePeriod = 1 / controllerRateHz;
sampleTimes = (timeSeconds(1):samplePeriod:timeSeconds(end)).';
assert(numel(sampleTimes) >= 2, ...
    'The LabVolt recording is shorter than two controller periods.');
end

function phaseVoltages = lineToPhaseVoltages(lineVoltages)
assert(size(lineVoltages, 2) == 3, ...
    'Three measured line-voltage channels are required.');
phaseVoltages = [(lineVoltages(:, 1) - lineVoltages(:, 3)) / 3, ...
    (lineVoltages(:, 2) - lineVoltages(:, 1)) / 3, ...
    (lineVoltages(:, 3) - lineVoltages(:, 2)) / 3];
end

function [timeSeconds, measuredVoltages, phaseCurrents, cycleSamples] = ...
        repeatLastMeasuredCycle(measuredVoltages, phaseCurrents, ...
        controllerRateHz, expectedFrequencyHz, targetSeconds)
cycleSamples = round(controllerRateHz / expectedFrequencyHz);
assert(size(measuredVoltages, 1) >= cycleSamples, ...
    ['The initial measured file does not contain one complete ', ...
     'cycle at the expected frequency.']);
measuredCycle = measuredVoltages(end - cycleSamples + 1:end, :);
currentCycle = phaseCurrents(end - cycleSamples + 1:end, :);
targetSamples = max(2 * cycleSamples, round(targetSeconds * ...
    controllerRateHz));
repeatCount = ceil(targetSamples / cycleSamples);
measuredVoltages = repmat(measuredCycle, repeatCount, 1);
phaseCurrents = repmat(currentCycle, repeatCount, 1);
measuredVoltages = measuredVoltages(1:targetSamples, :);
phaseCurrents = phaseCurrents(1:targetSamples, :);
timeSeconds = (0:targetSamples - 1).' / controllerRateHz;
end

function [alignedInput, bestShift] = alignPeriodicInput( ...
        currentInput, previousInput, cycleSamples)
assert(size(currentInput, 1) >= cycleSamples && ...
    size(previousInput, 1) >= cycleSamples, ...
    'Phase alignment requires one complete input cycle.');

currentCycle = currentInput(1:cycleSamples, :);
previousCycle = previousInput(end - cycleSamples + 1:end, :);
previousVoltage = normalizeAlignmentSignal(previousCycle(:, 1:3));
bestScore = inf;
bestShift = 0;
for candidateShift = 0:cycleSamples - 1
    candidateCycle = circshift(currentCycle, -candidateShift, 1);
    candidateVoltage = normalizeAlignmentSignal( ...
        candidateCycle(:, 1:3));
    score = mean((candidateVoltage - previousVoltage).^2, 'all');
    if score < bestScore
        bestScore = score;
        bestShift = candidateShift;
    end
end

alignedCycle = circshift(currentCycle, -bestShift, 1);
repeatCount = ceil(size(currentInput, 1) / cycleSamples);
alignedInput = repmat(alignedCycle, repeatCount, 1);
alignedInput = alignedInput(1:size(currentInput, 1), :);
end

function normalized = normalizeAlignmentSignal(signal)
centered = signal - mean(signal, 1);
scale = sqrt(mean(centered.^2, 1));
scale(scale < 1e-12) = 1;
normalized = centered ./ scale;
end

function bytes = int16ToLittleEndian(values)
words = typecast(values(:), 'uint16');
bytes = reshape([uint8(bitand(words, 255)).'; ...
    uint8(bitshift(words, -8)).'], 1, []);
end

function values = littleEndianToInt16(bytes)
words = uint16(bytes(1:2:end)) + ...
    bitshift(uint16(bytes(2:2:end)), 8);
values = typecast(words(:), 'int16').';
end

function sequence = nextSequence(sequence)
if sequence >= int16(32767)
    sequence = int16(1);
else
    sequence = sequence + int16(1);
end
end

function waitUntil(timer, targetSeconds)
remaining = targetSeconds - toc(timer);
while remaining > 0
    pause(min(remaining, 0.001));
    remaining = targetSeconds - toc(timer);
end
end

function [matched, values, buffer] = waitForResult( ...
        device, buffer, expectedSequence, timeoutSeconds)
startTime = tic;
matched = false;
values = zeros(1, 6, 'int16');
while toc(startTime) < timeoutSeconds
    [sequences, results, buffer] = readAvailableResults(device, buffer);
    matchIndex = find(sequences == expectedSequence, 1, 'last');
    if ~isempty(matchIndex)
        matched = true;
        values = results(matchIndex, :);
        return;
    end
    pause(0.0005);
end
end

function [sequences, results, buffer] = readAvailableResults(device, buffer)
if device.NumBytesAvailable > 0
    incoming = read(device, device.NumBytesAvailable, 'uint8');
    buffer = [buffer, incoming(:).'];
end

sequences = int16([]);
results = zeros(0, 6, 'int16');
while true
    startIndex = find(buffer == uint8('R'), 1);
    if isempty(startIndex)
        buffer = uint8([]);
        return;
    end
    if startIndex > 1
        buffer = buffer(startIndex:end);
    end
    if numel(buffer) < 16
        return;
    end
    if buffer(16) ~= uint8('T')
        buffer = buffer(2:end);
        continue;
    end

    words = littleEndianToInt16(buffer(2:15));
    sequences(end + 1, 1) = words(1); %#ok<AGROW>
    results(end + 1, :) = words(2:7); %#ok<AGROW>
    buffer = buffer(17:end);
end
end

function writeTextReport(filePath, report)
fileId = fopen(filePath, 'w');
assert(fileId >= 0, 'Unable to create report: %s', filePath);
cleanupObj = onCleanup(@() fclose(fileId));
fprintf(fileId, 'F28379D slow hardware-iteration report\n\n');
fprintf(fileId, 'Algorithm implementation: %s\n', report.result);
fprintf(fileId, 'Measured hardware-control response: %s\n', ...
    report.hardwareAssessment);
fprintf(fileId, 'Overall assessment: %s\n', report.overallAssessment);
fprintf(fileId, 'Input source: %s\n', report.inputOrigin);
fprintf(fileId, 'Voltage input mapping: %s\n', ...
    report.voltageInputMapping);
fprintf(fileId, 'Measured data file: %s\n', report.dataFile);
fprintf(fileId, 'Repeated source-cycle samples: %d\n', ...
    report.repeatedCycleSamples);
fprintf(fileId, 'Requested repeated duration: %.6f s\n', ...
    report.repeatedDurationSeconds);
fprintf(fileId, 'Phase-alignment shift: %d controller samples\n', ...
    report.phaseAlignmentShiftSamples);
fprintf(fileId, 'Returned frames: %d\n', report.framesReturned);
fprintf(fileId, 'Controller rate: %.0f Hz\n', report.controllerRateHz);
fprintf(fileId, 'Transfer rate: %.0f frames/s\n', report.transferRateHz);
fprintf(fileId, 'Mean round trip: %.6f ms\n', ...
    report.meanRoundTripMilliseconds);
fprintf(fileId, 'Maximum round trip: %.6f ms\n', ...
    report.maximumRoundTripMilliseconds);
fprintf(fileId, ['Allowed numerical difference ', ...
    '[angle id iq dutyA dutyB dutyC]: %s Q13 counts\n\n'], ...
    strjoin(string(report.toleranceCounts), ' '));
fprintf(fileId, 'Controller-cycle frequency estimate: %.6f Hz\n', ...
    report.measuredFrequencyHz);
fprintf(fileId, 'Measured line-voltage RMS [V34 V45 V53]: %.6f %.6f %.6f V\n', ...
    report.lineVoltageRms);
fprintf(fileId, ['Reconstructed controller phase-voltage RMS ', ...
    '[Va Vb Vc]: %.6f %.6f %.6f V\n'], report.phaseVoltageRms);
fprintf(fileId, 'Phase-current RMS [Ia Ib Ic]: %.6f %.6f %.6f A\n', ...
    report.phaseCurrentRms);
fprintf(fileId, 'Phase-current peak [Ia Ib Ic]: %.6f %.6f %.6f A\n', ...
    report.phaseCurrentPeak);
fprintf(fileId, 'Replay-cycle voltage THD h2-h50 [V34 V45 V53]: %.6f %.6f %.6f %%\n', ...
    report.lineVoltageThdPercent);
fprintf(fileId, 'Replay-cycle current THD h2-h50 [Ia Ib Ic]: %.6f %.6f %.6f %%\n', ...
    report.phaseCurrentThdPercent);
fprintf(fileId, 'Mean i_d: %.6f p.u. = %.6f A\n', ...
    report.idMeanPerUnit, report.idMeanAmperes);
fprintf(fileId, 'Mean i_q: %.6f p.u. = %.6f A\n', ...
    report.iqMeanPerUnit, report.iqMeanAmperes);
fprintf(fileId, 'i_d/i_q standard deviation: %.6f / %.6f p.u.\n', ...
    report.idRipplePerUnit, report.iqRipplePerUnit);
fprintf(fileId, 'Duty minima [A B C]: %.6f %.6f %.6f\n', ...
    report.dutyMinimum);
fprintf(fileId, 'Duty maxima [A B C]: %.6f %.6f %.6f\n', ...
    report.dutyMaximum);
fprintf(fileId, 'PWM replay cycle: %d samples, %.6f to %.6f s\n\n', ...
    report.pwmSequenceSamples, report.cycleStartSeconds, ...
    report.cycleEndSeconds);
labels = {'PLL angle', 'i_d', 'i_q', 'Duty A', 'Duty B', 'Duty C'};
for k = 1:6
    fprintf(fileId, '%s: max %.0f count(s), RMS %.6f count(s)\n', ...
        labels{k}, report.maximumAbsoluteErrorCounts(k), ...
        report.rootMeanSquareErrorCounts(k));
end
clear cleanupObj;
end

function thdPercent = calculateCycleThd(signals, maximumHarmonic)
sampleCount = size(signals, 1);
spectrum = abs(fft(signals - mean(signals, 1), [], 1)) / sampleCount;
maximumBin = min(maximumHarmonic + 1, floor(sampleCount / 2) + 1);
fundamental = spectrum(2, :);
harmonics = spectrum(3:maximumBin, :);
thdPercent = 100 * sqrt(sum(harmonics.^2, 1)) ./ fundamental;
thdPercent(fundamental <= eps) = NaN;
end

function directory = nextIterationDirectory(baseDirectory)
if ~isfolder(baseDirectory)
    mkdir(baseDirectory);
end
iteration = 1;
while true
    candidate = fullfile(baseDirectory, ...
        sprintf('Iteration_%03d', iteration));
    if ~isfolder(candidate)
        mkdir(candidate);
        directory = string(candidate);
        return;
    end
    iteration = iteration + 1;
end
end

function closeReferenceModels(referenceModel, sourceModel, sourceWasLoaded)
if bdIsLoaded(referenceModel)
    close_system(referenceModel, 0);
end
if ~sourceWasLoaded && bdIsLoaded(sourceModel)
    close_system(sourceModel, 0);
end
end

function closeSerial(device)
if ~isempty(device) && isvalid(device)
    flush(device);
    delete(device);
end
end
