function report = run_slow_iteration(portName, measuredFile, options)
%RUN_SLOW_ITERATION Calculate and apply one batch-control update.

arguments
    portName (1,1) string
    measuredFile (1,1) string
    options.PackageDirectory (1,1) string = ""
end

scriptDirectory = fileparts(mfilename('fullpath'));
if strlength(options.PackageDirectory) == 0
    options.PackageDirectory = string(fileparts(scriptDirectory));
end

modelFile = fullfile(options.PackageDirectory, 'Model', ...
    'IterativeController.slx');
resultsDirectory = fullfile(options.PackageDirectory, 'Results');
stateDirectory = fullfile(options.PackageDirectory, 'State');
historyFile = fullfile(stateDirectory, 'Controller_Input_History.mat');
activeSequenceFile = fullfile(stateDirectory, 'Active_PWM_Sequence.mat');
iterationHistoryFile = fullfile(stateDirectory, 'Iteration_History.csv');

assert(isfile(measuredFile), 'Measured output file not found: %s', ...
    measuredFile);
assert(isfile(modelFile), 'Controller model not found: %s', modelFile);
assert(isfile(activeSequenceFile), ...
    'Run Load_Initial_PWM.m before the first hardware iteration.');

report = send_data(portName, ...
    FilePath=measuredFile, ...
    ModelFile=modelFile, ...
    ResultsDirectory=resultsDirectory, ...
    VoltageRangePeak=80, ...
    CurrentRangePeak=0.04, ...
    ControllerRateHz=20000, ...
    TransferRateHz=10, ...
    ExpectedFrequencyHz=50, ...
    IdReferencePerUnit=0.253, ...
    ReferenceHistoryFile=historyFile, ...
    MinimumSourceDurationSeconds=0.18, ...
    RepeatLastCycleToSeconds=0.20, ...
    CreateIterationDirectory=true);

assert(report.algorithmPassed, ...
    ['F28379D and Simulink do not agree. The previous PWM remains ', ...
     'active and no new sequence was loaded.']);
assert(report.frequencyPassed, ...
    ['The PLL result is outside 50 +/- 2 Hz. The previous PWM remains ', ...
     'active and no new sequence was loaded.']);
assert(all(report.phaseCurrentPeak <= 0.05), ...
    ['At least one measured phase-current peak exceeds 0.05 A. ', ...
     'The previous PWM remains active.']);

stored = load(activeSequenceFile, 'activeDuty');
assert(isfield(stored, 'activeDuty') && ...
    isequal(size(stored.activeDuty), [400, 3]), ...
    'The saved active PWM sequence is invalid.');

candidateDuty = double(report.dutySequenceEncoded) / 8192;
candidateDuty = resamplePeriodicCycle(candidateDuty, 400);

maximumDutyChange = 0.005;
dutyChange = candidateDuty - stored.activeDuty;
dutyChange = min(max(dutyChange, -maximumDutyChange), ...
    maximumDutyChange);
activeDuty = stored.activeDuty + dutyChange;
activeDuty = min(max(activeDuty, 0.05), 0.95);
activeDutyEncoded = int16(round(activeDuty * 8192));

uploadStatus = upload_pwm_sequence(portName, activeDutyEncoded, ...
    BaudRate=115200);
assert(uploadStatus.loaded, 'The new PWM sequence was not activated.');

if ~isfolder(stateDirectory)
    mkdir(stateDirectory);
end
save(activeSequenceFile, 'activeDuty', 'activeDutyEncoded', ...
    'uploadStatus');

sequenceTime = (0:399).' * 0.00005;
sequenceTable = table(sequenceTime, activeDuty(:, 1), ...
    activeDuty(:, 2), activeDuty(:, 3), ...
    double(activeDutyEncoded(:, 1)), ...
    double(activeDutyEncoded(:, 2)), ...
    double(activeDutyEncoded(:, 3)), ...
    'VariableNames', {'Time_s', 'DutyA', 'DutyB', 'DutyC', ...
    'DutyA_Q13', 'DutyB_Q13', 'DutyC_Q13'});
writetable(sequenceTable, fullfile(stateDirectory, ...
    'Active_PWM_Sequence.csv'));
writetable(sequenceTable, fullfile(report.resultsDirectory, ...
    'Applied_PWM_Sequence.csv'));
[~, ~, measuredExtension] = fileparts(measuredFile);
measuredCopyFile = fullfile(char(report.resultsDirectory), ...
    ['LVDAC_Measured_Output', char(measuredExtension)]);
copyfile(char(measuredFile), char(measuredCopyFile));

report.maximumAppliedDutyChange = max(abs(dutyChange), [], 'all');
report.appliedDutyMinimum = min(activeDuty, [], 1);
report.appliedDutyMaximum = max(activeDuty, [], 1);
report.upload = uploadStatus;
save(report.matFile, 'report', '-append');
appendAppliedSequence(report.reportFile, report);
appendHistory(iterationHistoryFile, report);

fprintf('\nSlow hardware iteration complete.\n');
fprintf('Maximum applied duty change: %.6f\n', ...
    report.maximumAppliedDutyChange);
fprintf('The new 400-sample, 50 Hz PWM cycle is active.\n');
fprintf('Results: %s\n', report.resultsDirectory);
end

function output = resamplePeriodicCycle(input, outputSamples)
inputSamples = size(input, 1);
assert(inputSamples >= 2, 'The controller did not return a PWM cycle.');
sourcePosition = (0:inputSamples).' / inputSamples;
targetPosition = (0:outputSamples - 1).' / outputSamples;
periodicInput = [input; input(1, :)];
output = interp1(sourcePosition, periodicInput, targetPosition, 'linear');
end

function appendAppliedSequence(filePath, report)
fileId = fopen(filePath, 'a');
assert(fileId >= 0, 'Unable to update report: %s', filePath);
cleanupObj = onCleanup(@() fclose(fileId));
fprintf(fileId, '\nApplied batch update\n');
fprintf(fileId, 'PWM samples: 400\n');
fprintf(fileId, 'PWM replay rate: 20000 Hz\n');
fprintf(fileId, 'PWM fundamental frequency: 50 Hz\n');
fprintf(fileId, 'Maximum duty change this iteration: %.6f\n', ...
    report.maximumAppliedDutyChange);
fprintf(fileId, 'Applied duty minima [A B C]: %.6f %.6f %.6f\n', ...
    report.appliedDutyMinimum);
fprintf(fileId, 'Applied duty maxima [A B C]: %.6f %.6f %.6f\n', ...
    report.appliedDutyMaximum);
clear cleanupObj;
end

function appendHistory(filePath, report)
row = table(string(datetime('now', 'Format', 'yyyy-MM-dd HH:mm:ss')), ...
    report.measuredFrequencyHz, report.idMeanAmperes, ...
    report.iqMeanAmperes, report.lineVoltageRms(1), ...
    report.lineVoltageRms(2), report.lineVoltageRms(3), ...
    report.phaseCurrentRms(1), report.phaseCurrentRms(2), ...
    report.phaseCurrentRms(3), report.maximumAppliedDutyChange, ...
    'VariableNames', {'Timestamp', 'PLL_Hz', 'Id_A', 'Iq_A', ...
    'V34_RMS_V', 'V45_RMS_V', 'V53_RMS_V', ...
    'Ia_RMS_A', 'Ib_RMS_A', 'Ic_RMS_A', 'MaximumDutyChange'});
if isfile(filePath)
    history = readtable(filePath, 'TextType', 'string');
    oldVoltageNames = {'Vab_RMS_V', 'Vbc_RMS_V', 'Vca_RMS_V'};
    newVoltageNames = {'V34_RMS_V', 'V45_RMS_V', 'V53_RMS_V'};
    if all(ismember(oldVoltageNames, history.Properties.VariableNames))
        history = renamevars(history, oldVoltageNames, newVoltageNames);
    end
    history = [history; row];
else
    history = row;
end
writetable(history, filePath);
end

