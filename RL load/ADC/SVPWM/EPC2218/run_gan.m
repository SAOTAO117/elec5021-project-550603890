function run_gan
%RUN_GAN Simulate the EPC2218 plant and export one 20 ms replay record.
folder = fileparts(mfilename('fullpath'));
[~, methodName] = fileparts(fileparts(folder));
method = string(methodName);
addpath(folder);
model = "inverterRL60V" + method + "ADCEPC2218";
assert(~bdIsLoaded(model), 'Close the existing model named %s first.', model);
load_system(fullfile(folder, model + '.slx'));
cleanup = onCleanup(@() close_system(model, 0));
folder = fullfile(folder, 'output');
if ~isfolder(folder), mkdir(folder); end
Simulink.fileGenControl('set', 'CacheFolder', fullfile(folder, 'cache'), ...
    'CodeGenFolder', fullfile(folder, 'codegen'), 'createDir', true);
input = Simulink.SimulationInput(model);
input = input.setModelParameter('StartTime', '0', 'StopTime', '0.5', ...
    'SimulationMode', 'accelerator', 'SimscapeLogType', 'none', ...
    'SaveFinalState', 'on', 'SaveOperatingPoint', 'on', ...
    'FinalStateName', 'finalOp', 'ReturnWorkspaceOutputs', 'on');
warmup = sim(input);
assert(warmup.tout(end) >= 0.5 - 1e-10, 'Warmup did not finish.');
save(fullfile(folder, method + '_warmup.mat'), 'warmup', '-v7.3');
input = input.setInitialState(warmup.finalOp.loggedStates);
input = input.setModelParameter('StartTime', '0.5', 'StopTime', '0.52', ...
    'SimscapeLogType', 'all', 'SimscapeLogLimitData', 'off');
period = sim(input);
assert(period.tout(end) >= 0.52 - 1e-10, 'Period capture did not finish.');
save(fullfile(folder, method + '_period.mat'), 'period', '-v7.3');
logs = period.simlog;
t = logs.Inverter.G.series.time;
gates = logs.Inverter.G.series.values('1');
keep = [true; any(diff(gates, 1, 1) ~= 0, 2)];
keep(end) = true;
writematrix([t(keep)-t(1), gates(keep, :)], fullfile(folder, method+'_gates.csv'));
t = logs.x2_mH.IL.series.time;
current = logs.x2_mH.IL.series.values('A');
writematrix([t-t(1), current], fullfile(folder, method+'_bridge_current.csv'));
average = @(node) trapz(node.series.time, node.series.values('W'))/(t(end)-t(1));
cap = 'c_80uF';
if method == "SVPWM", cap = 'c_700uF'; end
metrics = struct('load_W', average(logs.RL_Load.wye_impedance.power_dissipated), ...
    'filter_W', average(logs.x2_mH.power_dissipated) + ...
    average(logs.filter_2mH.power_dissipated) + ...
    average(logs.(cap).power_dissipated) + average(logs.Capacitor.power_dissipated));
metrics.bridgeRms = sqrt(trapz(t, current.^2)/(t(end)-t(1)));
fid = fopen(fullfile(folder, method+'_cycle_load.json'), 'w');
assert(fid >= 0, 'Cannot write the cycle metrics.');
fileCleanup = onCleanup(@() fclose(fid));
fwrite(fid, jsonencode(metrics, PrettyPrint=true), 'char');
fprintf('Replay inputs saved in %s.\n', folder);
end
