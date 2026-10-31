function generate_controller_code
%GENERATE_CONTROLLER_CODE Generate C code without programming a board.
root = fileparts(mfilename('fullpath'));
modelDirectory = fullfile(root, 'Model');
previousDirectory = pwd;
cd(modelDirectory);
restoreDirectory = onCleanup(@() cd(previousDirectory));
model = 'IterativeController';
assert(~bdIsLoaded(model), 'Close IterativeController before generating code.');
load_system(fullfile(modelDirectory, [model '.slx']));
closeModel = onCleanup(@() close_system(model, 0));
generated = fullfile(root, 'Generated');
Simulink.fileGenControl('set', 'CacheFolder', fullfile(generated, 'cache'), ...
    'CodeGenFolder', fullfile(generated, 'code'), 'createDir', true);
set_param(model, 'CustomInclude', modelDirectory, ...
    'CustomSource', fullfile(modelDirectory, 'iteration_serial.c'), ...
    'GenCodeOnly', 'on', 'GenerateReport', 'off');
slbuild(model);
fprintf('Generated controller code: %s\n', generated);
fprintf('The recorded firmware in Program was not replaced.\n');
end
