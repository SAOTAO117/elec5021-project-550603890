function status = upload_pwm_sequence(portName, dutySequenceEncoded, options)
%UPLOAD_PWM_SEQUENCE Load one complete duty cycle for 20 kHz board replay.

arguments
    portName (1,1) string
    dutySequenceEncoded (:,3) int16
    options.BaudRate (1,1) double ...
        {mustBeInteger, mustBePositive} = 115200
    options.TimeoutSeconds (1,1) double ...
        {mustBeFinite, mustBePositive} = 2
end

sampleCount = size(dutySequenceEncoded, 1);
assert(sampleCount >= 2 && sampleCount <= 2048, ...
    'PWM sequence length must be between 2 and 2048 samples.');
duty = double(dutySequenceEncoded) / 8192;
assert(all(duty >= 0.05 & duty <= 0.95, 'all'), ...
    'At least one PWM duty command is outside 0.05 to 0.95.');

availablePorts = string(serialportlist('available'));
assert(any(strcmpi(portName, availablePorts)), ...
    'Serial port %s is not available. Available ports: %s', ...
    portName, strjoin(availablePorts, ', '));

device = serialport(portName, options.BaudRate, ...
    'Timeout', options.TimeoutSeconds);
cleanupObj = onCleanup(@() closeSerial(device));
flush(device);

oldStatus = queryStatus(device, options.TimeoutSeconds);

beginFrame = [uint8('B'), wordBytes(uint16(sampleCount)), uint8('C')];
write(device, beginFrame, 'uint8');
beginStatus = waitForStatus(device, 'B', options.TimeoutSeconds);
assert(beginStatus.code == 0, ...
    'The board rejected the PWM sequence length (status %d).', ...
    beginStatus.code);

for sample = 1:sampleCount
    dutyWords = typecast(dutySequenceEncoded(sample, :), 'uint16');
    payload = [uint16(sample - 1), dutyWords];
    frame = [uint8('P'), wordBytes(payload), uint8('Q')];
    write(device, frame, 'uint8');
    if mod(sample, 32) == 0
        pause(0.005);
    end
end

write(device, uint8(['G', 'H']), 'uint8');
commitStatus = waitForStatus(device, 'G', options.TimeoutSeconds);
assert(commitStatus.code == 0, ...
    'The board rejected the PWM sequence (status %d).', ...
    commitStatus.code);

timer = tic;
status = commitStatus;
while toc(timer) < options.TimeoutSeconds
    status = queryStatus(device, options.TimeoutSeconds);
    generationChanged = status.generation ~= oldStatus.generation;
    if status.active && status.length == sampleCount && generationChanged
        break;
    end
    pause(0.02);
end

assert(status.active, 'The board did not activate PWM replay.');
assert(status.length == sampleCount, ...
    'The active PWM sequence has %d samples instead of %d.', ...
    status.length, sampleCount);
assert(status.generation ~= oldStatus.generation, ...
    'The board did not switch to the newly loaded PWM sequence.');

status.samples = sampleCount;
status.minimumDuty = min(duty, [], 1);
status.maximumDuty = max(duty, [], 1);
status.loaded = true;

fprintf('PWM sequence loaded: %d samples at 20 kHz.\n', sampleCount);
fprintf('Replay generation: %d; active: %d.\n', ...
    status.generation, status.active);

clear cleanupObj;
closeSerial(device);
end

function status = queryStatus(device, timeoutSeconds)
write(device, uint8(['J', 'K']), 'uint8');
status = waitForStatus(device, 'J', timeoutSeconds);
end

function status = waitForStatus(device, expectedCommand, timeoutSeconds)
buffer = uint8([]);
timer = tic;
while toc(timer) < timeoutSeconds
    if device.NumBytesAvailable > 0
        incoming = read(device, device.NumBytesAvailable, 'uint8');
        buffer = [buffer, incoming(:).']; %#ok<AGROW>
    end

    while true
        startIndex = find(buffer == uint8('A'), 1);
        if isempty(startIndex)
            buffer = uint8([]);
            break;
        end
        if startIndex > 1
            buffer = buffer(startIndex:end);
        end
        if numel(buffer) < 11
            break;
        end
        if buffer(11) ~= uint8('Z')
            buffer = buffer(2:end);
            continue;
        end

        command = char(buffer(2));
        status = struct( ...
            'command', string(command), ...
            'code', double(buffer(3)), ...
            'active', logical(buffer(4)), ...
            'generation', double(readWord(buffer(5:6))), ...
            'length', double(readWord(buffer(7:8))), ...
            'index', double(readWord(buffer(9:10))));
        buffer = buffer(12:end);
        if command == expectedCommand
            return;
        end
    end
    pause(0.001);
end
error('No %s status frame was returned by the board.', expectedCommand);
end

function bytes = wordBytes(words)
words = uint16(words(:));
bytes = reshape([uint8(bitand(words, 255)).'; ...
    uint8(bitshift(words, -8)).'], 1, []);
end

function value = readWord(bytes)
value = uint16(bytes(1)) + bitshift(uint16(bytes(2)), 8);
end

function closeSerial(device)
if ~isempty(device) && isvalid(device)
    flush(device);
    delete(device);
end
end
