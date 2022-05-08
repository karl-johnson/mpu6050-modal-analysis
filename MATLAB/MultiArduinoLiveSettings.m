clear; close all; fclose all; delete(instrfind); clc; % Start Fresh (TM)
% USER SETTINGS
% The following line only works if
% - the only available COM ports are Arduinos
% - all Arduinos have the right firmware uploaded
% TODO auto detect
ARDUINO_SERIAL_PORTS = serialportlist;
REC_LENGTH = 8192; % Length of recording, samples (power of 2 recommended)
ACCEL_FREQ = 100; % Accel sample rate, Hz (100,200,250,333,or 500)
ACCEL_SCALE = 2; % Accel range, g (+/-) (2,4,8, or 16)
PLOT_HISTORY = 500;
UPDATE_RATE = 20; % Update plots this many times per second
% no touch
BAUD_RATE = 115200;
MAX_RAW_OUTPUT = 32768;
G_CONVERSION = ACCEL_SCALE/MAX_RAW_OUTPUT;
% Connect to Arduinos through Serial
NUM_ARDUINOS = length(ARDUINO_SERIAL_PORTS);
clear serialPortArray;
serialPortArray = [];
for portIndex = 1:NUM_ARDUINOS
    disp(['Connecting to ' char(ARDUINO_SERIAL_PORTS(portIndex)) '...']);
    thisSerialPort = serialport(ARDUINO_SERIAL_PORTS(portIndex),BAUD_RATE);
    configureTerminator(thisSerialPort,uint8(newline));
    configureCallback(thisSerialPort,"terminator",@showArduinoOutput);
    waitForResponse(thisSerialPort); % wait for Arduino to wake up
    % once it's responded, add this to array
    serialPortArray = [serialPortArray; thisSerialPort];
end
% Configure settings on all accelerometers
for portIndex = 1:NUM_ARDUINOS
    disp(['Writing settings to ' char(ARDUINO_SERIAL_PORTS(portIndex)) '...']);
    disp("Sending frequency...");
    writeLineAndWait(serialPortArray(portIndex), ['f' int2str(ACCEL_FREQ)]);
    disp("Sending scale...");
    writeLineAndWait(serialPortArray(portIndex), ['s' int2str(ACCEL_SCALE)]);
end
disp("Starting up live feed...");
xplots = cell(NUM_ARDUINOS,1);
yplots = cell(NUM_ARDUINOS,1);
zplots = cell(NUM_ARDUINOS,1);
tiledlayout(NUM_ARDUINOS,1);
for portIndex = 1:NUM_ARDUINOS
    nexttile
    hold on;
    xplots{portIndex} = plot(0,0);
    yplots{portIndex} = plot(0,0);
    zplots{portIndex} = plot(0,0);
    hold off;
    xlim([0 PLOT_HISTORY]);
    ylim([-ACCEL_SCALE ACCEL_SCALE]);
    title(char(ARDUINO_SERIAL_PORTS(portIndex)));
end

% start all arduinos in as quick of succession as possible in Matlab
numRecordedSamples = 0;
recordedData = cell(NUM_ARDUINOS,1);
for portIndex = 1:NUM_ARDUINOS
    flush(serialPortArray(portIndex));
    writeline(serialPortArray(portIndex), ['g' int2str(REC_LENGTH)]);
end
lastPrintTime = 0;
startTime = tic;
while(ishandle(xplots{1}))
    for portIndex = 1:NUM_ARDUINOS
        % blocking read
        dataIn = uint8(read(serialPortArray(portIndex),7,'uint8')); % includes terminator :/
        dataIn = dataIn(1:6); % remove terminator
        % TODO stop using terminator, it's useless
        [x, y, z] = getAccelVals(dataIn,G_CONVERSION);
        recordedData{portIndex}(numRecordedSamples+1,:) = [x, y, z];
    end
    numRecordedSamples = numRecordedSamples+1;
    if(toc(startTime) - lastPrintTime > (1/UPDATE_RATE))
        % remove old samples
        if(numRecordedSamples > PLOT_HISTORY)
            for portIndex = 1:NUM_ARDUINOS
                recordedData{portIndex} = recordedData{portIndex}(end-PLOT_HISTORY+1:end,:);
            end
            numRecordedSamples = PLOT_HISTORY;
        end
        % update plot
        for portIndex = 1:NUM_ARDUINOS
            set(xplots{portIndex}, 'XData', [1:numRecordedSamples], 'YData', recordedData{portIndex}(:,1));
            set(yplots{portIndex}, 'XData', [1:numRecordedSamples], 'YData', recordedData{portIndex}(:,2));
            set(zplots{portIndex}, 'XData', [1:numRecordedSamples], 'YData', recordedData{portIndex}(:,3));
        end
    end
end
clear serialPortArray;

%% Functions
function [x, y, z] = getAccelVals(lineIn,G_CONVERSION)
    % take 6-byte array and return accelerometer values scaled in g
    x = G_CONVERSION*double(swapbytes(typecast(lineIn(1:2), 'int16')));
    y = G_CONVERSION*double(swapbytes(typecast(lineIn(3:4), 'int16')));
    z = G_CONVERSION*double(swapbytes(typecast(lineIn(5:6), 'int16')));
end

function writeLineAndWait(device, data)
    configureCallback(device,"terminator",@showArduinoOutput);
    writeline(device, data);
    global isWaiting
    isWaiting = true;
    while(isWaiting)
        pause(0.1);
    end
    configureCallback(device,"off");
end

function waitForResponse(device)
    configureCallback(device,"terminator",@showArduinoOutput);
    global isWaiting
    isWaiting = true;
    while(isWaiting)
        pause(0.1);
    end
    configureCallback(device,"off");
end

function showArduinoOutput(src, ~)
    global isWaiting;
    disp(readline(src));
    isWaiting = false;
end