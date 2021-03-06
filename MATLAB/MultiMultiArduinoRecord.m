% Read from multiple Arduinos, each with some number of accelerometers
clear; close all; fclose all; delete(instrfind); clc; % Start Fresh (TM)
% USER SETTINGS
% The following line only works if
% - the only available COM ports are Arduinos
% - all Arduinos have the right firmware uploaded
ARDUINO_SERIAL_PORTS = serialportlist;
REC_LENGTH = 2048; % Length of recording, samples (power of 2 recommended)
ACCEL_FREQ = 200; % Accel sample rate, Hz (100,200,250,333,or 500)
% RIGHT NOW MAX 200 HZ
    ACCEL_SCALE = 2; % Accel range, g (+/-) (2,4,8, or 16)
PLOT_HISTORY = 500;
UPDATE_RATE = 20; %  Update plots this many times per second
% no touch
BAUD_RATE = 115200;
MAX_RAW_OUTPUT = 32768;
G_CONVERSION = ACCEL_SCALE/MAX_RAW_OUTPUT;
% Connect to Arduinos through Serial
NUM_ARDUINOS = length(ARDUINO_SERIAL_PORTS);
clear serialPortArray;
serialPortArray = [];
numAccelsPerPort = zeros(1,NUM_ARDUINOS);
accelNames = [];
for portIndex = 1:NUM_ARDUINOS
    disp(['Connecting to ' char(ARDUINO_SERIAL_PORTS(portIndex)) '...']);
    thisSerialPort = serialport(ARDUINO_SERIAL_PORTS(portIndex),BAUD_RATE);
    %configureTerminator(thisSerialPort,uint8(newline));
    %configureCallback(thisSerialPort,"terminator",@showArduinoOutput);
    startupMessage = char(readline(thisSerialPort));
    disp(startupMessage);
    numAccelsPerPort(portIndex) = str2double(startupMessage(1));
    serialPortArray = [serialPortArray; thisSerialPort];
    for accelIndex = 1:numAccelsPerPort(portIndex)
        totalAccelIndex = getTotalIndex(numAccelsPerPort, portIndex, accelIndex);
        accelNames = [accelNames string([char(ARDUINO_SERIAL_PORTS(portIndex)) '.' num2str(accelIndex)])];
    end
end
totalAccels = sum(numAccelsPerPort,'all');
% Configure settings on all accelerometers
for portIndex = 1:NUM_ARDUINOS
    disp(['Writing settings to ' char(ARDUINO_SERIAL_PORTS(portIndex)) '...']);
    disp("Sending frequency...");
    writeLineAndWait(serialPortArray(portIndex), ['f' int2str(ACCEL_FREQ)]);
    disp("Sending scale...");
    writeLineAndWait(serialPortArray(portIndex), ['s' int2str(ACCEL_SCALE)]);
end

disp("Ready to begin acquisition, press any key to begin: ");
pause;

% do a lil prep
numRecordedSamples = 0;
recordedData = cell(NUM_ARDUINOS,1);
for portIndex = 1:NUM_ARDUINOS
    flush(serialPortArray(portIndex));
    recordedData{portIndex} = zeros(REC_LENGTH, 3);
end

% some timing setup
lastPrintTime = 0;
startTime = tic;
% start all arduinos in as quick of succession as possible in Matlab
for portIndex = 1:NUM_ARDUINOS
    writeline(serialPortArray(portIndex), ['g' int2str(REC_LENGTH)]);
end
while(numRecordedSamples < REC_LENGTH)
    for portIndex = 1:NUM_ARDUINOS
        % blocking read
        numBytes = 6*numAccelsPerPort(portIndex) + 1;
        dataIn = uint8(read(serialPortArray(portIndex),numBytes,'uint8')); % includes terminator :/
        dataIn = dataIn(1:end-1); % remove terminator
        % TODO stop using terminator, it's useless
        for accelIndex = 1:numAccelsPerPort(portIndex)
            totalAccelIndex = getTotalIndex(numAccelsPerPort, portIndex, accelIndex);
            [x, y, z] = getAccelVals(dataIn(6*(accelIndex-1)+1:6*accelIndex),G_CONVERSION);
            recordedData{totalAccelIndex}(numRecordedSamples+1,:) = [x, y, z];
        end
    end
    numRecordedSamples = numRecordedSamples+1;
    if(toc(startTime) - lastPrintTime > 1)
        lastPrintTime = toc(startTime);
        disp(['t = +' num2str(toc(startTime)) 's: ' int2str(numRecordedSamples) '/' int2str(REC_LENGTH)]);
    end
end
disp("Collection finished!");
toc(startTime);
uisave();
clear serialPortArray;
%%
timeArray = (1/ACCEL_FREQ)*(1:REC_LENGTH);
figure;
tiledlayout(totalAccels,1);
for portIndex = 1:NUM_ARDUINOS
    for accelIndex = 1:numAccelsPerPort(portIndex)
        totalAccelIndex = getTotalIndex(numAccelsPerPort, portIndex, accelIndex);
        nexttile
        hold on;
        plot(timeArray,recordedData{totalAccelIndex}(:,1), "DisplayName", "X");
        plot(timeArray,recordedData{totalAccelIndex}(:,2), "DisplayName", "Y");
        plot(timeArray,recordedData{totalAccelIndex}(:,3), "DisplayName", "Z");
        hold off;
        xlabel("Time (s)");
        ylabel("Acceleration");
        ylim([-ACCEL_SCALE ACCEL_SCALE]);
        title(accelNames(totalAccelIndex));
    end
end
%%
totalAccelIndex = 1;
figure;
hold on;
plot(timeArray,recordedData{totalAccelIndex}(:,1), "DisplayName", "X");
plot(timeArray,recordedData{totalAccelIndex}(:,2), "DisplayName", "Y");
plot(timeArray,recordedData{totalAccelIndex}(:,3), "DisplayName", "Z");
hold off;
xlabel("Time (s)");
ylabel("Acceleration");
ylim([-ACCEL_SCALE ACCEL_SCALE]);
title(accelNames(totalAccelIndex));
%% Plot FFTs
figure;
tiledlayout(totalAccels,1);
for portIndex = 1:NUM_ARDUINOS
    for accelIndex = 1:numAccelsPerPort(portIndex)
        totalAccelIndex = getTotalIndex(numAccelsPerPort, portIndex, accelIndex);
        nexttile
        fftX=abs(fft(recordedData{totalAccelIndex}(:,1))).^2;
        fftY=abs(fft(recordedData{totalAccelIndex}(:,2))).^2;
        fftZ=abs(fft(recordedData{totalAccelIndex}(:,3))).^2;
        freqHz = (0:(length(fftX)-1))*ACCEL_FREQ/length(fftX);
        loglog(freqHz,fftX, "DisplayName", "X");
        hold on;
        loglog(freqHz,fftY, "DisplayName", "Y");
        loglog(freqHz,fftZ, "DisplayName", "Z");
        hold off;
        legend;
        grid on
        xlim([ACCEL_FREQ/length(fftX) ACCEL_FREQ/2]);
        xlabel("Frequency (Hz)");
        ylabel("PSD");
        ylim([0.1 1000]);
        title(accelNames(totalAccelIndex));
    end
end
%%
totalAccelIndex = 1;
fftX=abs(fft(recordedData{totalAccelIndex}(:,1))).^2;
fftY=abs(fft(recordedData{totalAccelIndex}(:,2))).^2;
fftZ=abs(fft(recordedData{totalAccelIndex}(:,3))).^2;
freqHz = (0:(length(fftX)-1))*ACCEL_FREQ/length(fftX);
figure;
loglog(freqHz,fftX, "DisplayName", "X");
hold on;
loglog(freqHz,fftY, "DisplayName", "Y");
loglog(freqHz,fftZ, "DisplayName", "Z");
hold off;
legend;
grid on
xlim([ACCEL_FREQ/length(fftX) ACCEL_FREQ/2]);
xlabel("Frequency (Hz)");
ylabel("PSD");
%ylim([0.1 1000]);
title(accelNames(totalAccelIndex));
%% Functions
function index = getTotalIndex(numAccelsPerPort, portIndex, accelIndex)
    index = sum(numAccelsPerPort(1:portIndex-1)) + accelIndex;
end

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