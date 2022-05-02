clear; close all; fclose all; delete(instrfind); clc; % Start Fresh (TM)
% Serial settings
BAUD_RATE = 115200;
ARDUINO_SERIAL_PORTS = ["COM9", "COM4", "COM21"];
NUM_ARDUINOS = length(ARDUINO_SERIAL_PORTS);
clear serialPortArray;
serialPortArray = [];
% Accelerometer settings
REC_LENGTH = 8192; % Length of recording, samples (power of 2 recommended)
ACCEL_FREQ = 500; % Recording rate, Hz (100,200,250,333,or 500)
% NOTE: right now LPF is fixed at 184 Hz, will add setting eventually
ACCEL_SCALE = 2; % Max accel reading, g (+/-) (2,4,8, or 16)
MAX_RAW_OUTPUT = 32768; % no touch
G_CONVERSION = ACCEL_SCALE/MAX_RAW_OUTPUT; % no touch
% Connect to Arduinos through Serial
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
        dataIn = uint8(read(serialPortArray(portIndex),7,'uint8')); % includes terminator :/
        dataIn = dataIn(1:6); % remove terminator
        % TODO stop using terminator, it's useless
        [x, y, z] = getAccelVals(dataIn,G_CONVERSION);
        recordedData{portIndex}(numRecordedSamples+1,:) = [x, y, z];
    end
    numRecordedSamples = numRecordedSamples+1;
    if(toc(startTime) - lastPrintTime > 1)
        lastPrintTime = toc(startTime);
        disp(['t = +' num2str(toc(startTime)) 's: ' int2str(numRecordedSamples) '/' int2str(REC_LENGTH)]);
    end
end
disp("Collection finished!");
toc(startTime);
%%
timeArray = (1/ACCEL_FREQ)*(1:REC_LENGTH);
figure;
tiledlayout(NUM_ARDUINOS,1);
for portIndex = 1:NUM_ARDUINOS
    nexttile
    hold on;
    plot(timeArray,recordedData{portIndex}(:,1), "DisplayName", "X");
    plot(timeArray,recordedData{portIndex}(:,2), "DisplayName", "Y");
    plot(timeArray,recordedData{portIndex}(:,3), "DisplayName", "Z");
    hold off;
    xlabel("Time (s)");
    ylabel("Acceleration");
    legend;
    title(['Accelerometer on ' char(ARDUINO_SERIAL_PORTS(portIndex))]);
end
%% Plot FFTs

figure;
tiledlayout(NUM_ARDUINOS,1);
for portIndex = 1:NUM_ARDUINOS
    nexttile
    fftX=abs(fft(recordedData{portIndex}(:,1))).^2;
    fftY=abs(fft(recordedData{portIndex}(:,2))).^2;
    fftZ=abs(fft(recordedData{portIndex}(:,3))).^2;
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
    title(['Accelerometer on ' char(ARDUINO_SERIAL_PORTS(portIndex))]);
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