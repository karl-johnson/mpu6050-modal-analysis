clear; close all; fclose all; delete(instrfind); clc; % Start Fresh
BAUD_RATE = 115200;
REC_LENGTH = 8192; % Length of recording, samples (power of 2 recommended)
ACCEL_FREQ = 500; % Recording rate, Hz (100,200,250,333,or 500)
% NOTE: right now LPF is fixed at 184 Hz, will add setting eventually
ACCEL_SCALE = 2; % Max accel reading, g (+/-) (2,4,8, or 16)
MAX_RAW_OUTPUT = 32768; % no touch
G_CONVERSION = ACCEL_SCALE/MAX_RAW_OUTPUT; % no touch
% Connect to Arduino through Serial
ARDUINO_SERIAL_PORT = 'COM8'; % Change this as needed
global isWaiting;
isWaiting = false;
clear arduinoSerial;
arduinoSerial = serialport(ARDUINO_SERIAL_PORT,BAUD_RATE); % Create Serial Object a to connect to arduino (choose com port correctly)
configureTerminator(arduinoSerial,uint8(newline));
configureCallback(arduinoSerial,"terminator",@showArduinoOutput);
isWaiting = true;
while(isWaiting)
    pause(0.1); % if we don't put a pause, callbacks don't run
end
disp("Writing settings to accelerometer...");
disp("Sending frequency...");
writeLineAndWait(arduinoSerial, ['f' int2str(ACCEL_FREQ)]);
disp("Sending scale...");
writeLineAndWait(arduinoSerial, ['s' int2str(ACCEL_SCALE)]);
disp("Ready to begin acquisition, press any key to begin: ");
pause;
numRecordedSamples = 0;
flush(arduinoSerial);
startTime = tic;
lastPrintTime = 0;
recordedData = zeros(REC_LENGTH, 3);
recordedRaw = uint8(zeros(REC_LENGTH,6));
hasPrintedProgress = false;
writeline(arduinoSerial, ['g' int2str(REC_LENGTH)]);
pause(1);
while(numRecordedSamples < REC_LENGTH)
    dataIn = uint8(read(arduinoSerial,7,'uint8')); % includes terminator :/
    dataIn = dataIn(1:6);
    numRecordedSamples = numRecordedSamples+1;
    recordedRaw(numRecordedSamples,:) = dataIn;
    [x, y, z] = getAccelVals(dataIn,G_CONVERSION);
    recordedData(numRecordedSamples,:) = [x, y, z];
    if(toc(startTime) - lastPrintTime > 1)
        lastPrintTime = toc(startTime);
        disp(['t = +' num2str(toc(startTime)) 's: ' int2str(numRecordedSamples) '/' int2str(REC_LENGTH)]);
    end
end
disp("Collection finished!");
timeArray = (1/ACCEL_FREQ)*(1:REC_LENGTH);
toc(startTime);
%%
startT=1;
endT=2048;
figure;
hold on;
plot(timeArray,recordedData(:,1), "DisplayName", "X");
plot(timeArray,recordedData(:,2), "DisplayName", "Y");
plot(timeArray,recordedData(:,3), "DisplayName", "Z");
xline(timeArray(startT));
xline(timeArray(endT));
hold off;
xlabel("Time (s)");
ylabel("Acceleration");
legend;
clear arduinoSerial
%% Plot FFTs

figure;
fftX=abs(fft(recordedData(startT:endT,1))).^2;
fftY=abs(fft(recordedData(startT:endT,2))).^2;
fftZ=abs(fft(recordedData(startT:endT,3))).^2;
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
ylabel("Power Spectral Density (|FFT|^2)");
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

function showArduinoOutput(src, ~)
    global isWaiting;
    disp(readline(src));
    isWaiting = false;
end