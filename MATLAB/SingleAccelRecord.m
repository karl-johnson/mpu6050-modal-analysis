clear all; close all; fclose all; delete(instrfind); clc; % Start Fresh
REC_LENGTH = 8192; % Maximum number of recorded samples
SAMPLING_RATE = 500; % Hz
ACCEL_RANGE = 4;
MAX_RAW_OUTPUT = 32768;
G_CONVERSION = ACCEL_RANGE/MAX_RAW_OUTPUT;
%% Connect to Arduino through Serial
ARDUINO_SERIAL_PORT = 'COM8'; % Change this as needed
arduinoSerial = serialport(ARDUINO_SERIAL_PORT,115200); % Create Serial Object a to connect to arduino (choose com port correctly)
pause(1) % Wait data on Serial
disp("Ready to begin acquisition, press any key to begin: ");
pause;
numRecordedSamples = 0;
flush(arduinoSerial);
startTime = tic;
recordedData = zeros(REC_LENGTH, 3);
recordedRaw = zeros(REC_LENGTH, 6);
while(numRecordedSamples < REC_LENGTH)
    if(arduinoSerial.NumBytesAvailable >= 7)
        tempBytes = uint8(char(readline(arduinoSerial)));
        %disp(tempBytes);
        if(length(tempBytes) == 6)
            numRecordedSamples = numRecordedSamples+1;
            recordedRaw(numRecordedSamples,:) = tempBytes;
            [x, y, z] = getAccelVals(tempBytes,G_CONVERSION);
            recordedData(numRecordedSamples,:) = [x, y, z];
        end
    end
    %if(mod(numRecordedSamples,round(REC_LENGTH/10))==0)
    %    disp([num2str(round(100*numRecordedSamples/REC_LENGTH)) '%']);
    %  end
end
disp("Collection finished!");
timeArray = (1/SAMPLING_RATE)*(1:REC_LENGTH);
toc(startTime);
%%
startT=750;
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
freqHz = (0:(length(fftX)-1))*SAMPLING_RATE/length(fftX);
semilogy(freqHz,fftX, "DisplayName", "X");
hold on;
xlabel("Frequency (Hz)");
ylabel("Power spectral density (g/Hz)");
semilogy(freqHz,fftY, "DisplayName", "Y");
semilogy(freqHz,fftZ, "DisplayName", "Z");
hold off;
legend;
xlim([SAMPLING_RATE/length(fftX), SAMPLING_RATE/2]);
grid on
%% save
save("Table1.mat");
xlim([SAMPLING_RATE/length(fftX) SAMPLING_RATE/2]);
function [x, y, z] = getAccelVals(lineIn,G_CONVERSION)
    % take 6-byte array and return accelerometer values scaled in g
    x = G_CONVERSION*double(swapbytes(typecast(lineIn(1:2), 'int16')));
    y = G_CONVERSION*double(swapbytes(typecast(lineIn(3:4), 'int16')));
    z = G_CONVERSION*double(swapbytes(typecast(lineIn(5:6), 'int16')));
end