close all;                          % close all figures
clear;                              % clear all variables
clc;                                % clear the command terminal

%-------Madgw
 %addpath('quaternion_library');  load('ExampleData.mat');
% 
% %-------AllData/Real/Optoel
load('AllData4.csv');N=1600;
Accelerometer=AllData4(1:N,1:3);Gyroscope=AllData4(1:N,1:3);Magnetometer=AllData4(1:N,1:3);

%-------Data
%load 'rpy_9axis.mat' sensorData Fs;accelerometerReadings = sensorData.Acceleration;gyroscopeReadings = sensorData.AngularVelocity;   
load 'rpy_9axis' sensorData Fs;accelerometerReadings = sensorData.Acceleration;gyroscopeReadings = sensorData.AngularVelocity;magnetometerReadings = sensorData.MagneticField;
decim = 100;%decim = 40;

%-------Filter
%fuse = imufilter('SampleRate',125,'DecimationFactor',decim); 
%fuse = ahrsfilter('SampleRate',125,'DecimationFactor',decim);fuse = complementaryFilter('SampleRate',125);

%q = fuse(accelerometerReadings,gyroscopeReadings,magnetometerReadings);
%q = fuse(Accelerometer(1:N,:),Gyroscope(1:N,:),Magnetometer(1:N,:));
q = fuse(Accelerometer(1:N,:),Gyroscope(1:N,:));%q = fuse(accelerometerReadings,gyroscopeReadings);

%time1 = (0:decim:size(accelerometerReadings,1)-1)/Fs;
%plot(time1,eulerd(q,'ZYX','frame'))
plot(eulerd(q,'ZYX','frame'))

title('Orientation Estimate')
legend('Z-axis', 'Y-axis', 'X-axis')
xlabel('Time (s)')
ylabel('Rotation (degrees)')

%%%-------------------------------------------------------------------------------
% clear all;close all;clc;
% 
% fs =5000;
% firstLoopNumSamples = fs*4;secondLoopNumSamples = fs*2;totalNumSamples = firstLoopNumSamples + secondLoopNumSamples;
% traj = kinematicTrajectory('SampleRate',fs);
% 
% accBody = zeros(totalNumSamples,3);angVelBody = zeros(totalNumSamples,3);angVelBody(1:firstLoopNumSamples,3) = (2*pi)/4;
% angVelBody(firstLoopNumSamples+1:end,3) = (2*pi)/2;
% 
% [~,orientationNED,~,accNED,angVelNED] = traj(accBody,angVelBody);
% 
% IMU = imuSensor('accel-mag','SampleRate',fs);
% 
% IMU.Accelerometer = accelparams( ...
%     'MeasurementRange',19.62, ...            % m/s^2
%     'Resolution',0.0023936, ...              % m/s^2 / LSB
%     'TemperatureScaleFactor',0.008, ...      % % / degree C
%     'ConstantBias',0.1962, ...               % m/s^2
%     'TemperatureBias',0.0014715, ...         % m/s^2 / degree C
%     'NoiseDensity',0.0012361);               % m/s^2 / Hz^(1/2)
% 
% IMU.Magnetometer = magparams( ...
%     'MeasurementRange',1200, ...             % uT
%     'Resolution',.1, ...                    % uT / LSB
%     'TemperatureScaleFactor',0.1, ...        % % / degree C
%     'ConstantBias',1, ...                    % uT
%     'TemperatureBias',[0.8 0.8 2.4], ...     % uT / degree C
%     'NoiseDensity',[0.6 0.6 0.9]/sqrt(1)); % uT / Hz^(1/2)
% 
% [accelReadings,magReadings] = IMU(accNED,angVelNED,orientationNED);
% figure;scatter3(magReadings(:,1),magReadings(:,2),magReadings(:,3));axis equal;title(' Magnetometer Data');
% 
% [A,b,expMFS]  = magcal(magReadings);xCorrected = (magReadings-b)*A;%r=z-xCorrected;
% figure;scatter3(xCorrected(:,1),xCorrected(:,2),xCorrected(:,3));axis equal;title(' MagCalibrated Magnetometer Data');
% r = sum(xCorrected.^2,2) - expMFS.^2;E = sqrt(r.'*r./fs)./(2*expMFS.^2);
% fprintf('Residual error in corrected data : %.2f\n\n',E);
% 
% figure
% t = (0:(totalNumSamples-1))/fs;
% subplot(2,1,1)
% plot(t,accelReadings)
% legend('X-axis','Y-axis','Z-axis')
% ylabel('Acceleration (m/s^2)')
% title('Accelerometer Readings')
% 
% subplot(2,1,2)
% plot(t,magReadings)
% legend('X-axis','Y-axis','Z-axis')
% ylabel('Magnetic Field (\muT)')
% xlabel('Time (s)')
% title('Magnetometer Readings')
% 
% windowSize = 500; b = (1/windowSize)*ones(1,windowSize);a = 1;
% y = filter(b,a,magReadings(:,:));magReadings=y;
% 
% figure
% plot(t,magReadings)
% legend('X-axis','Y-axis','Z-axis')
% ylabel('Magnetic Field (\muT)')
% xlabel('Time (s)')
% title('Magnetometer Readings')