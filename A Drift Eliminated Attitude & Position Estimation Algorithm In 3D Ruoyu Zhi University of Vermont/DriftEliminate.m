clear;
close all;
clc;
addpath('Quaternions');
addpath('ximu_matlab_library');
tic;
% ---------------------------------------------------------------------

% Select dataset (comment in/out)
filePath = 'Datasets/aroundVotey';
AllDataX=load('ximu_matlab_library/Datasets/outsideRecNoMag2_CalInertialAndMag.csv');%spiralStairs_CalInertialAndMag   %aroundVotey_CalInertialAndMag.csv
%startTime = 0;stopTime = 136;

% sensor data ----------------------------------------------------
%load('AllData8.csv');format long
N=24517;time=0:0.008:0.008*(N-1);time=time';AllData=AllDataX(:,2:10);%xy =AllData(1:N,7:9);x=xy;%xy =Magnetometer(1:N,1:3);%A=x-y;xy =(-1)*xy;

Accelerometer(:,1)=AllData(:,1);Accelerometer(:,2)=AllData(:,2);Accelerometer(:,3)=AllData(:,3);%Accelerometer(:,1)=AllData(1:N,1);Accelerometer(:,2)=AllData(1:N,2);Accelerometer(:,3)=AllData(1:N,3);
Gyroscope(:,1)=AllData(:,4); Gyroscope(:,2)=AllData(:,5); Gyroscope(:,3)=AllData(:,6); 
Magnetometer(:,1)=AllData(:,7);Magnetometer(:,2)=AllData(:,8);Magnetometer(:,3)=AllData(:,9);%Gyroscope(1:3,1)=-0.48;Gyroscope(1:3,2)=-0.30;Gyroscope(1:3,2)=-0.06;

% indexSel =0:0.008:0.008*(N-1);
gyrX = Gyroscope(:,1);gyrY = Gyroscope(:,2);gyrZ = Gyroscope(:,3);%gyrX = Gyroscope(:,1)/10;gyrY = Gyroscope(:,2)/10;gyrZ = Gyroscope(:,3)/10;
accX = Accelerometer(:,1);accY = Accelerometer(:,2);accZ = Accelerometer(:,3);%accX = Accelerometer(:,1)/9.81;accY = Accelerometer(:,2)/9.81;accZ = Accelerometer(:,3)/9.81;
magX = Magnetometer(:,1);magY = Magnetometer(:,2);magZ = Magnetometer(:,3);
%-------------------------------------------------
% filePath = 'Datasets/aroundVotey1stCorner';
% startTime = 0;
% stopTime = 16.3;
% filePath = 'Datasets/aroundVoteyStraight';
% startTime = 5;
% stopTime = 35;
% filePath = 'Datasets/outsideRecNoMag2';
% startTime = 0;
% stopTime = 136;
% filePath = 'Datasets/VoteyUpStairs';
% startTime = 0;
% stopTime = 49.2;
% filePath = 'Datasets/VoteyUpStairsStepByStep';
% startTime = 0;
% stopTime = 60.9;
% ---------------------------------------------------------------------

% Import data
%  samplePeriod = 1/125;
% xIMUdata = xIMUdataClass(filePath, 'InertialMagneticSampleRate',1/samplePeriod);
% time = xIMUdata.CalInertialAndMagneticData;
% gyrX = xIMUdata.CalInertialAndMagneticData.Gyroscope.X / 10;
% gyrY = xIMUdata.CalInertialAndMagneticData.Gyroscope.Y / 10;
% gyrZ = xIMUdata.CalInertialAndMagneticData.Gyroscope.Z / 10;
% accX = xIMUdata.CalInertialAndMagneticData.Accelerometer.X / 9.81;
% accY = xIMUdata.CalInertialAndMagneticData.Accelerometer.Y / 9.81;
% accZ = xIMUdata.CalInertialAndMagneticData.Accelerometer.Z / 9.81;
% magX = xIMUdata.CalInertialAndMagneticData.Magnetometer.X;
% magY = xIMUdata.CalInertialAndMagneticData.Magnetometer.Y;
% magZ = xIMUdata.CalInertialAndMagneticData.Magnetometer.Z;
% clear('xIMUdata');
% % % ---------------------------------------------------------------------

% %Manually frame data
% % startTime = 0;
% % stopTime = 10;
 %indexSel = find(sign(time-startTime)+1, 1) : find(sign(time-stopTime)+1, 1);
% time = time(indexSel);
% gyrX = gyrX(indexSel);
% gyrY = gyrY(indexSel);
% gyrZ = gyrZ(indexSel);
% accX = accX(indexSel);
% accY = accY(indexSel);
% accZ = accZ(indexSel);
% magX = magX(indexSel);
% magY = magY(indexSel);
% magZ = magZ(indexSel);
% ---------------------------------------------------------------------
% Detect stationary periods
% Compute accelerometer magnitude
acc_mag = sqrt(accX.*accX + accY.*accY + accZ.*accZ);
% HP filter accelerometer data
filtCutOff = 0.0001;samplePeriod = 1/125;
[b, a] = butter(1, (2*filtCutOff)/(1/samplePeriod), 'high');
acc_magFilt = filtfilt(b, a, acc_mag);
% Compute absolute value
acc_magFilt = abs(acc_magFilt);
% LP filter accelerometer data
filtCutOff = 5;
[b, a] = butter(1, (2*filtCutOff)/(1/samplePeriod), 'low');
acc_magFilt = filtfilt(b, a, acc_magFilt);
% Threshold detection
stationary = acc_magFilt < 5.055;%stationary = acc_magFilt < 5.055;
% ---------------------------------------------------------------------
% Plot data raw sensor data and stationary periods
figure('Position', [9 39 900 600], 'NumberTitle', 'off', 'Name', 'SensorData');
ax(1) = subplot(2,1,1);
 hold on;
 plot(time, gyrX, 'r');
 plot(time, gyrY, 'g');
 plot(time, gyrZ, 'b');
 title('Gyroscope');
 xlabel('Time (s)');
 ylabel('Angular velocity (^\circ/s)');
 legend('X', 'Y', 'Z');
 hold off;
ax(2) = subplot(2,1,2);
 hold on;
 plot(time, accX, 'r');
 plot(time, accY, 'g');
 plot(time, accZ, 'b');
 plot(time, acc_magFilt, ':k');
 plot(time, stationary, 'k', 'LineWidth', 2);
 title('Accelerometer');
 xlabel('Time (s)');
 ylabel('Acceleration (g)');
 legend('X', 'Y', 'Z', 'Filtered', 'Stationary');
 hold off;
linkaxes(ax,'x');
% ---------------------------------------------------------------------
% Compute orientation
quat = zeros(length(time), 4);
AHRSalgorithm = AHRS('SamplePeriod', samplePeriod, 'Kp', 1, 'KpInit',1,'Ki',0);
% Initial convergence
initPeriod = 2;
indexSel = 1 : find(sign(time-(time(1)+initPeriod))+1, 1);
for i = 1:2000
 %AHRSalgorithm.UpdateIMU([0 0 0], [mean(accX(indexSel)) mean(accY(indexSel)) mean(accZ(indexSel))]);
 AHRSalgorithm.Update([0 0 0], [mean(accX(indexSel)) mean(accY(indexSel)) mean(accZ(indexSel))],[mean(magX(indexSel)) mean(magY(indexSel)) mean(magZ(indexSel))]);
end
% For all data
for t = 1:length(time)
 if(stationary(t))
 AHRSalgorithm.Kp = 0.5;
 else
 AHRSalgorithm.Kp = 0;
 end
 %AHRSalgorithm.UpdateIMU(deg2rad([gyrX(t) gyrY(t) gyrZ(t)]),[accX(t) accY(t) accZ(t)]);
 AHRSalgorithm.Update(deg2rad([gyrX(t) gyrY(t) gyrZ(t)]),[accX(t) accY(t) accZ(t)],[magX(t) magY(t) magZ(t)]);
 quat(t,:) = AHRSalgorithm.Quaternion;
end
% ---------------------------------------------------------------------
% Compute translational accelerations
% Rotate accelerations from sensor frame to Earth frame
acc = [accX accY accZ];
acc = quaternRotate(acc, quaternConj(quat));
% % Remove gravity from measurements
% acc = acc - [zeros(length(time), 2) ones(length(time), 1)]; %unnecessary due to velocity integral drift compensation
% Convert acceleration measurements to m/s/s
acc = acc * 9.81;
% Plot translational accelerations
figure('Position', [9 39 900 300], 'NumberTitle', 'off', 'Name','Accelerations');
hold on;
plot(time, acc(:,1), 'r');
plot(time, acc(:,2), 'g');
plot(time, acc(:,3), 'b');
title('Acceleration');
xlabel('Time (s)');
ylabel('Acceleration (m/s/s)');
legend('X', 'Y', 'Z');
hold off;
% ---------------------------------------------------------------------
% Compute translational velocities
acc(:,3) = acc(:,3) - 9.81;
% Integrate acceleration to yield velocity
vel = zeros(size(acc));
for t = 2:length(vel)
 vel(t,:) = vel(t-1,:) + acc(t,:) * samplePeriod;
 if(stationary(t) == 1)
 vel(t,:) = [0 0 0]; % apply ZUPT update when foot stationary
 end
end
% Compute integral drift during non-stationary periods
velDrift = zeros(size(vel));
stationaryStart = find([0; diff(stationary)] == 1);%stationaryStart = find([0; diff(stationary)] == 1);
stationaryEnd = find([0; diff(stationary)] == -1);%stationaryEnd = find([0; diff(stationary)] == -1);
for i = 1:numel(stationaryEnd)
 driftRate = vel(stationaryStart(i)-1, :)/(stationaryStart(i) - stationaryEnd(i));
 enum = 1:(stationaryStart(i) - stationaryEnd(i));
 drift = [enum'*driftRate(1) enum'*driftRate(2) enum'*driftRate(3)];
 velDrift(stationaryEnd(i):stationaryStart(i)-1, :) = drift;
end
% Remove integral drift
vel = vel - velDrift;
% vel = vel-vel;
% Plot translational velocity
figure('Position', [9 39 900 300], 'NumberTitle', 'off', 'Name','Velocity');
hold on;
plot(time, vel(:,1), 'r');
plot(time, vel(:,2), 'g');
plot(time, vel(:,3), 'b');
title('Velocity');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
legend('X', 'Y', 'Z');
hold off;
% ---------------------------------------------------------------------
% Compute translational position
% Integrate velocity to yield position
pos = zeros(size(vel));
for t = 2:length(pos)
 pos(t,:) = pos(t-1,:) + vel(t,:) * samplePeriod; % integrate velocity to yield position
end
toc;
% get position of the first 5 steps
first5X = pos(stationaryEnd(1:5),1);
first5Y = pos(stationaryEnd(1:5),2);
% use polyfit to calculate best fit straight line for the first five step and serve as X-Y dominant direction
p = polyfit(first5X,first5Y,1);
% Heuristic Drift Elimination
step = zeros(length(stationaryEnd),3);
headingDifferenceXP = zeros(length(step),1); % heading difference between current step and dominant XP direction
headingDifferenceYP = zeros(length(step),1); % heading difference between current step and dominant YP direction
headingDifferenceXN = zeros(length(step),1); % heading difference between current step and dominant XN direction
headingDifferenceYN = zeros(length(step),1); % heading difference between current step and dominant YN direction
HDEquat = zeros(length(step),4);
% start EHDE correction
for i = 1:length(stationaryEnd)
 % step calculated from position
 step(i,:) = pos(stationaryStart(i),:) - pos(stationaryEnd(i),:);
 % walking on flat floor
 if abs(step(i,3)) < 0.2
 fprintf('into plane correction.\n');

 % dominant direction in 3D determined
 domDirXP = [1,p(1),0];
 domDirYN = [1,-1/p(1),0];
 domDirXN = [-1,-p(1),0];
 domDirYP = [-1,1/p(1),0];
 % heading difference between current step and each of four dominant directions
 headingDifferenceXP(i) = getAngle(step(i,:),domDirXP);
 headingDifferenceYP(i) = getAngle(step(i,:),domDirYP);
 headingDifferenceXN(i) = getAngle(step(i,:),domDirXN);
 headingDifferenceYN(i) = getAngle(step(i,:),domDirYN);

 % determine which possible zone does current step fall into
 if abs(headingDifferenceXP(i)) < 0.2
 % calculate quaternion from current step to determined dominant direction
 HDEquat(i,:) = calcQuat(step(i,:), domDirXP);
 % rotate current step tp nearest dominant direction
 pos(stationaryEnd(i):length(pos),:) = quaternRotate(pos(stationaryEnd(i):length(pos),:),HDEquat(i,:));
 fprintf('plane correction to XP made %dth iteration.\n',i);
 % if not the first step, parallel move the remaining whole trajectory to the end of previous step
 if i >= 2
 pos(stationaryEnd(i):length(pos),1) = pos(stationaryEnd(i):length(pos),1) - ( pos(stationaryEnd(i),1) - pos(stationaryStart(i-1),1) );
 pos(stationaryEnd(i):length(pos),2) = pos(stationaryEnd(i):length(pos),2) - ( pos(stationaryEnd(i),2) - pos(stationaryStart(i-1),2) );
 pos(stationaryEnd(i):length(pos),3) = pos(stationaryEnd(i):length(pos),3) - ( pos(stationaryEnd(i),3) - pos(stationaryStart(i-1),3) );
 end
 elseif abs(headingDifferenceYP(i)) < 0.2
 HDEquat(i,:) = calcQuat(step(i,:), domDirYP);
 pos(stationaryEnd(i):length(pos),:) = quaternRotate(pos(stationaryEnd(i):length(pos),:),HDEquat(i,:));
 fprintf('plane correction to YP made %dth iteration.\n',i);
 pos(stationaryEnd(i):length(pos),1) = pos(stationaryEnd(i):length(pos),1) - ( pos(stationaryEnd(i),1) - pos(stationaryStart(i-1),1) );
 pos(stationaryEnd(i):length(pos),2) = pos(stationaryEnd(i):length(pos),2) - ( pos(stationaryEnd(i),2) - pos(stationaryStart(i-1),2) );
 pos(stationaryEnd(i):length(pos),3) = pos(stationaryEnd(i):length(pos),3) - ( pos(stationaryEnd(i),3) - pos(stationaryStart(i-1),3) );
 elseif abs(headingDifferenceXN(i)) < 0.2
 HDEquat(i,:) = calcQuat(step(i,:), domDirXN);
 fprintf('plane correction to XN made %dth iteration.\n',i);
 pos(stationaryEnd(i):length(pos),:) = quaternRotate(pos(stationaryEnd(i):length(pos),:),HDEquat(i,:));
 pos(stationaryEnd(i):length(pos),1) = pos(stationaryEnd(i):length(pos),1) - ( pos(stationaryEnd(i),1) - pos(stationaryStart(i-1),1) );
 pos(stationaryEnd(i):length(pos),2) = pos(stationaryEnd(i):length(pos),2) - ( pos(stationaryEnd(i),2) - pos(stationaryStart(i-1),2) );
 pos(stationaryEnd(i):length(pos),3) = pos(stationaryEnd(i):length(pos),3) - ( pos(stationaryEnd(i),3) - pos(stationaryStart(i-1),3) );
 elseif abs(headingDifferenceYN(i)) < 0.2
 HDEquat(i,:) = calcQuat(step(i,:), domDirYN);
 fprintf('plane correction to YN made %dth iteration.\n',i);
 pos(stationaryEnd(i):length(pos),:) = quaternRotate(pos(stationaryEnd(i):length(pos),:),HDEquat(i,:));
 pos(stationaryEnd(i):length(pos),1) = pos(stationaryEnd(i):length(pos),1) - ( pos(stationaryEnd(i),1) - pos(stationaryStart(i-1),1) );
 pos(stationaryEnd(i):length(pos),2) = pos(stationaryEnd(i):length(pos),2) - ( pos(stationaryEnd(i),2) - pos(stationaryStart(i-1),2) );
 pos(stationaryEnd(i):length(pos),3) = pos(stationaryEnd(i):length(pos),3) - ( pos(stationaryEnd(i),3) - pos(stationaryStart(i-1),3) );
 end
 % if walking along stairs
 elseif abs(step(i,3)) > 0.2
 fprintf('into stair correction.\n');

 % dominant direction in 3D determined, gradient remained
 domDirXP = [1,p(1),step(i,3) * norm([1,p(1)]) / norm(step(i,1:2))];
 domDirYN = [1,-1/p(1),step(i,3) * norm([1,-1/p(1)]) /norm(step(i,1:2))];
 domDirXN = [-1,-p(1),step(i,3) * norm([-1,p(1)]) /norm(step(i,1:2))];
 domDirYP = [-1,1/p(1),step(i,3) * norm([-1,1/p(1)]) / norm(step(i,1:2))];

 % heading difference between current step and each of four dominant directions
 headingDifferenceXP(i) = getAngle(step(i,:),domDirXP);
 headingDifferenceYP(i) = getAngle(step(i,:),domDirYP);
 headingDifferenceXN(i) = getAngle(step(i,:),domDirXN);
 headingDifferenceYN(i) = getAngle(step(i,:),domDirYN);

 % determine which possible zone does current step fall into and correct current step to nearest dominant direction
 if abs(headingDifferenceXP(i)) < 0.4
 HDEquat(i,:) = calcQuat(step(i,:), domDirXP);
 fprintf('stair correction to XP made %dth iteration.\n',i);
 pos(stationaryEnd(i):length(pos),:) = quaternRotate(pos(stationaryEnd(i):length(pos),:),HDEquat(i,:));
 pos(stationaryEnd(i):length(pos),1) = pos(stationaryEnd(i):length(pos),1) - ( pos(stationaryEnd(i),1) - pos(stationaryStart(i-1),1) );
 pos(stationaryEnd(i):length(pos),2) = pos(stationaryEnd(i):length(pos),2) - ( pos(stationaryEnd(i),2) - pos(stationaryStart(i-1),2) );
 pos(stationaryEnd(i):length(pos),3) = pos(stationaryEnd(i):length(pos),3) - ( pos(stationaryEnd(i),3) - pos(stationaryStart(i-1),3) );
 elseif abs(headingDifferenceYP(i)) < 0.4
 HDEquat(i,:) = calcQuat(step(i,:), domDirYP);
 fprintf('stair correction to YP made %dth iteration.\n',i);
 pos(stationaryEnd(i):length(pos),:) = quaternRotate(pos(stationaryEnd(i):length(pos),:),HDEquat(i,:));
 pos(stationaryEnd(i):length(pos),1) = pos(stationaryEnd(i):length(pos),1) - ( pos(stationaryEnd(i),1) - pos(stationaryStart(i-1),1) );
 pos(stationaryEnd(i):length(pos),2) = pos(stationaryEnd(i):length(pos),2) - ( pos(stationaryEnd(i),2) - pos(stationaryStart(i-1),2) );
 pos(stationaryEnd(i):length(pos),3) = pos(stationaryEnd(i):length(pos),3) - ( pos(stationaryEnd(i),3) - pos(stationaryStart(i-1),3) );
 elseif abs(headingDifferenceXN(i)) < 0.4
 HDEquat(i,:) = calcQuat(step(i,:), domDirXN);
 fprintf('stair correction to XN made %dth iteration.\n',i);
 pos(stationaryEnd(i):length(pos),:) = quaternRotate(pos(stationaryEnd(i):length(pos),:),HDEquat(i,:));
 pos(stationaryEnd(i):length(pos),1) = pos(stationaryEnd(i):length(pos),1) - ( pos(stationaryEnd(i),1) - pos(stationaryStart(i-1),1) );
 pos(stationaryEnd(i):length(pos),2) = pos(stationaryEnd(i):length(pos),2) - ( pos(stationaryEnd(i),2) - pos(stationaryStart(i-1),2) );
 pos(stationaryEnd(i):length(pos),3) = pos(stationaryEnd(i):length(pos),3) - ( pos(stationaryEnd(i),3) - pos(stationaryStart(i-1),3) );
 elseif abs(headingDifferenceYN(i)) < 0.4
 HDEquat(i,:) = calcQuat(step(i,:), domDirYN);
 fprintf('stair correction to YN made %dth iteration.\n',i);
 pos(stationaryEnd(i):length(pos),:) = quaternRotate(pos(stationaryEnd(i):length(pos),:),HDEquat(i,:));  
 pos(stationaryEnd(i):length(pos),1) = pos(stationaryEnd(i):length(pos),1) - ( pos(stationaryEnd(i),1) - pos(stationaryStart(i-1),1) );
 pos(stationaryEnd(i):length(pos),2) = pos(stationaryEnd(i):length(pos),2) - ( pos(stationaryEnd(i),2) - pos(stationaryStart(i-1),2) );
 pos(stationaryEnd(i):length(pos),3) = pos(stationaryEnd(i):length(pos),3) - ( pos(stationaryEnd(i),3) - pos(stationaryStart(i-1),3) );
 end
 end
end
% % calculate heading difference between curret step and previous step in right hand coordinate
% headingBetweenSteps = zeros(length(step)-1,1);
% for i = 1:length(headingBetweenSteps)
% headingBetweenSteps(i) = getAngle(step(i,:),step(i+1,:));
% end;
% Plot translational position
figure('Position', [9 39 900 600], 'NumberTitle', 'off', 'Name','Position');
hold on;
plot(time, pos(:,1), 'r');
plot(time, pos(:,2), 'g');
plot(time, pos(:,3), 'b');
title('Position');
xlabel('Time (s)');
ylabel('Position (m)');
legend('X', 'Y', 'Z');
hold off;
% Plot quaternion
figure('Position', [9 39 900 600], 'NumberTitle', 'off', 'Name','Quaternions');hold on;
plot(quat);title('Quaternions');
xlabel('Time (s)');
ylabel('Position (m)');
legend('1', '2', '3', '4');hold off;
% ---------------------------------------------------------------------
% Plot 3D foot trajectory
% % Remove stationary periods from data to plot
% posPlot = pos(find(~stationary), :);
% quatPlot = quat(find(~stationary), :);
posPlot = pos;
quatPlot = quat;
% Extend final sample to delay end of animation
extraTime = 5;
onesVector = ones(extraTime*(1/samplePeriod), 1);
posPlot = [posPlot; [posPlot(end, 1)*onesVector, posPlot(end,2)*onesVector, posPlot(end, 3)*onesVector]];
quatPlot = [quatPlot; [quatPlot(end, 1)*onesVector, quatPlot(end, 2)*onesVector, quatPlot(end, 3)*onesVector, quatPlot(end, 4)*onesVector]];
toc;
% Create 6 DOF animation
SamplePlotFreq = 30;
Spin = 50;
SixDofAnimation(posPlot, quatern2rotMat(quatPlot), ... 
    'SamplePlotFreq', SamplePlotFreq, 'Trail', 'DotsOnly', ...
 'Position', [9 39 1280 768], 'View', [(100:(Spin/(length(posPlot)-1)):(100+Spin))', 10*ones(length(posPlot),1)], ... 
 'AxisLength', 0.2, 'ShowArrowHead', false, ...
    'Xlabel', 'X (m)', 'Ylabel', 'Y (m)', 'Zlabel', 'Z(m)', 'ShowLegend', false, ...
 'CreateAVI', false, 'AVIfileNameEnum', true,'AVIfileName', '9Dof', 'AVIfps', ((1/samplePeriod) / SamplePlotFreq));