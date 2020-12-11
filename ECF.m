addpath('quaternion_library'); clear all;close all;clc;
load('AllData8.csv');%load('AllData_Double_deney2.csv');%load('ExampleData.mat');

% SENSOR DATA ----------------------------------------------------
% Ideal IMU Data
% acc = zeros(N,3);av = zeros(N,3);rng(1);% q = randrot(N,1); % uniformly distributed random rotations
% imu = imuSensor('accel-mag');%imu2 = imuSensor('accel-gyro');% [~,x] = imu(acc,av,q);

format long
N=7485;time=0:0.008:0.008*(N-1);time=time';AllData=AllData8;%xy =AllData(1:N,7:9);x=xy;%xy =Magnetometer(1:N,1:3);%A=x-y;xy =(-1)*xy;

Accelerometer(:,1)=AllData(:,2);Accelerometer(:,2)=AllData(:,1);Accelerometer(:,3)=AllData(:,3);%Accelerometer(:,1)=AllData(1:N,1);Accelerometer(:,2)=AllData(1:N,2);Accelerometer(:,3)=AllData(1:N,3);
Gyroscope(:,1)=AllData(:,4); Gyroscope(:,2)=AllData(:,5); Gyroscope(:,3)=AllData(:,6); 
Magnetometer(:,1)=-AllData(:,8);Magnetometer(:,2)=AllData(:,7);Magnetometer(:,3)=AllData(:,9);%Gyroscope(1:3,1)=-0.48;Gyroscope(1:3,2)=-0.30;Gyroscope(1:3,2)=-0.06;

% scatter3(Magnetometer(:,1),Magnetometer(:,2),Magnetometer(:,3));axis equal;title('Real Magnetometer Data');%figure;scatter3(xy(:,1),xy(:,2),xy(:,3));axis equal;title('Madgwick Magnetometer Data');
 
% FILTERED -(SENSOR) DATA 
windowSize = 500; b = (1/windowSize)*ones(1,windowSize);a = 1;
y = filter(b,a,Gyroscope(:,:));Gyroscope=y;w = filter(b,a,Accelerometer(:,:));Accelerometer=w;z = filter(b,a,Magnetometer(:,:));Magnetometer=z;

% CALIBRATED HSI-Magnetometer (Hard and Soft Iron Effects Cal)----------------------
 
 [A,b1,expMFS]  = magcal(Magnetometer);xCorrected = (Magnetometer-b1)*A;%r=z-xCorrected;
 % [Axy,bxy,expMFSxy]  = magcal(t,'eye');xCorrected2 = (t-bxy)*Axy;%r=z-xCorrected;
 % [Adiag,bdiag,expMFSdiag] = magcal(t,'diag');xDiagCorrected = (t-bdiag)*Adiag;
 
% Plot Magnetometer -Calibrated ---------------------------------------------------
  %figure;scatter3(xCorrected(:,1),xCorrected(:,2),xCorrected(:,3));axis equal;title('Magnetometer Data Calibrated');

%% SENSOR Data - Import and plot

figure('Name', 'Sensor Data');
axis(1) = subplot(3,1,1);
hold on;
plot(time, Gyroscope(:,1), 'r');
plot(time, Gyroscope(:,2), 'g');
plot(time, Gyroscope(:,3), 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Angular rate (deg/s)');
title('Gyroscope');
hold off;
axis(2) = subplot(3,1,2);
hold on;
plot(time, Accelerometer(:,1), 'r');
plot(time, Accelerometer(:,2), 'g');
plot(time, Accelerometer(:,3), 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Acceleration (g)');
title('Accelerometer');
hold off;
axis(3) = subplot(3,1,3);
hold on;
plot(time, Magnetometer(:,1), 'r');%plot(time, xCorrected(:,1), '--r');
plot(time, Magnetometer(:,2), 'g');%plot(time, xCorrected(:,2), '--g');
plot(time, Magnetometer(:,3), 'b');%plot(time, xCorrected(:,3), '--b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Flux (G)');
title('Magnetometer');
hold off;
linkaxes(axis, 'x');
%------------------------------------------------------------------
% Compute orientation
quaternion(1,:) = [1 0 0 0];%quaternion = zeros(length(time), 4);
AHRSalgorithm = ECF_AHRS();%'SamplePeriod', 1/125,'Kp',0.001,'KpInit',200,'Ki',0.00001    ECF_AHRS

% Initial convergence
% initPeriod = 2;
% indexSel = 1 : find(sign(time-(time(1)+initPeriod))+1, 1);
% for i = 1:2000
%  AHRSalgorithm.UpdateIMU([0 0 0], [mean( Accelerometer(indexSel,1)) mean( Accelerometer(indexSel,2)) mean( Accelerometer(indexSel,3))]);%[mean(Gyroscope(indexSel,1)) mean(Gyroscope(indexSel,2)) mean(Gyroscope(indexSel,3))],
% % AHRSalgorithm.Update([0 0 0], [mean(accX(indexSel)) mean(accY(indexSel))mean(accZ(indexSel))],[mean(magX(indexSel)) mean(magY(indexSel)) mean(magZ(indexSel))]);
% end

% For all data
for t = 1:length(time)
 %if(stationary(t));  AHRSalgorithm.Kp = 0.5;  else
 %AHRSalgorithm.Kp = 0;%end
 AHRSalgorithm.Update(deg2rad([Gyroscope(t,1) Gyroscope(t,2) Gyroscope(t,3)]),[Accelerometer(t,1) Accelerometer(t,2) Accelerometer(t,3)], [xCorrected(t,1) xCorrected(t,2) xCorrected(t,3)]); %, xCorrected(t,1) xCorrected(t,2) xCorrected(t,3)
% AHRSalgorithm.UpdateIMU(deg2rad([gyrX(t) gyrY(t) gyrZ(t)]),[accX(t) accY(t) accZ(t)],[magX(t) magY(t) magZ(t)]);
 quaternion(t,:) = AHRSalgorithm.Quaternion;
 end

euler = quatern2euler(quaternConj(quaternion)) * (180/pi);	% use conjugate for sensor frame relative to Earth and convert to degrees.
%euler1= quat2eul(quaternion);euler = [phi theta psi]
 
% GYRO INTEGRATION
%Velocity/Distance: a = acceleration_vector; t = time_vector; v = cumtrapz(t, a); c = cumtrapz(t, v);
%euler1(:,1)=cumtrapz(time, Gyroscope(:,1));euler1(:,2)=cumtrapz(time, Gyroscope(:,2));euler1(:,3)=cumtrapz(time, Gyroscope(:,3));

figure('Name', 'Euler Angles');
axis(4) = subplot(2,1,1);
hold on;
plot(time, euler(:,1), 'r'); %plot(time, euler1(:,1), '--r');
plot(time, euler(:,2), 'g'); %plot(time, euler1(:,2), '--g');
plot(time, euler(:,3), 'b'); %plot(time, euler1(:,3), '--b');% plot(time, euler(:,4),'--c');plot(time, euler(:,5),'--m');

title('Euler angles');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('\phi', '\theta', '\psi');
hold off;

axis(5) = subplot(2,1,2);
hold on;
plot(time, quaternion(:,1), 'y');
plot(time, quaternion(:,2), 'r');
plot(time, quaternion(:,3), 'g'); 
plot(time, quaternion(:,4), 'b'); 

title('Quaternions');
xlabel('Time (s)');
ylabel('quaternion');
legend('q1', 'q2', 'q3','q4');
hold off;