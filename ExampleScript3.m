addpath('quaternion_library'); clear all;close all;clc;
load('AllData4.csv');%load('ExampleData.mat');

%Ideal Magnetometer Data----------------------------------------------------
% acc = zeros(N,3);av = zeros(N,3);rng(1);
% q = randrot(N,1); % uniformly distributed random rotations
% imu = imuSensor('accel-mag');%imu2 = imuSensor('accel-gyro');
% [~,x] = imu(acc,av,q);

N=2756;time=0:0.008:0.008*(N-1);time=time';AllData=AllData4;%xy =AllData(1:N,7:9);x=xy;%xy =Magnetometer(1:N,1:3);%A=x-y;xy =(-1)*xy;

Accelerometer(:,1)=AllData(:,1);Accelerometer(:,2)=AllData(:,2);Accelerometer(:,3)=AllData(:,3);%Accelerometer(:,1)=AllData(1:N,1);Accelerometer(:,2)=AllData(1:N,2);Accelerometer(:,3)=AllData(1:N,3);
Gyroscope(:,1)=AllData(:,4); Gyroscope(:,2)=AllData(:,5); Gyroscope(:,3)=AllData(:,6); 
Magnetometer(:,1)=AllData(:,8);Magnetometer(:,2)=AllData(:,7);Magnetometer(:,3)=AllData(:,9);%Gyroscope(1:3,1)=-0.48;Gyroscope(1:3,2)=-0.30;Gyroscope(1:3,2)=-0.06;

% scatter3(Magnetometer(:,1),Magnetometer(:,2),Magnetometer(:,3));axis equal;title('Real Magnetometer Data');%figure;scatter3(xy(:,1),xy(:,2),xy(:,3));axis equal;title('Madgwick Magnetometer Data');
% 
% %Magnetometer Calibrated (Hard and Soft Iron Effects Cal)----------------------
% 
 [A,b,expMFS]  = magcal(Magnetometer);xCorrected = (Magnetometer-b)*A;%r=z-xCorrected;
% % [Axy,bxy,expMFSxy]  = magcal(t,'eye');xCorrected2 = (t-bxy)*Axy;%r=z-xCorrected;
% % [Adiag,bdiag,expMFSdiag] = magcal(t,'diag');xDiagCorrected = (t-bdiag)*Adiag;
% 
% %Magnetometer Calibrated Plot---------------------------------------------------
% 
% figure;scatter3(xCorrected(:,1),xCorrected(:,2),xCorrected(:,3));axis equal;title('Magnetometer Data Calibrated');

%% Import and plot sensor data

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
plot(time, Magnetometer(:,1), 'r');
plot(time, Magnetometer(:,2), 'g');
plot(time, Magnetometer(:,3), 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Flux (G)');
title('Magnetometer');

% plot(time, xCorrected(:,1), '--r');
% plot(time, xCorrected(:,2), '--g');
% plot(time, xCorrected(:,3), '--b');
% legend('X', 'Y', 'Z');
% xlabel('Time (s)');
% ylabel('Flux (G)');
% title('Magnetometer');


hold off;
linkaxes(axis, 'x');


%% Process sensor data through algorithm

AHRS = MadgwickAHRS('SamplePeriod', 1/125, 'Beta', 0.001);
%AHRS = MahonyAHRS('SamplePeriod', 1/125, 'Kp', 0.001);

quaternion = zeros(length(time), 4);
for t = 1:length(time)
    AHRS.Update(Gyroscope(t,:) * (pi/180), Accelerometer(t,:), xCorrected(t,:));	% gyroscope units must be radians
    quaternion(t, :) = AHRS.Quaternion;
end

%% Plot algorithm output as Euler angles
% The first and third Euler angles in the sequence (phi and psi) become
% unreliable when the middle angles of the sequence (theta) approaches ±90
% degrees. This problem commonly referred to as Gimbal Lock.
% See: http://en.wikipedia.org/wiki/Gimbal_lock

euler = quatern2euler(quaternConj(quaternion)) * (180/pi);	% use conjugate for sensor frame relative to Earth and convert to degrees.

figure('Name', 'Euler Angles');
hold on;
plot(time, euler(:,1), 'r');
plot(time, euler(:,2), 'g');
plot(time, euler(:,3), 'b');
title('Euler angles');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('\phi', '\theta', '\psi');
hold off;

%% End of script