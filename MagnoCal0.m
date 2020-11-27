addpath('quaternion_library'); clear all;close all;clc;
% load('AllData_Update.csv');%load('ExampleData.mat');
load('Magno4.csv');
%Ideal Magnetometer Data----------------------------------------------------

N = 2700;rng(1);
% acc = zeros(N,3);av = zeros(N,3);
% q = randrot(N,1); % uniformly distributed random rotations
% imu = imuSensor('accel-mag');%imu2 = imuSensor('accel-gyro');

%[~,x] = imu(acc,av,q);
% xy =AllData_Update(1:N,7:9);x=xy;
xy =Magno4(1:N,1:3);x=xy;%A=x-y;xy =(-1)*xy;

scatter3(x(:,1),x(:,2),x(:,3));axis equal;title('Real Magnetometer Data');%figure;scatter3(xy(:,1),xy(:,2),xy(:,3));axis equal;title('Madgwick Magnetometer Data');

%Magnetometer Calibrated (Hard and Soft Iron Effects Cal)----------------------

[A,b,expMFS]  = magcal(xy);xCorrected = (xy-b)*A;%r=z-xCorrected;
% [Axy,bxy,expMFSxy]  = magcal(t,'eye');xCorrected2 = (t-bxy)*Axy;%r=z-xCorrected;
% [Adiag,bdiag,expMFSdiag] = magcal(t,'diag');xDiagCorrected = (t-bdiag)*Adiag;

%Magnetometer Calibrated Plot---------------------------------------------------

figure;scatter3(xCorrected(:,1),xCorrected(:,2),xCorrected(:,3));axis equal;title('Magnetometer Data Calibrated');

%Magnetometer Calibration Residual error
r = sum(xCorrected.^2,2) - expMFS.^2;
E = sqrt(r.'*r./N)./(2*expMFS.^2);
fprintf('Residual error in corrected data : %.2f\n\n',E);