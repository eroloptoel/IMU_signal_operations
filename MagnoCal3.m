addpath('quaternion_library'); clear all;close all;clc;
load('ExampleData.mat');load('Magno4.csv');%load('RealMagno3.csv');

% Magnetometer Data----------------------------------------------------

N = 2457;rng(1);acc = zeros(N,3);av = zeros(N,3);q = randrot(N,1); % uniformly distributed random rotations
imu = imuSensor('accel-mag');%imu2 = imuSensor('accel-gyro');

[~,x] = imu(acc,av,q);%xy =Magnetometer(1:N,1:3);%A=x-y;
xy =Magno4(1:N,1:3);
%xy =(-1)*xy;

scatter3(x(:,1),x(:,2),x(:,3));axis equal;title('Ideal Magnetometer Data');figure
scatter3(xy(:,1),xy(:,2),xy(:,3));axis equal;title('Real Magnetometer Data');

%-------- Magnetometer Calibration (Hard+Soft)-----------------------------------------------------
[A,b,expMFS]  = magcal(xy);xCorrected = (xy-b)*A;

%-------- Manuel MagCal
t=xy;offset(1)=(min(t(:,1))+max(t(:,1)))/2;offset(2)=(min(t(:,2))+max(t(:,2)))/2;offset(3)=(min(t(:,3))+max(t(:,3)))/2;%offset_x_y_z

avg_delta_x=(max(t(:,1))-min(t(:,1)))/2;     avg_delta_y=(max(t(:,2))-min(t(:,2)))/2;     avg_delta_z=(max(t(:,3))-min(t(:,3)))/2;
%avg_delta=sqrt(avg_delta_x^2+avg_delta_y^2+avg_delta_z^2)/3;
avg_delta=(avg_delta_x+avg_delta_y+avg_delta_z)/3;

scale_x=avg_delta/avg_delta_x;  scale_y=avg_delta/avg_delta_y;  scale_z=avg_delta/avg_delta_z;%scale_x=avg_delta/avg_delta_x*.65;scale_y=avg_delta/avg_delta_y*.9;scale_z=avg_delta/avg_delta_z*.7;
mxy=(scale_x-scale_y);myz=(scale_y-scale_z)*1;mxz=scale_x-scale_z;%avg_scale=(scale_x+scale_y+scale_z)/3;
A2=[scale_x mxy mxz;mxy scale_y myz;mxz myz scale_z];

% if avg_delta<=1 
%     mxy=0;myz=0;mxz=0;
% else mxy=(scale_x-scale_y)/2;myz=(scale_z-scale_y)/8;mxz=scale_z-scale_x;%avg_scale=(scale_x+scale_y+scale_z)/3;
% end
% A2=[scale_x mxy mxz;mxy scale_y myz;mxz myz scale_z];
corrected_x2 = (t-offset)*A2;

%-------- Plot+Metric
figure;scatter3(xCorrected(:,1),xCorrected(:,2),xCorrected(:,3));axis equal;title('Magnetometer Data Calibrated');
r = sum(xCorrected.^2,2) - expMFS.^2;E = sqrt(r.'*r./N)./(2*expMFS.^2);
fprintf('Residual error in corrected data : %.2f\n\n',E);

figure;scatter3(corrected_x2(:,1),corrected_x2(:,2),corrected_x2(:,3));axis equal;title('Magnetometer Data Calibrated2');
r2 = sum(corrected_x2.^2,2) - expMFS.^2;E2 = sqrt(r2.'*r2./N)./(2*expMFS.^2);
fprintf('Residual error in corrected data2 : %.2f\n\n',E2);%r2=sqrt(sum(xCorrected-corrected_x.^2));