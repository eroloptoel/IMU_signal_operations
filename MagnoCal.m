addpath('quaternion_library'); clear all;close all;clc;
load('ExampleData.mat');
%Ideal Magnetometer Data----------------------------------------------------

N = 10000;rng(1);
acc = zeros(N,3);av = zeros(N,3);
q = randrot(N,1); % uniformly distributed random rotations
imu = imuSensor('accel-mag');%imu2 = imuSensor('accel-gyro');

[~,x] = imu(acc,av,q);
%xy =Magnetometer(1:N,1:3);%A=x-y;xy =(-1)*xy;xy =mag(1:N,1:3);

scatter3(x(:,1),x(:,2),x(:,3));axis equal;title('Ideal Magnetometer Data');%figure;scatter3(xy(:,1),xy(:,2),xy(:,3));axis equal;title('Madgwick Magnetometer Data');

%Magnetometer Data With a Hard Iron Offset----------------------------------

imu.Magnetometer.ConstantBias = [2 10 40];[~,y] = imu(acc,av,q);
figure;scatter3(y(:,1),y(:,2),y(:,3));axis equal;title('Magnetometer Data With a Hard Iron Offset');

%Magnetometer Data With Hard and Soft Iron Effects--------------------------

nedmf = imu.MagneticField;Rsoft = [2.5 0.3 0.5; 0.3 2 .2; 0.5 0.2 3];
soft = rotateframe(conj(q),rotateframe(q,nedmf)*Rsoft);

for ii=1:numel(q)
    imu.MagneticField = soft(ii,:);
    [~,z(ii,:)] = imu(acc(ii,:),av(ii,:),q(ii));
end

figure;scatter3(z(:,1),z(:,2),z(:,3));axis equal;title('Magnetometer Data With Hard and Soft Iron Effects');

%+ Noise ---------------------------------------------------------------------

imu.Magnetometer.NoiseDensity = 0.08;
for ii=1:numel(q)
    imu.MagneticField = soft(ii,:);
    [~,t(ii,:)] = imu(acc(ii,:),av(ii,:),q(ii));
end

%Magnetometer Calibrated (Hard and Soft Iron Effects Cal)----------------------

[A,b,expMFS]  = magcal(t);xCorrected = (t-b)*A;%r=z-xCorrected;
% [Axy,bxy,expMFSxy]  = magcal(t,'eye');xCorrected2 = (t-bxy)*Axy;%r=z-xCorrected;
% [Adiag,bdiag,expMFSdiag] = magcal(t,'diag');xDiagCorrected = (t-bdiag)*Adiag;

%Magnetometer Calibrated Plot---------------------------------------------------

figure;scatter3(xCorrected(:,1),xCorrected(:,2),xCorrected(:,3));axis equal;title('Magnetometer Data Calibrated');

% figure;scatter3(xCorrected2(:,1),xCorrected2(:,2),xCorrected2(:,3));axis equal;title(' Magnetometer Data Calibrated2');
% % figure;qw1=x-xCorrected;scatter3(qw1(:,1),qw1(:,2),qw1(:,3));axis equal;title('Magnetometer Ideal Data and Calibrated Data difference');
% figure;scatter3(xDiagCorrected(:,1),xDiagCorrected(:,2),xDiagCorrected(:,3));axis equal;title('Magnetometer Data Calibrated3');
% figure;qw2=x-xDiagCorrected;scatter3(qw2(:,1),qw2(:,2),qw2(:,3));axis equal;title('Magnetometer Ýdeal Data and Calibrated3 Data difference');
% %qw=qw1-qw2;;scatter3(qw(:,1),qw(:,2),qw(:,3));axis equal;title('Magnetometer Data Calibrated and Data Calibrated2 difference');

%Magnetometer Calibration Residual error
r = sum(xCorrected.^2,2) - expMFS.^2;
E = sqrt(r.'*r./N)./(2*expMFS.^2);
fprintf('Residual error in corrected data : %.2f\n\n',E);