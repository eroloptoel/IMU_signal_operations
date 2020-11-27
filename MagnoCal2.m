addpath('quaternion_library'); clear all;close all;clc;
%load('ExampleData.mat');%load('RealMagno3.csv');
%-------Parameters----------------------------------------------------------------------

N = 8700;rng(1);acc = zeros(N,3);av = zeros(N,3);q = randrot(N,1); % uniformly distributed random rotations

%-------Ideal Magnetometer Data---------------------------------------------------------

imu = imuSensor('accel-mag');%imu2 = imuSensor('accel-gyro');
[~,x] = imu(acc,av,q);%xy =Magnetometer(1:N,1:3);%A=x-y;%Madgwick Data
%x =RealMagno3(1:N,1:3);

scatter3(x(:,1),x(:,2),x(:,3));axis equal;title('Ideal Magnetometer Data');
%figure;scatter3(xy(:,1),xy(:,2),xy(:,3));axis equal;title('Madgwick Magnetometer Data');

%-------Magnetometer Data With a Hard Iron Offset----------------------------------------

imu.Magnetometer.ConstantBias = [2 10 40];[~,y] = imu(acc,av,q);
figure;scatter3(y(:,1),y(:,2),y(:,3));axis equal;title('Magnetometer Data With a Hard Iron Offset');

%------ Magnetometer Data With Hard and Soft Iron Effects---------------------------------

nedmf = imu.MagneticField;Rsoft = [2.5 0.3 0.5; 0.3 2 .2; 0.5 0.2 3];
soft = rotateframe(conj(q),rotateframe(q,nedmf)*Rsoft);

for ii=1:numel(q)
    imu.MagneticField = soft(ii,:);
    [~,z(ii,:)] = imu(acc(ii,:),av(ii,:),q(ii));
end

figure;scatter3(z(:,1),z(:,2),z(:,3));axis equal;title('Magnetometer Data With Hard and Soft Iron Effects');

%------+Noise----------------------------------------------------------------------------

imu.Magnetometer.NoiseDensity = 8;
for ii=1:numel(q)
    imu.MagneticField = soft(ii,:);
    [~,t(ii,:)] = imu(acc(ii,:),av(ii,:),q(ii));
end %figure;scatter3(t(:,1),t(:,2),t(:,3));axis equal;

%--------Magnetometer Calibrated (Hard and Soft Iron Effects Calibration)-------------------
t=x;
[A,b,expMFS]  = magcal(t);xCorrected = (t-b)*A;%r=z-xCorrected;%İdeal magcal calibration
%[Adiag,bdiag,expMFSdiag] = magcal(t,'diag');xDiagCorrected = (x-bdiag)*Adiag;%İdeal magcal-diag calibration
%[Axy,bxy,expMFSxy]  = magcal(xy,'diag');xCorrected2 = (xy-bxy)*Axy;%r=z-xCorrected;%Madgwick magcal-diag calibration

%---------Magnetometer Calibrated2(Hard)-----------------------------------------------------t=xy;

offset(1)=(min(t(:,1))+max(t(:,1)))/2;offset(2)=(min(t(:,2))+max(t(:,2)))/2;offset(3)=(min(t(:,3))+max(t(:,3)))/2;%offset_x_y_z
%xcal(:,1)=t(:,1)-offset(1);xcal(:,2)=t(:,2)-offset(2);xcal(:,3)=t(:,3)-offset(3);%xcal=t-offset;xcal=xcal*eye(3)*0.8;%rand(3,3);distance = sqrt((x-xCenter).^2+(y-yCenter).^2+(z-zCenter).^2);
%figure;scatter3(xcal(:,1),xcal(:,2),xcal(:,3));axis equal;

%-------- Magnetometer Calibrated2(Soft)------------------------------------------------------

avg_delta_x=(max(t(:,1))-min(t(:,1)))/2;     avg_delta_y=(max(t(:,2))-min(t(:,2)))/2;     avg_delta_z=(max(t(:,3))-min(t(:,3)))/2;
avg_delta=(avg_delta_x+avg_delta_y+avg_delta_z)/3;
scale_x=avg_delta/avg_delta_x;  scale_y=avg_delta/avg_delta_y;  scale_z=avg_delta/avg_delta_z;%scale_x=avg_delta/avg_delta_x*.65;scale_y=avg_delta/avg_delta_y*.9;scale_z=avg_delta/avg_delta_z*.7;
mxy=(scale_x-scale_y)/2;myz=(scale_z-scale_y)/8;mxz=scale_z-scale_x;%avg_scale=(scale_x+scale_y+scale_z)/3;
A2=[scale_x mxy mxz;mxy scale_y myz;mxz myz scale_z];
% m12=(max(t(:,1)-min(t(:,2)))/2);m13=(max(t(:,1)-min(t(:,3)))/2);m21=(max(t(:,2)-min(t(:,1)))/2);m23=(max(t(:,2)-min(t(:,3)))/2);m31=(max(t(:,3)-min(t(:,1)))/2);m32=(max(t(:,3)-min(t(:,2)))/2);
%corrected_x=(xcal(:,1))*scale_x;corrected_y =(xcal(:,2))*scale_y;corrected_z =(xcal(:,3))*scale_z;%corrected_x=(xcal(2,:)*A);
corrected_x2 = (t-offset)*A2;

%---------Magnetometer Calibrated Data Plot----------------------------------------------

figure;scatter3(xCorrected(:,1),xCorrected(:,2),xCorrected(:,3));axis equal;title('Magnetometer Data Calibrated');
%figure;scatter3(xDiagCorrected(:,1),xDiagCorrected(:,2),xDiagCorrected(:,3));axis equal;title('Magnetometer Data Calibrated2');
%figure;scatter3(xCorrected2(:,1),xCorrected2(:,2),xCorrected2(:,3));axis equal;title('Madgwick Magnetometer Data Calibrated');%figure;qw1=x-xCorrected;scatter3(qw1(:,1),qw1(:,2),qw1(:,3));axis equal;title('Magnetometer Ideal Data and Calibrated Data difference');
% figure;scatter3(xcal(:,1),xcal(:,2),xcal(:,3));axis equal;title('Data Calibration');%figure;scatter3(corrected_x,corrected_y,corrected_z);axis equal;
figure;scatter3(corrected_x2(:,1),corrected_x2(:,2),corrected_x2(:,3));axis equal;title('Magnetometer Data Calibrated2');

%plotmatrix(xCorrected);figure;plotmatrix(z);ellipsoid(A,b,expMFS,t,xCorrected,'Auto');% de = HelperDrawEllipsoid;de.plotCalibrated(A,b,expMFS,x,xCorrected,'Auto');
% [Aeye,beye,expMFSeye] = magcal(x,'eye');xEyeCorrected = (x-beye)*Aeye;% [ax1,ax2] = de.plotCalibrated(Aeye,beye,expMFSeye,x,xEyeCorrected,'Eye');% view(ax1,[-1 0 0]);view(ax2,[-1 0 0]);

% figure;qw2=x-xDiagCorrected;scatter3(qw2(:,1),qw2(:,2),qw2(:,3));axis equal;title('Magnetometer İdeal Data and Calibrated2 Data difference');
% figure;qw=qw1-qw2;;scatter3(qw(:,1),qw(:,2),qw(:,3));axis equal;title('Magnetometer Data Calibrated and Data Calibrated2 difference');

%-----------Magnetometer Calibration Residual Error----------------------------------------

r = sum(xCorrected.^2,2) - expMFS.^2;E = sqrt(r.'*r./N)./(2*expMFS.^2);
fprintf('Residual error in corrected data : %.2f\n\n',E);%r2=sqrt(sum(xCorrected-corrected_x.^2));
r2 = sum(corrected_x2.^2,2) - expMFS.^2;E2 = sqrt(r2.'*r2./N)./(2*expMFS.^2);
fprintf('Residual error in corrected data2 : %.2f\n\n',E2);%r2=sqrt(sum(xCorrected-corrected_x.^2));

% close all;for i=1:N;yaw1(i)=atan(xcal(i,2)/xcal(i,1));yaw2(i)=atan(xCorrected(i,2)/xCorrected(i,1)); end;plot (yaw1);figure;plot (yaw2);
% close all;for i=1:N;rol1(i)=atan(xcal(i,2)/xcal(i,3));roll2(i)=atan(xCorrected(i,2)/xCorrected(i,3)); end;plot (yaw1);figure;plot (yaw2);