
addpath('quaternion_library'); clear all;close all;clc;
load('ExampleData.mat');load('Magno4.csv');%load('RealMagno3.csv');%load('ExampleData.mat');load('RealMagno3.csv');
N = 2457;rng(1);acc = zeros(N,3);av = zeros(N,3);
q = randrot(N,1); % uniformly distributed random rotations

% imu = imuSensor('accel-mag');%imu2 = imuSensor('accel-gyro');
%  [~,xyz] = imu(acc,av,q);
 xyz =Magno4(1:N,1:3);
scatter3(xyz(:,1),xyz(:,2),xyz(:,3));axis equal;title('Real Magnetometer Data');

% %Magnetometer Data (Hard and Soft Iron Effects)--------------------------
% imu.Magnetometer.ConstantBias = [2 10 40];imu.Magnetometer.NoiseDensity = 0.08;
% 
% nedmf = imu.MagneticField;Rsoft = [2.5 0.3 0.5; 0.3 2 .2; 0.5 0.2 3];
% soft = rotateframe(conj(q),rotateframe(q,nedmf)*Rsoft);
% 
% for ii=1:numel(q)
%     imu.MagneticField = soft(ii,:);
%     [~,xyz(ii,:)] = imu(acc(ii,:),av(ii,:),q(ii));
% end

X=xyz(:,1); Y=xyz(:,2); Z=xyz(:,3);

%Magnetometer Calibrated (Hard and Soft Iron Effects Cal)----------------------

[ofs,gain,rotM]=ellipsoid_fit(X,Y,Z);
[gain,rotM]=refine_3D_fit(gain,rotM); % optional refinement
XC=X-ofs(1); YC=Y-ofs(2); ZC=Z-ofs(3); % translate to (0,0,0)
XYZC=[XC,YC,ZC]*rotM; % rotate to XYZ axes

refr = 50; % reference radius    refr = 80;********************
XC=XYZC(:,1)/gain(1)*refr;
YC=XYZC(:,2)/gain(2)*refr;
ZC=XYZC(:,3)/gain(3)*refr; % scale to sphere

figure;
subplot(2,2,1); hold on; plot(XC,YC,'ro'); plot(X,Y,'kx');
xlabel('X'); ylabel('Y'); axis equal; grid on;
subplot(2,2,2); hold on; plot(ZC,YC,'go'); plot(Z,Y,'kx');
xlabel('Z'); ylabel('Y'); axis equal; grid on;
subplot(2,2,3); hold on; plot(XC,ZC,'bo'); plot(X,Z,'kx');
xlabel('X'); ylabel('Z'); axis equal; grid on;

[A,b,expMFS]  = magcal(xyz);xCorrected = (xyz-b)*A;
%xCorrected=[XC YC ZC];
figure;scatter3(xCorrected(:,1),xCorrected(:,2),xCorrected(:,3));axis equal;title('Magnetometer Data Magcal Calibrated');
r = sum(xCorrected.^2,2) - expMFS.^2;
E = sqrt(r.'*r./N)./(2*expMFS.^2);
fprintf('Residual error in corrected data : %.2f\n\n',E);

%Functions------------------------------------------------------------------
function [gain,rotM]=refine_3D_fit(gain,rotM)
 % largest element should be on diagonal
 m=0; rm=0; cm=0;
 for r=1:3, for c=1:3,
 if abs(rotM(r,c))>m, m=abs(rotM(r,c)); rm=r; cm=c; end; % record max
 end; end;
 if rm~=cm, % swap cols if not on diagonal
 t=rotM(:,cm); rotM(:,cm)=rotM(:,rm); rotM(:,rm)=t;
 t=gain(cm); gain(cm)=gain(rm); gain(rm)=t;
 end; % largest now in the diagonal, in row rm

 % do the same on remaining 2x2 matrix
 switch rm, case 1, i=[2 3]; case 2, i=[1 3]; case 3, i=[1 2]; end;
 m=0; rm=0; cm=0;
 for r=1:2, for c=1:2,
 if abs(rotM(i(r),i(c)))>m, m=abs(rotM(i(r),i(c))); rm=i(r); cm=i(c); end;
 end; end;
 if rm~=cm, % swap cols if not on diagonal
 t=rotM(:,cm); rotM(:,cm)=rotM(:,rm); rotM(:,rm)=t;
 t=gain(cm); gain(cm)=gain(rm); gain(rm)=t;
 end;

 % neg cols to make it positive along diagonal
 if rotM(1,1)<0, rotM(:,1)=-rotM(:,1); end;
if rotM(2,2)<0, rotM(:,2)=-rotM(:,2); end;
 if rotM(3,3)<0, rotM(:,3)=-rotM(:,3); end;
end

function [ofs,gain,rotM]=ellipsoid_fit(x,y,z)
% Fit a rotated ellipsoid to a set of xyz data points
% XYZ: N(rows) x 3(cols), matrix of N data points (x,y,z)
% x=xyz(:,1); y=xyz(:,2); z=xyz(:,3);
x2=x.*x; y2=y.*y; z2=z.*z;
D = [x2+y2-2*z2, x2-2*y2+z2, 4*x.*y, 2*x.*z, 2*y.*z, 2*x, 2*y, 2*z, ones(length(x),1)];
R = x2+y2+z2;
b = (D'*D)\(D'*R);%b=floor(b); % least square solution
[row, col] = find(isnan(b));
for i=1:100
    b(row, col)=0;
end
mtxref = [ 3 1 1 0 0 0 0 0 0 0; 3 1 -2 0 0 0 0 0 0 0; 3 -2 1 0 0 0 0 0 0 0; ...
 0 0 0 2 0 0 0 0 0 0; 0 0 0 0 1 0 0 0 0 0; 0 0 0 0 0 1 0 0 0 0; ...
 0 0 0 0 0 0 1 0 0 0; 0 0 0 0 0 0 0 1 0 0; 0 0 0 0 0 0 0 0 1 0; ...
 0 0 0 0 0 0 0 0 0 1];
v = mtxref*[-1/3; b]; nn=v(10); v = -v(1:9);
A = [ v(1) v(4) v(5) v(7); v(4) v(2) v(6) v(8); v(5) v(6) v(3) v(9); v(7) v(8) v(9) -nn ];
ofs=-A(1:3,1:3)\[v(7);v(8);v(9)]; % offset is center of ellipsoid
Tmtx=eye(4); Tmtx(4,1:3)=ofs'; AT=Tmtx*A*Tmtx'; % ellipsoid translated to (0,0,0)
[rotM ev]=eig(AT(1:3,1:3)/-AT(4,4)); % eigenvectors (rotation) and eigenvalues (gain)
% if (isnan(ev(:,:))==1 || isnan(rotM(:,:))==1 || isnan(D==1)||isinf(D==1))% return;end ;%TF2 = isnan(ev);ev(TF2) = 0.1;TF3 = isinf(ev);ev(TF3) = 1;TF4 = isnan(rotM);rotM(TF4) = 0.1;TF5 = isinf(rotM);rotM(TF5) = 1;
gain=sqrt(1./diag(ev)); % gain is radius of the ellipsoid
end

% function [ofs,gain,rotM]=ellipsoid_fit(XYZ,varargin)
% % Fit an (non)rotated ellipsoid or sphere to a set of xyz data points
% % XYZ: N(rows) x 3(cols), matrix of N data points (x,y,z)
% % optional flag f, default to 0 (fitting of rotated ellipsoid)
% x=XYZ(:,1); y=XYZ(:,2); z=XYZ(:,3); if nargin>1, f=varargin{1}; else f=0; end;
% if f==0, D=[x.*x, y.*y, z.*z, 2*x.*y,2*x.*z,2*y.*z, 2*x,2*y,2*z]; % any axes (rotated ellipsoid)
% elseif f==1, D=[x.*x, y.*y, z.*z, 2*x,2*y,2*z]; % XYZ axes (non-rotated ellipsoid)
% elseif f==2, D=[x.*x+y.*y, z.*z, 2*x,2*y,2*z]; % and radius x=y
% elseif f==3, D=[x.*x+z.*z, y.*y, 2*x,2*y,2*z]; % and radius x=z
% elseif f==4, D=[y.*y+z.*z, x.*x, 2*x,2*y,2*z]; % and radius y=z
% elseif f==5, D=[x.*x+y.*y+z.*z, 2*x,2*y,2*z]; % and radius x=y=z (sphere)
% end;
% v = (D'*D)\(D'*ones(length(x),1)); % least square fitting
% if f==0, % rotated ellipsoid
%  A = [ v(1) v(4) v(5) v(7); v(4) v(2) v(6) v(8); v(5) v(6) v(3) v(9); v(7) v(8) v(9) -1 ];
%  ofs=-A(1:3,1:3)\[v(7);v(8);v(9)]; % offset is center of ellipsoid
%  Tmtx=eye(4); Tmtx(4,1:3)=ofs'; AT=Tmtx*A*Tmtx'; % ellipsoid translated to (0,0,0)
%  [rotM ev]=eig(AT(1:3,1:3)/-AT(4,4)); % eigenvectors (rotation) and eigenvalues (gain)
%  gain=sqrt(1./diag(ev)); % gain is radius of the ellipsoid
% else % non-rotated ellipsoid
%  if f==1, v = [ v(1) v(2) v(3) 0 0 0 v(4) v(5) v(6) ];
%  elseif f==2, v = [ v(1) v(1) v(2) 0 0 0 v(3) v(4) v(5) ];
%  elseif f==3, v = [ v(1) v(2) v(1) 0 0 0 v(3) v(4) v(5) ];
%  elseif f==4, v = [ v(2) v(1) v(1) 0 0 0 v(3) v(4) v(5) ];
%  elseif f==5, v = [ v(1) v(1) v(1) 0 0 0 v(2) v(3) v(4) ]; % sphere
%  end;
%  ofs=-(v(1:3).\v(7:9))'; % offset is center of ellipsoid
%  rotM=eye(3); % eigenvectors (rotation), identity = no rotation
%  g=1+(v(7)^2/v(1)+v(8)^2/v(2)+v(9)^2/v(3));
%  gain=(sqrt(g./v(1:3)))'; % find radii of the ellipsoid (scale)
% end