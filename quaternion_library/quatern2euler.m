function euler = quatern2euler(q)
%QUATERN2EULER Converts a quaternion orientation to ZYX Euler angles
%
%   q = quatern2euler(q)
%
%   Converts a quaternion orientation to ZYX Euler angles where phi is a
%   rotation around X, theta around Y and psi around Z.
%
%   For more information see:
%   http://www.x-io.co.uk/node/8#quaternions
%
%	Date          Author          Notes
%	27/09/2011    SOH Madgwick    Initial release
% 
    R(1,1,:) = 2.*q(:,1).^2-1+2.*q(:,2).^2;
    R(2,1,:) = 2.*(q(:,2).*q(:,3)-q(:,1).*q(:,4));
    R(3,1,:) = 2.*(q(:,2).*q(:,4)+q(:,1).*q(:,3));
    R(3,2,:) = 2.*(q(:,3).*q(:,4)-q(:,1).*q(:,2));
    R(3,3,:) = 2.*q(:,1).^2-1+2.*q(:,4).^2;
      
    phi = atan2(R(3,2,:), R(3,3,:) );
    theta = -atan(R(3,1,:) ./ sqrt(1-R(3,1,:).^2) );
    psi = atan2(R(2,1,:), R(1,1,:) );
          
%      if theta>=(pi*0.4444)&theta<=(pi*0.5) 
%         roll=0;yaw=-2*(atan2(q(:,2),q(:,0)));end
%      if theta>=(-1*pi*0.5)&theta<=(-1*pi*0.4444) 
%         roll=0;yaw=2*(atan2(q(:,2),q(:,0)));end
    
%  phi = asin(R(3,2,:)./sqrt(1+R(3,2,:).^2));theta = -asin(R(3,1,:)./sqrt(1+R(3,1,:).^2))psi = asin(R(2,1,:)./sqrt(1+R(2,1,:).^2));;      
          
 euler = [phi(1,:)' theta(1,:)' psi(1,:)'];
    
end

