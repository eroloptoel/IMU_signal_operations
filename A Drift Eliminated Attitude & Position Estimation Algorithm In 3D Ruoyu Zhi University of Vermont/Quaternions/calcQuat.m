function quat = calcQuat(v1,v2)
 a = cross(v1,v2);
 b = acos(dot(v1,v2)/(norm(v1)*norm(v2)));
 quat = [cos(b/2) a];
 k = sqrt((quat(2)^2 + quat(3)^2 + quat(4)^2)/(1-quat(1)^2));
 quat = [cos(b/2) a/k];
% b = sqrt((norm(v1))^2 + (norm(v2))^2) + dot(v1,v2);
% quat = [b a];
% quat = quat/norm(quat);
end