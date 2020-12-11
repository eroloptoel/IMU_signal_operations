function angle = getAngle(v1,v2)
 a = dot(v1,v2);
 b = norm(v1) * norm(v2);


% angle = acos(a/b);
% if angle > pi/2
% angle = pi-angle;
% end


 temp = cross(v1,v2);
 if temp(3)>0
 angle = acos(a/b);
 else
 angle = -acos(a/b);
 end
end
