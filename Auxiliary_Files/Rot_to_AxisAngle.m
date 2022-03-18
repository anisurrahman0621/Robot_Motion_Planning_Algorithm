function Rot = Rot_to_AxisAngle(R)
ang=acos((trace(R)-1)/2);
theta(1:3,1)=ang;
ax=[R(3,2)-R(2,3);R(1,3)-R(3,1);R(2,1)-R(1,2)]*1/(2*sin(ang));
Rot=[ax, theta];

%[axis, angle] was giving me an error so I changed the output to Rot which
%works fine
end

