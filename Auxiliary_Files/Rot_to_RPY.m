function RPY_angles = Rot_to_RPY(R)
Cb=sqrt(R(1,1)^2+R(2,1)^2);
Sb=-R(3,1)
b=atan2(Sb,Cb);
%a=atan2(R(2,1)/Cb,R(1,1)/Cb);
%c=atan2(R(3,2)/Cb,R(3,3)/Cb);
Ca=R(1,1)/cos(b);
Sa=R(2,1)/cos(b);
a=atan2(Sa,Ca);
Sc=R(3,2)/cos(b);
Cc=R(3,3)/cos(b);
c=atan2(Sc,Cc);
RPY_angles=[c b a]';
end

