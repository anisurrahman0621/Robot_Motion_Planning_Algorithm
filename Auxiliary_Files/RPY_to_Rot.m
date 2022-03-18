function R = RPY_to_Rot(RPY_angles)
a=RPY_angles(1);
b=RPY_angles(2);
c=RPY_angles(3);
R1=[1 0 0;0 cos(a) -sin(a);0 sin(a) cos(a)];
R2=[cos(b) 0 sin(b);0 1 0;-sin(b) 0 cos(b)];
R3=[cos(c) -sin(c) 0;sin(c) cos(c) 0;0 0 1];
R=R3*R2*R1;
end