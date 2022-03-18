function R = Quat_to_Rot(Q)
qo=Q(1);
q=Q(2:4);
angle=2*acos(qo);
axis=q/sin(0.5*angle);
R=AxisAngle_to_Rot(axis, angle);
end