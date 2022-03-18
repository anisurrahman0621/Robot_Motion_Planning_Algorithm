function R = AxisAngle_to_Rot(axis, angle)
wx=axis(1);
wy=axis(2);
wz=axis(3);
w_hat=[0 -wz wy;wz 0 -wx; -wy wx 0];
L=(wx^2+wy^2+wz^2)^(1/2);
R=eye(3)+w_hat*sin(L*angle)/L+(w_hat)^2*(1-cos(L*angle))/L^2;
end


