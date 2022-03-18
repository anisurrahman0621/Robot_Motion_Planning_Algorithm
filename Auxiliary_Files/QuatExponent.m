function A_tau=QuatExponent(A,tau)

Ar=A(:,1);
Ad=A(:,2);

R=Quat_to_Rot(Ar);
H=Rot_to_AxisAngle(R);

uq=[0;H(:,1)];
theta=H(1,2);

pq(1:4,1)=2*QuatMultiply(Ad,QuatInverse(Ar));
d=dot(pq,uq);

u(1:3,1)=uq(2:4);
p(1:3,1)=pq(2:4);

m=0.5*(cross(p,u)+(p-d*u)*cot(theta/2));

A_tau(1:4,1)=[cos(tau*theta/2);u*sin(tau*theta/2)];
A_tau(1:4,2)=[-tau*d/2*sin(tau*theta/2);u*tau*d/2*cos(tau*theta/2)+m*sin(tau*theta/2)];

end




