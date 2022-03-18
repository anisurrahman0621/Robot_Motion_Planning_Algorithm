function T=QuatMultiply(Q,P)

q0=Q(1); % scalar
p0=P(1);

q=Q(2:4); %vector
p=P(2:4);

T=[q0*p0-dot(q,p);q0*p+p0*q+cross(q,p)];
end