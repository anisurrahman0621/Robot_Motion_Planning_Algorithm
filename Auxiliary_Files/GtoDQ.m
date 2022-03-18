function A=GtoDQ(g)

R=g(1:3,1:3); %extract R and p
p=g(1:3,4);

Ar=[Rot_to_Quat(R)]'; %convert to quaterion representation

pq=[0;p];
Ad=0.5*QuatMultiply(pq,Ar);  %convert to quaterion representation

A=[Ar Ad]; %DQ
end
