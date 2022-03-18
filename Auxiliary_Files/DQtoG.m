function g=DQtoG(A);

Ar=A(1:4,1); %Seperate DQ to Quaternioin
Ad=A(1:4,2);


R=Quat_to_Rot(Ar); %extract R matrix

p=2*QuatMultiply(Ad,QuatInverse(Ar)); %extract p vector

g=[R p(2:4);0 0 0 1];
end