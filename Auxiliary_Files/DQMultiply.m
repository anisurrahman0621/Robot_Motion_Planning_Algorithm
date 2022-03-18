function C=DQMultiply(A,B)

P=A(:,1);%Seperate DQs into repsective quaternions
Q=A(:,2);
U=B(:,1);
V=B(:,2);

C=[QuatMultiply(P,U),QuatMultiply(Q,U)+QuatMultiply(P,V)];
end
