function P_star=QuatInverse(P)

p0=P(1); %scalarn and vecto seperation
p=P(2:4);

P_star=[p0;-p];
end