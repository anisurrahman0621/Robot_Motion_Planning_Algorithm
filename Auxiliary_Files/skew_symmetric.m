function A=skew_symmetric(p)
p1=p(1);
p2=p(2);
p3=p(3);
A=[0 -p3 p2;p3 0 -p1;-p2 p1 0];
end