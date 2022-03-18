function Adj=AdjointOfg(g)
R=g(1:3,1:3);
p=g(1:3,4);
zero=zeros(3);
p_hat=skew_symmetric(p);
Adj=[R p_hat*R;zero R];
end

