function J_B=BodymanipJac(gst0,axis_joints, q_joints, type_joints, theta)
%In A
B=size(axis_joints);
N=B(2);
for i=1:N
    w(1:3,i)=axis_joints(1:3,i);
    q(1:3,i)=q_joints(1:3,i);
end
for i=1:N
    Z=[0 0 0 1];
    W=w(:,i);
    if type_joints(i)=='revolute'
        R=AxisAngle_to_Rot(W,theta(i));
        Q=q(:,i);
        B=(eye(3)-R)*Q;
        g(:,:,i)=[R B;Z];
        psi(1:6,i)=[cross(-W,Q);Q];
    else
        g(:,:,i)=[eye(3) W*theta(i);Z];
        psi(1:6,i)=[W*theta(i);zeros(3,1)];
    end
end
QQ=eye(4);
for i=1:N
    QQ=QQ*g(:,:,i);
    G(:,:,i)=QQ;
end
for i=1:N
     GG=(G(:,:,i))*gst0;
    RR=GG(1:3,1:3);
    pp=GG(1:3,4);
    pp_hat=skew_symmetric(pp);
    Adj_inv=[RR' -RR'*pp_hat;zeros(3) RR'];
    psi_cross(1:6,i)=Adj_inv*psi(1:6,i);
    J_B(1:6,i)=psi_cross(1:6,i);
end
end

