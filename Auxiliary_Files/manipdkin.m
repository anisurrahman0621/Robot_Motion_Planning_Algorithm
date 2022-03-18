function [g_base_tool] = manipdkin(gst0, axis_joints, q_joints, type_joints, theta)
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
    else
        g(:,:,i)=[eye(3) W*theta(i);Z];
    end
end
QQ=eye(4);
for i=1:N
    QQ=QQ*g(:,:,i);
end
g_base_tool=QQ*gst0;
end
