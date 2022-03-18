function J = SpatialmanipJac(axis_joints, q_joints, type_joints, theta)
n = length(axis_joints);
e_znew = zeros(4);
e_zold = zeros(4);
g = zeros(4);
J = zeros(6,n);
for i = 1:n
    zeta = [-cross(axis_joints(:,i),q_joints(:,i));axis_joints(:,i)];
    e_zold = e_znew;
    type = type_joints(i);
    A = type == "revolute";
    if A == 1
        e_w = AxisAngle_to_Rot(axis_joints(:,i),theta(i));
        Imeq = (eye(3)-e_w)*q_joints(:,i);
        e_znew = [e_w Imeq;zeros(1,3) 1];
    else
        e_znew = [eye(3) axis_joints(:,i)*theta(4);zeros(1,3) 1];
    end
    if i == 2
        g = e_zold;
    else
        g = g*e_zold;
    end
    if i == 1
        zeta_p = zeta;
    else
        Adg = AdjointOfg(g);
        zeta_p = Adg*zeta;
    end
    J(:,i) = zeta_p;
end