function T=BaxterFK(theta)

w1r = [0;0;1];
w2r = [-1/sqrt(2);1/sqrt(2);0];
w3r = [1/sqrt(2);1/sqrt(2);0];
w4r = w2r;
w5r = w3r;
w6r = w2r;
w7r = w3r;


l0=270.35/1000;
l1=69/1000;
l2=364.35/1000;
l3=69/1000;
l4=374.29/1000;
l5=10/1000;
l6=229.53/1000;

q1=[0,0,l0]';
q2=[l1/sqrt(2),l1/sqrt(2),l0]';
q3=q2;
q4=[(l1+l2)/sqrt(2),(l1+l2)/sqrt(2),l0-l3]';
q5=q4;
q6=[(l1+l2+l4)/sqrt(2),(l1+l2+l4)/sqrt(2),l0-l3-l5]';
q7=q6;

gst0=[eye(3) [(l1+l2+l4+l6)/sqrt(2) (l1+l2+l4+l6)/sqrt(2) l0-l3-l5]';0 0 0 1];

axis_joints=[w1r w2r w3r w4r w5r w6r w7r];
q_joints=[q1 q2 q3 q4 q5 q6 q7];
for i=1:7
type_joints(i,1)="revolute";
end

T=manipdkin(gst0, axis_joints, q_joints, type_joints, theta);
end
