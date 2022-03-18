function config = RRR_inverse_2D(Link_Lengths, pose_end)
config=zeros(3,2); %Initialize anser matrix
L1=Link_Lengths(1);
L2=Link_Lengths(2);
L3=Link_Lengths(3);%Link lengths
x=pose_end(1)-L3*cos(pose_end(3));
y=pose_end(2)-L3*sin(pose_end(3)); %End effector poisition
a=(x^2+y^2-L1^2-L2^2)/(2*L1*L2); %From class
theta2=atan2(sqrt(1-a^2),a);
theta1=atan2(y,x)-atan2((L2*sin(theta2)),(L1+L2*cos(theta2))); %Geometry derived by hand in hw
config(1:2,1)=[theta1 theta2]; %First solution occupies first column of answer matrix
Ntheta2=atan2((-1*sqrt(1-a^2)),a); %Using negative to find second solution
Ntheta1=atan2(y,x)-atan2((L2*sin(Ntheta2)),(L1+L2*cos(Ntheta2)));%Finding second solution for theta 1
config(1:2,2)=[Ntheta1 Ntheta2]; %Second answer occupies second column of answer matrix
config(3,1)=pose_end(3)-theta1-theta2;
config(3,2)=pose_end(3)-Ntheta2-Ntheta2;
end


