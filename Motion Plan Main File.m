clear all
clc

%Baxter parameters

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

%The handle coordinates are generated using random angles to ensure that
%the final destination is actually reachable by the robot arm

gf(:,:,1)=BaxterFK([-pi()/4 pi()/13 pi()/9 pi()/3 -pi()/2.5 -3*pi()/5 2*pi()/7]);

%Once the handle is reached, a 12cm doorhandle is turned 180 deg clockwise
%The orientation will change by 90 deg clockwise and the final position
%will be translated 12cm to the left and down relative to the door knob's
%frame

gf(:,:,2)=gf(:,:,1)*[AxisAngle_to_Rot([1 0 0],-pi()/2) [0 -0.012 -0.012]';0 0 0 1];

%After the handle is pulled, the door needs to be opened which is
%essentially rotating about the z axis by 90 degrees relative to the
%handle/door orientation. The door is 30 cm is long so x and y coordinates
%will change negatively by that amount since the rotatoin is CCW (pulling)

gf(:,:,3)=gf(:,:,2)*[AxisAngle_to_Rot([0 0 1],-pi()/2) [-0.30 -0.30 0]';0 0 0 1];

%Dual Quaternion Rep for each configuration
for i=1:3
    Df(:,:,i)=GtoDQ(gf(:,:,i));
end
%All angles initially 0
theta(1:7,1)=0;
g0=BaxterFK(theta); %generate g matrix from joint angles


D0=GtoDQ(g0); %Dual Quaternion rep of g0

for i=1:3
    p_final(1:3,i)=gf(1:3,4,i); %goal positions of the EE
    q_final(1:4,i)=Df(1:4,1,i);
end



C=1;
k=1;
beta=1;

tic
for i=1:3
    E(k)=1; %k will be looped throughout three goals so E(k) will have to be reset
    %above the logic condition when the next goal is intially being worked towards
    while E>0.0001 %Error tolerance
        Di=QuatExponent(DQMultiply(DQInverse(D0),Df(:,:,i)),0.01);
        DI=DQMultiply(D0,Di); %interpolation step
        
        
        g_now=DQtoG(D0); %generate gamma for interpolation step
        p_now(1:3,1)=g_now(1:3,4);
        q_now(1:4,1)=D0(:,1);
        gamma_now=[p_now;q_now];
        
        x(k)=p_now(1); %will be used to plot EE path later
        y(k)=p_now(2);
        z(k)=p_now(3);
        
        g_interp=DQtoG(DI); %G matrix from interpolation step
        q_next(1:4,1)=DI(:,1); %generate gamma for interpolation point
        p_next(1:3,1)=g_interp(1:3,4);
        gamma_next=[p_next;q_next];
        
        q0=q_now(1); %Used for J1 matrix
        q1=q_now(2);
        q2=q_now(3);
        q3=q_now(4);
        
        J1=[-q1 q0 q3 -q2;-q2 -q3 q0 q1;-q3 q2 -q1 q0];
        J2=[eye(3) 2*skew_symmetric(p_now)*J1;zeros(3) 2*J1];
        Js=SpatialmanipJac(axis_joints,q_joints,type_joints,theta(:,k));
        B=Js'*inv(Js*Js')*J2;
        
        diff=gamma_next-gamma_now; %direction of change for theta
        
        if abs(max(diff))>0.01745/2 %linearization won't hold for steps too large so 1/2 degree is used
            beta=0.0175/2/abs(max(diff)); %this insures that the max step will always be less than 1/2 degree
        else beta=1; %if the step is smaller that is fine
        end
        theta(1:7,k+1)=theta(:,k)+beta*B*diff; %next theta values
        
        g_test=BaxterFK(theta(1:7,k+1)); %find the actual position from FK
        p_test(1:3,1)=g_test(1:3,4);
        q_test(1:4,1)=Rot_to_Quat(g_test(1:3,1:3));
        Rorientation(1:3,1:3,k)=g_test(1:3,1:3);
        
        distance=norm(p_test-p_final(:,i)); %error in position
        qq=min([norm(q_test-q_final(:,i)) norm(q_test+q_final(:,i))]); %error in rotation
        E(k+1)=max([distance qq]); %max of the two errors is used
        
        D0=GtoDQ(g_test); %D0 loops to next value
        k=k+1; %advances indices
    end
    step(i)=k-1;
end
toc

%The large block of commented code is used to animate the path. This takes
%very long due to the large number of points. A video file of the path is
%provided in the submission .zip file. 

% view(-30,-23)
% xfin=p_final(1,:);
% yfin=p_final(2,:);
% zfin=p_final(3,:);
% for i=1:3
%     scatter3(xfin(i),yfin(i),zfin(i),'g');
%     hold on
% end
%Animate tool path
% curve=animatedline('LineWidth',2);
% set(gca,'XLim',[min(x) max(x)],'YLim',[min(y) max(y)],'Zlim',[min(z) max(z)])
% hold on
% for i=1:length(x)
%     addpoints(curve,x(i),y(i),z(i));
%     head=scatter3(x(i),y(i),z(i),'filled','MarkerFaceColor','r','MarkerEdgeColor','r');
%     drawnow
%     xlabel('x (m)')
%     ylabel('y (m)')
%     zlabel('z (m)')
%     title('End Effector Path')
%     delete(head)
% end

%Plot theta values
for i=1:k
    K(i)=i;
end

figure()
plot(K,theta)
title('\theta vs Iteration Number')
xlabel('Iteration Number')
ylabel('\theta (rad)')
legend('\theta_{1}','\theta_{2}','\theta_{3}','\theta_{4}','\theta_{5}','\theta_{6}','\theta_{7}')

%Plot Errors
figure()
plot(K,E)
xlabel('Iteration Number')
ylabel('Highest Error (m or rad)')
title('Error vs Iteration Number')


for i=1:3
Check(:,:,i)=BaxterFK(theta(:,step(i))); %validate theta solution set
end
%theta solutions validated


Rorientation=Rorientation/50; %decrease length of vectors for visual pruposes. It is known that their length should be one
Pos=[x;y;z];

figure()
for i=1:length(x)
quiver3(x(i),y(i),z(i),Rorientation(1,1,i),Rorientation(2,1,i),Rorientation(3,1,i),'r') %Xwb
hold on
quiver3(x(i),y(i),z(i),Rorientation(1,2,i),Rorientation(2,2,i),Rorientation(3,2,i),'k') %Ywb
hold on
quiver3(x(i),y(i),z(i),Rorientation(1,3,i),Rorientation(2,3,i),Rorientation(3,3,i),'g') %Zwb
hold on
end
view(-30,-23)
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')
title('Position and Orientation of End Effector')
legend('X^{W}_{B}','Y^{W}_{B}','Z^{W}_{B}')
axis equal

fprintf('\nThe g matrices for the theta values at each goal are:\n')
display(Check)
fprintf('\nThe goal g matrices are:\n')
display(gf)
fprintf('\nSolutions for joint angles at goal position are validated\n')
