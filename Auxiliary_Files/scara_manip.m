%SCARA
clear all
close all
clc
l0=0.2;
l1=0.6;
l2=0.3;
P0=[0 l1+l2 l0]';
theta(1:3)=pi()/4;
theta(4)=0.1;
Z=[0 0 0 1];
gst0=[eye(3) P0;Z];
for i=1:4
axis_joints(1:3,i)=[0 0 1]';
end
q_joints=[0 0 0;0 l1 0;0 l1+l2 0;0 0 0]';
type_joints=["revolute";"revolute";"revolute";"prismatic"];
gst=manipdkin(gst0, axis_joints, q_joints, type_joints, theta);