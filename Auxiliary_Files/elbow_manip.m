%Elbow
clear all
close all
clc
l0=0.8;
l1=0.8;
l2=0.5;
P0=[0 l1+l2 l0]';
theta(1:6)=pi()/3;
Z=[0 0 0 1];
gst0=[eye(3) P0;Z];
axis_joints=[0 0 1;-1 0 0;-1 0 0;0 0 1;-1 0 0;0 1 0]';
q_joints=[0 0 l0;0 0 l0;0 l1 l0;0 l1+l2 l0;0 l1+l2 l0;0 l1+l2 l0]';
type_joints(1:6,1)="revolute";
pos=manipdkin(gst0, axis_joints, q_joints, type_joints, theta);

