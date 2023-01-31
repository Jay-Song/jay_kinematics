%%
clc; clear all; close all;
addpath('./functions');

%% define link structure
global link;
link(1).parent = -1; link(1).child =  2; link(1).sibling = -1; link(1).relative_position = [0;  0;       0]; link(1).joint_angle = 0; link(1).joint_axis = [0; 0; 1]; link(1).pos = [0; 0; 0]; link(1).rot = eye(3);
link(2).parent =  1; link(2).child =  3; link(2).sibling = -1; link(2).relative_position = [0;  0;       0]; link(2).joint_angle = 0; link(2).joint_axis = [1; 0; 0]; link(2).pos = [0; 0; 0]; link(2).rot = eye(3);
link(3).parent =  2; link(3).child =  4; link(3).sibling = -1; link(3).relative_position = [0;  0;       0]; link(3).joint_angle = 0; link(3).joint_axis = [0; 1; 0]; link(3).pos = [0; 0; 0]; link(3).rot = eye(3);
link(4).parent =  3; link(4).child =  5; link(4).sibling = -1; link(4).relative_position = [0;  0;  -0.093]; link(4).joint_angle = 0; link(4).joint_axis = [0; 1; 0]; link(4).pos = [0; 0; 0]; link(4).rot = eye(3);
link(5).parent =  4; link(5).child =  6; link(5).sibling = -1; link(5).relative_position = [0;  0;  -0.093]; link(5).joint_angle = 0; link(5).joint_axis = [0; 1; 0]; link(5).pos = [0; 0; 0]; link(5).rot = eye(3);
link(6).parent =  5; link(6).child =  7; link(6).sibling = -1; link(6).relative_position = [0;  0;       0]; link(6).joint_angle = 0; link(6).joint_axis = [1; 0; 0]; link(6).pos = [0; 0; 0]; link(6).rot = eye(3);
link(7).parent =  6; link(7).child = -1; link(7).sibling = -1; link(7).relative_position = [0;  0; -0.0335]; link(7).joint_angle = 0; link(7).joint_axis = [0; 0; 0]; link(7).pos = [0; 0; 0]; link(7).rot = eye(3);

%% make desired pose
joint_angle_deg = [10.1 20.2 15.3 27.4 10.5 30.6];
joint_angle_rad = joint_angle_deg * pi /180.0

link(1).joint_angle = joint_angle_rad(1);
link(2).joint_angle = joint_angle_rad(2);
link(3).joint_angle = joint_angle_rad(3);
link(4).joint_angle = joint_angle_rad(4);
link(5).joint_angle = joint_angle_rad(5);
link(6).joint_angle = joint_angle_rad(6);

calcForwardKinematics(1);
% drawRobot();

%% solve Inverse Kinematics
IK_result = calcInverseKinematicsForOP3(link(7).rot, link(7).pos)

%% compare desired values and results from Inverse Kinematics
angle_diff_norm = norm(joint_angle_rad - IK_result)

%%
rmpath('./functions');