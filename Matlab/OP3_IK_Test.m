%%
clc; clear all; close all;
addpath('./functions');

%% define link structure
global link;
link(1).relative_position = [0;       0;        0]; link(1).joint_angle = 0; link(1).joint_axis = [0; 0; 1]; link(1).joint_dir = 1; link(1).pos = [0; 0; 0]; link(1).rot = eye(3);
link(2).relative_position = [0;       0;        0]; link(2).joint_angle = 0; link(2).joint_axis = [1; 0; 0]; link(2).joint_dir = 1; link(2).pos = [0; 0; 0]; link(2).rot = eye(3);
link(3).relative_position = [0.0001;  0;        0]; link(3).joint_angle = 0; link(3).joint_axis = [0; 1; 0]; link(3).joint_dir = 1; link(3).pos = [0; 0; 0]; link(3).rot = eye(3);
link(4).relative_position = [0.0001;  0; -0.11015]; link(4).joint_angle = 0; link(4).joint_axis = [0; 1; 0]; link(4).joint_dir = 1; link(4).pos = [0; 0; 0]; link(4).rot = eye(3);
link(5).relative_position = [0;       0;    -0.11]; link(5).joint_angle = 0; link(5).joint_axis = [0; 1; 0]; link(5).joint_dir = 1; link(5).pos = [0; 0; 0]; link(5).rot = eye(3);
link(6).relative_position = [0;       0;        0]; link(6).joint_angle = 0; link(6).joint_axis = [1; 0; 0]; link(6).joint_dir = 1; link(6).pos = [0; 0; 0]; link(6).rot = eye(3);
link(7).relative_position = [0;       0;  -0.0305]; link(7).joint_angle = 0; link(7).joint_axis = [0; 0; 0]; link(7).joint_dir = 1; link(7).pos = [0; 0; 0]; link(7).rot = eye(3);

%% make desired pose
joint_angle_deg = [10.1 20.2 15.3 27.4 10.5 67.6];
joint_angle_rad = joint_angle_deg * pi /180.0
desired = calcForwardKinematics(joint_angle_rad);
% drawRobot();

%% solve Inverse Kinematics
IK_result = calcInverseKinematicsForOP3(desired(1:3, 1:3), desired(1:3, 4))

%% compare desired values and results from Inverse Kinematics
angle_diff_norm = norm(joint_angle_rad - IK_result)
pose_diff_norm  = norm(desired - calcForwardKinematics(IK_result))

%%
rmpath('./functions');