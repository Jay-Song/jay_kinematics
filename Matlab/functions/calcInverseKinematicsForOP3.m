function [joint_angle_rad] = calcInverseKinematicsForOP3(rot, pos)

global link;

%% Initialize return values
q1 = 0;
q2 = 0;
q3 = 0;
q4 = 0;
q5 = 0;
q6 = 0;

%% Initialize values for calc
thigh_length = sqrt(link(4).relative_position(1)^2 + link(4).relative_position(3)^2);
calf_length = abs(link(5).relative_position(3));
ankle_length = abs(link(7).relative_position(3));

hip_offset_angle_rad = atan2(link(4).relative_position(1), abs(link(4).relative_position(3)));
knee_offset_anglr_rad = -hip_offset_angle_rad;

R06 = rot;
p06 = pos + ankle_length*[R06(1,3); R06(2,3); R06(3,3)]; % ankle_length * z_last = pos of 6th joint

%% calc q6
p60 = R06'*(-p06);
q6 = atan2(p60(2), p60(3));

%% calc q1
R05 = R06*Rx(-q6);
q1 = atan2(-R05(1,2), R05(2,2));

%% calc q4
p03 = Rz(q1)*link(3).relative_position;
p36 = p06 - p03;
C = norm(p36);
q4 = -acos((thigh_length^2 + calf_length^2 - C^2)/(2*thigh_length*calf_length)) + pi;

%% calc q5
alpha = asin(thigh_length*sin(pi-q4)/C);
p63 = R06'*(-p36);
q5 = -atan2(p63(1), sign(p63(3))*sqrt(p63(2)^2+p63(3)^2)) - alpha;

%% calc q2
R13 = Rz(-q1)*R05*Ry(-q5-q4);
q2 = atan2(R13(3,2), R13(2,2));

%% calc q3
q3 = atan2(R13(1,3), R13(1,1));

%% make return value
joint_angle_rad = [q1 q2 q3+hip_offset_angle_rad q4+knee_offset_anglr_rad q5 q6];

for i=1:length(joint_angle_rad)
    joint_angle_rad(i) = link(i).joint_dir * joint_angle_rad(i);
end

end

