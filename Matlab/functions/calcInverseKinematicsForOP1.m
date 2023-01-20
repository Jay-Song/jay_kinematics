function [joint_angle_rad] = calcInverseKinematicsForOP1(rot, pos)

global link;

%% Initialize return values
q1 = 0;
q2 = 0;
q3 = 0;
q4 = 0;
q5 = 0;
q6 = 0;

%% Initialize values for calc
thigh_length = abs(link(4).relative_position(3));
calf_length  = abs(link(5).relative_position(3));
ankle_length = abs(link(7).relative_position(3));

R06 = rot;
p06 = pos + ankle_length*[R06(1,3); R06(2,3); R06(3,3)]; % ankle_length * z_last = pos of 6th joint

%% calc q4
p60 = R06'*(-p06);
C = norm(p60);
q4 = -acos((thigh_length^2 + calf_length^2 - C^2)/(2*thigh_length*calf_length)) + pi;

%% calc q5
alpha = asin(thigh_length*sin(pi-q4)/C);
q5 = -atan2(p60(1), sign(p60(3))*sqrt(p60(2)^2+p60(3)^2)) - alpha;

%% calc q6
q6 = atan2(p60(2), p60(3));

%% calc q1
%R03 = R06*R65*R54*R43
R03 = R06*Rx(-q6)*Ry(-q5-q4);
q1 = atan2(-R03(1,2), R03(2,2));

%% calc q2
q2 = atan2(R03(3,2), -R03(1,2)*sin(q1) + R03(2,2)*cos(q1));

%% calc q3
q3 = atan2(-R03(3,1), R03(3,3));

%% make return value
joint_angle_rad = [q1 q2 q3 q4 q5 q6];

end

