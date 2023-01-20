function calcForwardKinematics(joint_index)

global link;

if joint_index == -1
   return;
elseif (joint_index == 1)
   link(joint_index).pos = link(joint_index).relative_position;
   link(joint_index).rot = getRotationMatrixFromAngleAxis(link(joint_index).joint_angle, link(joint_index).joint_axis);
else
   parent_index = link(joint_index).parent;
   link(joint_index).pos = link(parent_index).pos + link(parent_index).rot*link(joint_index).relative_position;
   link(joint_index).rot = link(parent_index).rot*getRotationMatrixFromAngleAxis(link(joint_index).joint_angle, link(joint_index).joint_axis);
end

calcForwardKinematics(link(joint_index).sibling);
calcForwardKinematics(link(joint_index).child);

% pos = [0 ; 0; 0];
% rot = eye(3);
% 
% for i = 1:length(link)
%     if i <= length(joint_angle_rad)
%         link(i).joint_angle = joint_angle_rad(i);
%     else
%         link(i).joint_angle = 0;
%     end
%     
%     % t(0->i) = t(0->i-1) + R(0 -> i-1)*relative_position
%     pos = pos + rot*link(i).relative_position;
%     
%     % R(0->i) = R(0->i-1) * R(i-1->i)
%     rot = rot * getRotationMatrixFromAngleAxis(link(i).joint_angle, link(i).joint_axis);
%     
%     link(i).pos = pos;
%     link(i).rot = rot;
% end
% 
% FK_result = [link(length(link)).rot link(length(link)).pos;
%                               0 0 0               1       ];

end

