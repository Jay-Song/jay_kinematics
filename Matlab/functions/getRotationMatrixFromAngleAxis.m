function rot = getRotationMatrixFromAngleAxis(angle, axis)

sa = sin(angle);
ca = cos(angle);

rot = ca*eye(3) + sa*getCrossProductMatrix(axis) + (1 - ca)*(axis*axis');

end

