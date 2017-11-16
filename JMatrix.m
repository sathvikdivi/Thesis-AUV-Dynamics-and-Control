function [J] = JMatrix(angles)

phi = angles(1); theta = angles(2); psi = angles(3);

J = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
     0,      cos(phi)      , -sin(phi);
     0, sin(phi)/cos(theta), cos(phi)/cos(theta)];
end