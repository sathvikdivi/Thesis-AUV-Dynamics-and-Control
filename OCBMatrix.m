function [OCB] = OCBMatrix(angles)

phi = angles(1); theta = angles(2); psi = angles(3);

OCB = [cos(psi)*cos(theta), -sin(psi)*cos(phi)+sin(theta)*sin(phi)*cos(psi), sin(psi)*sin(phi) + cos(psi)*cos(phi)*sin(theta);
       sin(psi)*cos(theta), cos(psi)*cos(phi)+sin(phi)*sin(theta)*sin(psi), -cos(psi)*sin(phi)+sin(theta)*sin(psi)*cos(phi);
       -sin(theta), cos(theta)*sin(phi), cos(theta)*cos(phi)];
end