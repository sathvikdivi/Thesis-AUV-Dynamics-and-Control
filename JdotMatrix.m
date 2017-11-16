function [Jmat,Jmat_dot] = JdotMatrix(angles,angles_dot)

phi = angles(1); theta = angles(2); psi = angles(3);
dphi = angles_dot(1); dtheta = angles_dot(2); dpsi = angles_dot(3);

Mat1 = [ cos(psi)*cos(theta), cos(psi)*sin(phi)*sin(theta) - cos(phi)*sin(psi), sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta), 0,                   0,                   0];
Mat2 = [ cos(theta)*sin(psi), cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta), cos(phi)*sin(psi)*sin(theta) - cos(psi)*sin(phi), 0,                   0,                   0];
Mat3 = [         -sin(theta),                              cos(theta)*sin(phi),                              cos(phi)*cos(theta), 0,                   0,                   0];
Mat4 = [                   0,                                                0,                                                0, 1, sin(phi)*tan(theta), cos(phi)*tan(theta)];
Mat5 = [                   0,                                                0,                                                0, 0,            cos(phi),           -sin(phi)];
Mat6 = [                   0,                                                0,                                                0, 0, sin(phi)/cos(theta), cos(phi)/cos(theta)];

Jmat = [Mat1;Mat2;Mat3;Mat4;Mat5;Mat6];

Matd1 = [ - dpsi*cos(theta)*sin(psi) - dtheta*cos(psi)*sin(theta), dphi*sin(phi)*sin(psi) - dpsi*cos(phi)*cos(psi) + dphi*cos(phi)*cos(psi)*sin(theta) + dtheta*cos(psi)*cos(theta)*sin(phi) - dpsi*sin(phi)*sin(psi)*sin(theta), dphi*cos(phi)*sin(psi) + dpsi*cos(psi)*sin(phi) + dtheta*cos(phi)*cos(psi)*cos(theta) - dphi*cos(psi)*sin(phi)*sin(theta) - dpsi*cos(phi)*sin(psi)*sin(theta), 0,                                                                    0,                                                                     0];
Matd2 = [   dpsi*cos(psi)*cos(theta) - dtheta*sin(psi)*sin(theta), dphi*cos(phi)*sin(psi)*sin(theta) - dpsi*cos(phi)*sin(psi) - dphi*cos(psi)*sin(phi) + dpsi*cos(psi)*sin(phi)*sin(theta) + dtheta*cos(theta)*sin(phi)*sin(psi), dpsi*sin(phi)*sin(psi) - dphi*cos(phi)*cos(psi) + dpsi*cos(phi)*cos(psi)*sin(theta) + dtheta*cos(phi)*cos(theta)*sin(psi) - dphi*sin(phi)*sin(psi)*sin(theta), 0,                                                                    0,                                                                     0];
Matd3 = [                                      -dtheta*cos(theta),                                                                                                         dphi*cos(phi)*cos(theta) - dtheta*sin(phi)*sin(theta),                                                                                                       - dphi*cos(theta)*sin(phi) - dtheta*cos(phi)*sin(theta), 0,                                                                    0,                                                                     0];
Matd4 = [                                                       0,                                                                                                                                                             0,                                                                                                                                                             0, 0,        dtheta*sin(phi)*(tan(theta)^2 + 1) + dphi*cos(phi)*tan(theta),         dtheta*cos(phi)*(tan(theta)^2 + 1) - dphi*sin(phi)*tan(theta)];
Matd5 = [                                                       0,                                                                                                                                                             0,                                                                                                                                                             0, 0,                                                       -dphi*sin(phi),                                                        -dphi*cos(phi)];
Matd6 = [                                                       0,                                                                                                                                                             0,                                                                                                                                                             0, 0, (dphi*cos(phi)*cos(theta) + dtheta*sin(phi)*sin(theta))/cos(theta)^2, -(dphi*cos(theta)*sin(phi) - dtheta*cos(phi)*sin(theta))/cos(theta)^2];

Jmat_dot = [Matd1;Matd2;Matd3;Matd4;Matd5;Matd6];

end