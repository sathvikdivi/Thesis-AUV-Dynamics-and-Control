function [Adive,Bdive,dFdive] = DiveControlMatrices(Z,M,Props,u,states)

v = states(2); 
w = states(3); 
p = states(4); 
q = states(5);
r = states(6);
phi = states(10);
theta = states(11);

Mdive = [Props.mass-Z.Zwd, -Z.Zqd, 0, 0;
         -M.Mwd, (Props.Iyy - M.Mqd), 0, 0;
         0, 0, 1, 0;
         0, 0, 0, 1];

CDdive = [Z.Zuw*u, (Z.Zuq*u+Props.mass*u), 0, 0;
           M.Muw*u, -M.Muq*u, -(Props.zG*Props.W - Props.zB*Props.B), 0;
           0, 1, 0, 0;
           1, 0, -u, 0];

Taudive = [Z.Zds*u^2;
            M.Mds*u^2;
            0;
            0];
% 
% delfdive = [-Props.mass*v*p + Props.mass*Props.zG*(p^2 + q^2) + Z.Zvp*v*p + Z.Zrp*r*p + Z.Zww*w*abs(w) + Z.Zqq*q*abs(q) + (Props.W - Props.B)*cos(theta)*cos(phi);
%             (Props.Izz - Props.Ixx)*r*p + Props.mass*Props.zG*v*r - Props.mass*Props.zG*w*q + M.Mvp*v*p + M.Mrp*r*p + M.Mww*w*abs(w) + M.Mqq*q*abs(q);
%             0;
%             0];

delfdive = [0;0;0;0];         

% Mdive = [(Props.Iyy - M.Mqd), 0, 0;
%          0, 1, 0;
%          0, 0, 1];
% 
% CDdive = [-M.Muq*u, -(Props.zG*Props.W - Props.zB*Props.B), 0;
%            1, 0, 0;
%            0, -u, 0];
% 
% Taudive = [ M.Mds*u^2;
%             0;
%             0];
% % 
% delfdive = [-Props.mass*v*p + Props.mass*Props.zG*(p^2 + q^2) + Z.Zvp*v*p + Z.Zrp*r*p + Z.Zww*w*abs(w) + Z.Zqq*q*abs(q) + (Props.W - Props.B)*cos(theta)*cos(phi);
%             (Props.Izz - Props.Ixx)*r*p + Props.mass*Props.zG*v*r - Props.mass*Props.zG*w*q + M.Mvp*v*p + M.Mrp*r*p + M.Mww*w*abs(w) + M.Mqq*q*abs(q);
%             0;
%             0];

% delfdive = [0;0;0];  


Adive = inv(Mdive)*CDdive;
Bdive = inv(Mdive)*Taudive;
dFdive = inv(Mdive)*delfdive;     

end