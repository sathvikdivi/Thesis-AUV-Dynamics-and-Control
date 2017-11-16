function [Asteer,Bsteer,dFsteer] = SteerControlMatrices(Y,N,Props,u,states)

v = states(2); 
w = states(3); 
p = states(4); 
q = states(5);
r = states(6);
phi = states(10);
theta = states(11);

Msteer = [Props.mass-Y.Yvd, -Y.Yrd, 0;
         -N.Nvd, (Props.Izz - N.Nrd), 0;
         0, 0, 1];

CDsteer = [Y.Yuv*u (Y.Yur*u-Props.mass*u) 0;
           N.Nuv*u N.Nur*u 0;
           0        1      0];

Tausteer = [Y.Ydr*u^2;
            N.Ndr*u^2;
            0];

% delfsteer = [(Y.Ywp*w*p + Y.Ypq*p*q + Y.Yvv*v*abs(v) + Y.Yrr*r*abs(r) + (Props.W - Props.B)*cos(theta)*sin(phi) + Props.mass*w*p  - Props.mass*Props.zG*(q*r)) ;
%              (N.Nwp*w*p + N.Npq*p*q + N.Nvv*v*abs(v) + N.Nrr*r*abs(r) + (Props.xG*Props.W - Props.xB*Props.B)*cos(theta)*sin(phi) + (Props.yG*Props.W - Props.yB*Props.B)*sin(theta) + (Props.Ixx - Props.Iyy)*p*q);
%              0];        

delfsteer = [0;0;0];         
Asteer = inv(Msteer)*CDsteer;
Bsteer = inv(Msteer)*Tausteer;
dFsteer = inv(Msteer)*delfsteer;     

end