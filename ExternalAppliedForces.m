function Fext = ExternalAppliedForces(X,Y,Z,K,M,N,Props,velocities,angles,inputs)

phi = angles(1); theta = angles(2); psi = angles(3);

u = velocities(1); v = velocities(2); w = velocities(3);
p = velocities(4); q = velocities(5); r = velocities(6);

props = inputs(1); delR = inputs(2); delS = inputs(3);

Xprops = inputs(1);
Kprops = 0;

Xext = -(Props.W - Props.B)*sin(theta) + X.Xuu*u*abs(u) + X.Xwq*w*q + X.Xqq*q*q + X.Xvr*v*r + X.Xrr*r*r ;
Yext = (Props.W - Props.B)*cos(theta)*sin(phi) + Y.Yvv*v*abs(v) + Y.Yrr*r*abs(r) + Y.Yur*u*r + Y.Ywp*w*p + Y.Ypq*p*q + Y.Yuv*u*v  ;
Zext = (Props.W - Props.B)*cos(theta)*cos(phi) + Z.Zww*w*abs(w) + Z.Zqq*q*abs(q) + Z.Zuq*u*q + Z.Zvp*v*p + Z.Zrp*r*p + Z.Zuw*u*w ;
Kext = (Props.yG*Props.W - Props.yB*Props.B)*cos(theta)*cos(phi) - (Props.zG*Props.W - Props.zB*Props.B)*cos(theta)*sin(phi) + K.Kpp*p*abs(p) ;
Mext = -(Props.zG*Props.W - Props.zB*Props.B)*sin(theta) - (Props.xG*Props.W - Props.xB*Props.B)*cos(theta)*cos(phi) + M.Mww*w*abs(w) + M.Mqq*q*abs(q) + M.Muq*u*q + M.Mvp*v*p + M.Mrp*r*p + M.Muw*u*w ;
Next = (Props.xG*Props.W - Props.xB*Props.B)*cos(theta)*sin(phi) + (Props.yG*Props.W - Props.yB*Props.B)*sin(theta) + N.Nvv*v*abs(v) + N.Nrr*r*abs(r) + N.Nur*u*r + N.Nwp*w*p + N.Npq*p*q + N.Nuv*u*v ;

Fext = [Xext;Yext;Zext;Kext;Mext;Next];

end