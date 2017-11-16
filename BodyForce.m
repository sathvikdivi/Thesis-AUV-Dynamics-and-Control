function FB = BodyForce(Props,velocities)

u = velocities(1); v = velocities(2); w = velocities(3);
p = velocities(4); q = velocities(5); r = velocities(6);

XB = Props.mass*(-v*r + w*q - Props.xG*(q^2 + r^2) + Props.yG*p*q + Props.zG*(p*r));
YB = Props.mass*(-w*p + u*r - Props.yG*(r^2 + p^2) + Props.zG*(q*r) + Props.xG*q*p);
ZB = Props.mass*(-u*q + v*p - Props.zG*(p^2 + q^2) + Props.xG*(r*p) + Props.yG*r*q);
KB = (Props.Izz - Props.Iyy)*q*r + Props.mass*Props.yG*(-u*q + v*p) - Props.mass*Props.zG*(-w*p + u*r);
MB = (Props.Ixx - Props.Izz)*r*p + Props.mass*Props.zG*(-v*r + w*q) - Props.mass*Props.xG*(-u*q + v*p);
NB = (Props.Iyy - Props.Ixx)*p*q + Props.mass*Props.xG*(-w*p + u*r) - Props.mass*Props.yG*(-v*r + w*q);

FB = [XB;YB;ZB;KB;MB;NB];
% FB(find(abs(FB) < 1e-3)) = 0;
end