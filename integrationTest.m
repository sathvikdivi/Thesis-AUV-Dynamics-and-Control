
clear
clc

Dnew = 1.25;
Dt = 3;
Lnew = 8.7;
rho =  1025;

a = 1.74;
b = 3.48;
c = 3.48;
d = Dnew;
n = 2;
th = rad2deg(25);
l = Lnew;
Cd_hull = 0.25;
Cd_turb = 0.35;
Cd_wing = 0.006;
cdc = 1.1;
cdf = 0.8;

L_wing = 3;
w_wing = 2.5;
x_wing = 3.625;

Af_hull = 0.25*pi*Dnew^2;
Ae_turb = 0.25*pi*Dt^2*1.47;
S_wing = L_wing*w_wing;

Xuu = -1.25*(0.5*1025*Cd_hull*Af_hull + 2*0.5*1025*Cd_turb*Ae_turb + 2*0.5*1025*Cd_wing*S_wing);

fun_nose = @(x) 0.5.*d.*(1 - ((x - a)./a).^2).^(1./n);
fun_tail = @(x) 0.5.*d - ((3*d)/(2*c.^2) - tan(th)/c)*(x - l).^2 + ((d/c.^3) - tan(th)/c.^2)*(x - (a+b)).^3;
fun_body = @(x) 0.5.*d.*ones(size(x));

Area_yy = 2*integral(fun_nose,0,a) + 2*integral(fun_tail,a+b,a+b+c) + 2*integral(fun_body,0,b);



Yvv = 1.25*(-0.5*rho*cdc*(Area_yy) - 2*0.5*rho*1.2*(0.5*Dt*sqrt(1.47)*w_wing));


Zww = 1.25*(Yvv - 2*0.5*rho*1.25*L_wing*w_wing) ; 

fun_nosex = @(x) x.*0.5.*d.*(1 - ((x - a)./a).^2).^(1./n);
fun_tailx = @(x) x.*0.5.*d - ((3*d)/(2*c.^2) - tan(th)/c)*(x - l).^2 + ((d/c.^3) - tan(th)/c.^2)*(x - (a+b)).^3;
fun_bodyx = @(x) 0.5.*x.*d.*ones(size(x));

Areax_yy = 2*integral(fun_nosex,0,a) + 2*integral(fun_tailx,a+b,a+b+c) + 2*integral(fun_bodyx,0,b);
Mww = 1.25*(0.5*rho*cdc*Areax_yy - 2*x_wing*0.5*rho*L_wing*w_wing*cdf);
Nvv = -Mww;


fun_nosexx = @(x) abs(x).*x.*0.5.*d.*(1 - ((x - a)./a).^2).^(1./n);
fun_tailxx = @(x) abs(x).*x.*0.5.*d - ((3*d)/(2*c.^2) - tan(th)/c)*(x - l).^2 + ((d/c.^3) - tan(th)/c.^2)*(x - (a+b)).^3;
fun_bodyxx = @(x) abs(x).*x.*0.5.*d.*ones(size(x));

Areaxx_yy = 2*integral(fun_nosexx,0,a) + 2*integral(fun_tailxx,a+b,a+b+c) + 2*integral(fun_bodyxx,0,b);
Yrr = -0.5*rho*cdc*Areaxx_yy*1.25 ;
Zqq = -Yrr;

fun_nosexxx = @(x) x.^3.*0.5.*d.*(1 - ((x - a)./a).^2).^(1./n);
fun_tailxxx = @(x) x.^3.*0.5.*d - ((3*d)/(2*c.^2) - tan(th)/c)*(x - l).^2 + ((d/c.^3) - tan(th)/c.^2)*(x - (a+b)).^3;
fun_bodyxxx = @(x) x.^3.*0.5.*d.*ones(size(x));

Areaxxx_yy = 2*integral(fun_nosexxx,0,a) + 2*integral(fun_tailxxx,a+b,a+b+c) + 2*integral(fun_bodyxxx,0,b);
Mqq = -0.5*rho*cdc*Areaxxx_yy*1.25 ;
Nrr = Mqq;

Yvvf = 2*0.5*rho*L_wing*w_wing*cdf;
Kpp = 1.25*Yvvf*x_wing^3;






a = 3.64;
xf1 = -1.25;
xf2 = 1.25;
fx = @(x) (2/pi)*1025*a^4*ones(size(x));
integral(fx,xf1,xf2)
