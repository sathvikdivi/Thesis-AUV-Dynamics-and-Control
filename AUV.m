function [Mmat,Props,X,Y,Z,K,M,N] = AUV()

global var1;

[X_norm,Y_norm,Z_norm,K_norm,M_norm,N_norm,RemusProps] = NormalizeQuantities();

LD = RemusProps.L/RemusProps.D;

Dnew = 1.25 ;
Lnew = LD*Dnew;
rho = RemusProps.rho;
Dt = 3;

Norm3 = 0.5*rho*Lnew^3;
Norm4 = 0.5*rho*Lnew^4;
Norm5 = 0.5*rho*Lnew^5;

g = 9.81;
V = (0.85*(0.25*pi*Dnew^2)*Lnew);
m = rho*V;

m = 18000;

W = m*g;
B = (1)*W;


m_wing = 2000;
m_hull = 8000;
m_DTS = 2000;

a = Lnew/2;
b = Dnew/2;
c = b;
Ixx_hull = 0.2*m_hull*(b^2 + c^2);
Iyy_hull = 0.2*m_hull*(a^2 + c^2);
Izz_hull = 0.2*m_hull*(a^2 + b^2);

L_wing = 3;
w_wing = 2.0;
t_wing = 0.5;
x_wing = 0;
y_wing = 0.5*(L_wing + Dnew);
z_wing = 0;
S_wing = L_wing*w_wing;
Cdx_wing = 0.006;
Cdz_wing = 1.25;
Ixx_wing = 2*((m_wing)*(L_wing^2 + t_wing^2)/12 + m_wing*(y_wing^2 + z_wing^2));
Iyy_wing = 2*((m_wing)*(w_wing^2 + t_wing^2)/12 + m_wing*(x_wing^2 + z_wing^2));
Izz_wing = 2*((m_wing)*(L_wing^2 + w_wing^2)/12 + m_wing*(x_wing^2 + y_wing^2));

x_DTS = 0;
y_DTS = (0.5*Dnew + L_wing + 0.5*Dt);
z_DTS = 0;
Ax_DTS = 0.25*pi*Dnew^2*1.47;
Cdx_DTS = 0.35;
Cdy_DTS = 1.20;
Cdz_DTS = 1.20;
Ixx_DTS = 2*(2230 + m_DTS*(y_DTS^2 + z_DTS^2));
Iyy_DTS = 2*(1620 + m_DTS*(x_DTS^2 + z_DTS^2));
Izz_DTS = 2*(1620 + m_DTS*(x_DTS^2 + y_DTS^2));

Ixx = Ixx_hull + Ixx_DTS + Ixx_wing;
Iyy = Iyy_hull + Iyy_DTS + Iyy_wing;
Izz = Izz_hull + Izz_DTS + Izz_wing;
Ixy = 0; Ixz = 0;
Iyx = 0; Iyz = 0;
Izx = 0; Izy = 0;
% % 
% Ixx =  4.98e4; Iyy = 2.2076e4; Izz = 6.77e4;
% Ixx =  3.00e4; Iyy = 4.166e4; Izz = 8.7e4;
% 
% Ixx = RemusProps.Ixx/RemusProps.W*W; Ixy = 0; Ixz = 0;
% Iyy = RemusProps.Iyy/RemusProps.W*W; Iyx = 0; Iyz = 0;
% Izz = RemusProps.Izz/RemusProps.W*W; Izx = 0; Izy = 0;


xB = 0; yB = 0; zB = 0;
xG = 0; yG = 0; zG = 0.35;

Props = struct('mass',m,'W',W,'B',B,'L',Lnew,'D',Dnew,'rho',rho,...
               'Ixx',Ixx,'Iyy',Iyy,'Izz',Izz,'Ixy',Ixy,'Ixz',Ixz,...
               'Iyx',Iyx,'Iyz',Iyz,'Izx',Izx,'Izy',Izy,...
               'xG',xG,'yG',yG,'zG',zG,'xB',xB,'yB',yB,'zB',zB,...
               'V',V);

MRB = [m, 0, 0, 0, m*zG, -m*yG;
        0, m, 0, -m*zG, 0 ,m*xG;
        0 ,0, m, m*yG, -m*xG, 0;
        0, -m*zG, m*yG, Ixx, 0, 0;
        m*zG, 0, -m*xG, 0, Iyy, 0;
        -m*yG, m*xG, 0, 0, 0, Izz];

Xuu = 1.2*(X_norm.Xuu_norm*Norm3 - 2*0.5*rho*Cdx_wing*S_wing - 2*0.5*rho*Cdx_DTS*Ax_DTS);
% Xuu = -4.9e3;
% Xuu = var1*(2*X_norm.Xuu_norm*Norm3) ;
Xud = var1*(2*X_norm.Xud_norm*Norm3);
Xvd = var1*X_norm.Xvd_norm*Norm3;
Xwd = var1*X_norm.Xwd_norm*Norm3;
Xpd = var1*X_norm.Xpd_norm*Norm4;
Xqd = var1*X_norm.Xqd_norm*Norm4;
Xrd = var1*X_norm.Xrd_norm*Norm4;
Xvr = var1*X_norm.Xvr_norm*Norm3; 
Xrr = var1*(X_norm.Xrr_norm*Norm4);
Xqq = var1*(1.5*X_norm.Xqq_norm*Norm4); 
Xwq = var1*X_norm.Xwq_norm*Norm3; 
 
X = struct('Xud',Xud,'Xvd',Xvd,'Xwd',Xwd,... 
           'Xpd',Xpd,'Xqd',Xqd,'Xrd',Xrd,...
           'Xvr',Xvr,'Xrr',Xrr,'Xqq',Xqq,...
           'Xwq',Xwq,'Xuu',Xuu);
       
Yud = var1*Y_norm.Yud_norm*Norm3;
Yvd = var1*(2*Y_norm.Yvd_norm*Norm3);
Ywd = var1*Y_norm.Ywd_norm*Norm3;
Ypd = var1*Y_norm.Ypd_norm*Norm4;
Yqd = var1*Y_norm.Yqd_norm*Norm4;
Yrd = var1*(1.75*Y_norm.Yrd_norm*Norm4);
Yuv = var1*Y_norm.Yuv_norm*Norm3;
Yvv = (2*Y_norm.Yvv_norm*Norm3);
% Yvv = var1*(Y_norm.Yvv_norm*Norm3);
Yrr = (1.25*Y_norm.Yrr_norm*Norm4);
Yur = var1*Y_norm.Yur_norm*Norm3;
Ywp = var1*Y_norm.Ywp_norm*Norm3;
Ypq = var1*Y_norm.Ypq_norm*Norm4;
Ydr = var1*Y_norm.Ydr_norm*Norm3;

% Yvv = -1.255e4;
% Yrr = -1.6e5;
Y = struct('Yud',Yud,'Yvd',Yvd,'Ywd',Ywd,...
           'Ypd',Ypd,'Yqd',Yqd,'Yrd',Yrd,...
           'Yvv',Yvv,'Yrr',Yrr,'Yur',Yur,...
           'Ywp',Ywp,'Ypq',Ypq,'Yuv',Yuv,...
           'Ydr',Ydr);
       
Zud = var1*Z_norm.Zud_norm*Norm3; 
Zvd = var1*Z_norm.Zvd_norm*Norm3;
Zwd = var1*(2*Z_norm.Zwd_norm*Norm3);
Zqd = var1*(2*Z_norm.Zqd_norm*Norm4);
Zpd = var1*Z_norm.Zpd_norm*Norm4;
Zrd = var1*Z_norm.Zrd_norm*Norm4;
Zuq = var1*Z_norm.Zuq_norm*Norm3;
Zvp = var1*Z_norm.Zvp_norm*Norm3;
Zrp = var1*Z_norm.Zrp_norm*Norm4;
% Zww = var1*(Z_norm.Zww_norm*Norm3 + 2*(-0.5*rho*Cdz_DTS*0.5*Dt*sqrt(1.47)*w_wing) + 2*(-0.5*rho*Cdz_wing*L_wing*w_wing));
Zww = (2*Z_norm.Zww_norm*Norm3);
Zqq = (2*Z_norm.Zqq_norm*Norm4);
Zuw = var1*Z_norm.Zuw_norm*Norm3;
Zds = var1*Z_norm.Zds_norm*Norm3;       

% Zww = -2.50e4;
% Zqq = 2.0e5;

Z = struct('Zud',Zud,'Zvd',Zvd,'Zwd',Zwd,...
           'Zpd',Zpd,'Zqd',Zqd,'Zrd',Zrd,...
           'Zww',Zww,'Zqq',Zqq,'Zuq',Zuq,...
           'Zvp',Zvp,'Zrp',Zrp,'Zuw',Zuw,...
           'Zds',Zds);

       
Kpd = 10*K_norm.Kpd_norm*Norm5; 
% Kpd = -2.8638e+05;
Kud = var1*K_norm.Kud_norm*Norm4;
Kvd = var1*K_norm.Kvd_norm*Norm4;
Kwd = var1*K_norm.Kwd_norm*Norm4;
Kqd = var1*K_norm.Kqd_norm*Norm5;
Krd = var1*K_norm.Krd_norm*Norm5;
% Kpp = var1*(K_norm.Kpp_norm*Norm5);
Kpp = -8.2e5;

K = struct('Kud',Kud,'Kvd',Kvd,'Kwd',Kwd,...
           'Kpd',Kpd,'Kqd',Kqd,'Krd',Krd,...
           'Kpp',Kpp);       

Mud = var1*M_norm.Mud_norm*Norm4;
Mvd = var1*M_norm.Mvd_norm*Norm4;
Mwd = var1*(M_norm.Mwd_norm*Norm4);
Mpd = var1*M_norm.Mpd_norm*Norm5;
Mqd = var1*(M_norm.Mqd_norm*Norm5);
Mrd = var1*M_norm.Mrd_norm*Norm5;
Mww = (M_norm.Mww_norm*Norm4);
Mqq = (M_norm.Mqq_norm*Norm5);
Muw = var1*M_norm.Muw_norm*Norm4; 
Muq = var1*M_norm.Muq_norm*Norm4; 
Mvp = var1*M_norm.Mvp_norm*Norm4; 
Mrp = var1*M_norm.Mrp_norm*Norm5; 
Mds = var1*M_norm.Mds_norm*Norm4;       

% Mww = -1.77e3;
% Mqq = -1.1e6;

M = struct('Mud',Mud,'Mvd',Mvd,'Mwd',Mwd,...
           'Mpd',Mpd,'Mqd',Mqd,'Mrd',Mrd,...
           'Mww',Mww,'Mqq',Mqq,'Muq',Muq,...
           'Muw',Muw,'Mvp',Mvp,'Mrp',Mrp,...
           'Mds',Mds);

Nud = var1*N_norm.Nud_norm*Norm4;
Nvd = var1*(N_norm.Nvd_norm*Norm4);
Nwd = var1*N_norm.Nwd_norm*Norm4;
Npd = var1*N_norm.Npd_norm*Norm5;
Nqd = var1*N_norm.Nqd_norm*Norm5;
Nrd = var1*(1*N_norm.Nrd_norm*Norm5);
Nvv = (N_norm.Nvv_norm*Norm4);
Nrr = (N_norm.Nrr_norm*Norm5);
Nuv = var1*N_norm.Nuv_norm*Norm4;
Nur = var1*N_norm.Nur_norm*Norm4;
Nwp = var1*N_norm.Nwp_norm*Norm4;
Npq = var1*N_norm.Npq_norm*Norm5;
Ndr = var1*N_norm.Ndr_norm*Norm4;

% Nvv = 1.7e3;
% Nrr = -1.1e6;

N = struct('Nud',Nud,'Nvd',Nvd,'Nwd',Nwd,...
           'Npd',Npd,'Nqd',Nqd,'Nrd',Nrd,...
           'Nrr',Nrr,'Nvv',Nvv,'Nuv',Nuv,...
           'Nur',Nur,'Nwp',Nwp,'Npq',Npq,...
           'Ndr',Ndr);
       
MA = -[Xud,Xvd,Xwd,Xpd,Xqd,Xrd;
       Yud,Yvd,Ywd,Ypd,Yqd,Yrd;
       Zud,Zvd,Zwd,Zpd,Zqd,Zrd;
       Kud,Kvd,Kwd,Kpd,Kqd,Krd;
       Mud,Mvd,Mwd,Mpd,Mqd,Mrd;
       Nud,Nvd,Nwd,Npd,Nqd,Nrd];

Mmat = MA + MRB;
       
end