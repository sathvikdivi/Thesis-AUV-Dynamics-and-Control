function [Mmat,Props,X,Y,Z,K,M,N] = RemusAUV()


var1 = 1;

%% AUV Props

% Hull
D = 1.91e-1; % m
L = 1.33; % m
g = 9.81;
rho = 1030;% kg/m^3
m = 300/9.81; % kg
W = 300; % N
B = 1*W; % N  %(1 + 1e-7, 1 + 1e-8)
Af = 2.85e-2;
Sw = 7.09e-1;
Ap = 2.26e-1;
V = 3.15e-2;

Ixx = 1.77e-1; Ixy = 0; Ixz = 0;
Iyy = 3.45; Iyx = Ixy; Iyz = 0;
Izz = 3.45; Izx = Ixz; Izy = Iyz;

xB = 0; yB = 0; zB = 0;
xG = 0; yG = 0; zG = 0.04; %1.96e-2


Props = struct('mass',m,'W',W,'B',B,'L',L,'D',D,'rho',rho,...
               'Ixx',Ixx,'Iyy',Iyy,'Izz',Izz,'Ixy',Ixy,'Ixz',Ixz,...
               'Iyx',Iyx,'Iyz',Iyz,'Izx',Izx,'Izy',Izy,...
               'xG',xG,'yG',yG,'zG',zG,'xB',xB,'yB',yB,'zB',zB,...
               'Af',Af,'Sw',Sw,'Ap',Ap,'V',V);

%% Rigid Body Mass Matrix
MRB = [m, 0, 0, 0, m*zG, -m*yG;
        0, m, 0, -m*zG, 0 ,m*xG;
        0 ,0, m, m*yG, -m*xG, 0;
        0, -m*zG, m*yG, Ixx, 0, 0;
        m*zG, 0, -m*xG, 0, Iyy, 0;
        -m*yG, m*xG, 0, 0, 0, Izz];

%% Hydrodynamic Coefficients

% X hydrodynamic coefficients
Xuu = -1.62;
Xud = (-9.30e-1);
Xvd = 0;
Xwd = 0;
Xpd = 0;
Xqd = 0;
Xrd = 0;
Xnn = 0.0001497;
Xvr = (+3.55e1); % Xvr = -Yvd
Xrr = (-1.93e0); % Xrr = -Yrd
Xqq = (-1.93); % Xqq = Zqd
Xwq = (-3.55e1); % Xwq = Zwd

Xud = var1*Xud; Xvd = var1*Xvd; Xwd = var1*Xwd; 
Xpd = var1*Xpd; Xqd = var1*Xqd; Xrd = var1*Xrd;

Xvr = var1*Xvr; Xwq = var1*Xwq; 
Xqq = var1*Xqq; Xrr = var1*Xrr;  

X = struct('Xud',Xud,'Xvd',Xvd,'Xwd',Xwd,...
           'Xpd',Xpd,'Xqd',Xqd,'Xrd',Xrd,...
           'Xvr',Xvr,'Xrr',Xrr,'Xqq',Xqq,...
           'Xwq',Xwq,'Xuu',Xuu,'Xnn',Xnn);
       
% Y hydrodynamic coefficients
Yud = 0; % Added mass
Yvd = (-3.55e1); % added mass
Ywd = 0; % added mass
Ypd = 0; % added `mass
Yqd = 0; % added mass
Yrd = (1.93); % added mass
Yuv = (-2.86e1); %  body cross-lift coefficient
Yvv = -1.31e2; % body cross drag coefficient
Yrr = 6.32e-1; % body cross drag coefficient
Yur = (5.22);
Ywp = (3.55e1); % Ywp = -Zwd
Ypq = (1.93e0); % Ypq = -Zqd
Ydr = 9.64;

Yud = var1*Yud; Yvd = var1*Yvd; Ywd = var1*Ywd; 
Ypd = var1*Ypd; Yqd = var1*Yqd; Yrd = var1*Yrd;

Ywp = var1*Ywp; Ypq = var1*Ypq;

Y = struct('Yud',Yud,'Yvd',Yvd,'Ywd',Ywd,...
           'Ypd',Ypd,'Yqd',Yqd,'Yrd',Yrd,...
           'Yvv',Yvv,'Yrr',Yrr,'Yur',Yur,...
           'Ywp',Ywp,'Ypq',Ypq,'Yuv',Yuv,...
           'Ydr',Ydr);
       
% Z hydrodynamic coefficients
Zud = 0; 
Zvd = 0;
Zwd = (-3.55e1);
Zqd = (-1.93e0);
Zpd = 0;
Zrd = 0;
Zuq = (-5.22);
Zvp = (-3.55e1);
Zrp = (1.93e0);
Zww = -1.31e2;
Zqq = -6.32e-1;
Zuw = (-2.86e1);
Zds = -9.64;

Zud = var1*Zud; Zvd = var1*Zvd; Zwd = var1*Zwd;
Zpd = var1*Zpd; Zqd = var1*Zqd; Zrd = var1*Zrd;

Zvp = var1*Zvp; Zrp = var1*Zrp;

Z = struct('Zud',Zud,'Zvd',Zvd,'Zwd',Zwd,...
           'Zpd',Zpd,'Zqd',Zqd,'Zrd',Zrd,...
           'Zww',Zww,'Zqq',Zqq,'Zuq',Zuq,...
           'Zvp',Zvp,'Zrp',Zrp,'Zuw',Zuw,...
           'Zds',Zds);
       
% K hydrodynamic coefficients
Kpd = (-7.04e-2); 
Kud = 0;
Kvd = 0;
Kwd = 0;
Kqd = 0;
Krd = 0;
Kpp = -1.3e-1;
Knn = 2.2e-5; 

Kpd = var1*Kpd;

K = struct('Kud',Kud,'Kvd',Kvd,'Kwd',Kwd,...
           'Kpd',Kpd,'Kqd',Kqd,'Krd',Krd,...
           'Kpp',Kpp,'Knn',Knn);

% M hydrodynamic coefficients
Mud = 0;
Mvd = 0;
Mwd = (-1.93);
Mpd = 0;
Mqd = (-4.88);
Mrd = 0;
Mww = 3.18;
Mqq = -1.88e2;
Muw = (2.40e1); % Muw = -(Zwd - Xud)
Muq = (-2); % Muq = -Zqd
Mvp = (-1.93); % Mvp = -Yrd
Mrp = (4.86); % Mrp = (Kpd - Nrd)
Mds = -6.15;

Mud = var1*Mud; Mvd = var1*Mvd; Mwd = var1*Mwd;
Mpd = var1*Mpd; Mqd = var1*Mqd; Mrd = var1*Mrd;

Mvp = var1*Mvp; Mrp = var1*Mrp; Muq = var1*Muq;

M = struct('Mud',Mud,'Mvd',Mvd,'Mwd',Mwd,...
           'Mpd',Mpd,'Mqd',Mqd,'Mrd',Mrd,...
           'Mww',Mww,'Mqq',Mqq,'Muq',Muq,...
           'Muw',Muw,'Mvp',Mvp,'Mrp',Mrp,...
           'Mds',Mds);

% N hydrodynamic coefficients
Nud = 0;
Nvd = (1.93);
Nwd = 0;
Npd = 0;
Nqd = 0;
Nrd = (-4.88);
Nvv = -3.18;
Nrr = -9.40e1;
Nuv = (-2.40e1); % Nuv = -(Xud - Yvd)
Nur = (-2); % Nur = Yrd
Nwp = (-1.93); % Nwp = Zqd
Npq = (-4.86); % Npq = -(Kpd - Mqd)
Ndr = -6.15; 

Nud = var1*Zud; Nvd = var1*Nvd; Nwd = var1*Nwd;
Npd = var1*Zpd; Nqd = var1*Nqd; Nrd = var1*Nrd;

Nur = var1*Nur; Npq = var1*Npq; Nwp = var1*Nwp;

N = struct('Nud',Nud,'Nvd',Nvd,'Nwd',Nwd,...
           'Npd',Npd,'Nqd',Nqd,'Nrd',Nrd,...
           'Nrr',Nrr,'Nvv',Nvv,'Nuv',Nuv,...
           'Nur',Nur,'Nwp',Nwp,'Npq',Npq,...
           'Ndr',Ndr);

%% Added Mass Matrix
MA = -[Xud,Xvd,Xwd,Xpd,Xqd,Xrd;
       Yud,Yvd,Ywd,Ypd,Yqd,Yrd;
       Zud,Zvd,Zwd,Zpd,Zqd,Zrd;
       Kud,Kvd,Kwd,Kpd,Kqd,Krd;
       Mud,Mvd,Mwd,Mpd,Mqd,Mrd;
       Nud,Nvd,Nwd,Npd,Nqd,Nrd];
% MA(2,6) = 0;
% MA(3,5) = 0;
% MA(5,3) = 0;
% MA(6,2) = 0;

% indArray = find(MA ~= 0);
% for count = 1:length(indArray)
%     MA(indArray(count)) = var*MA(indArray(count));
% end
%% Complete Mass Matrix

% Final Mass and Inertia Matrix
% MA = 0;
Mmat = MRB + MA;
% Mmat(1,5) = Mmat(1,5) - 0.3;
% Mmat(2,4) = 0;
% Mmat(4,2) = 0;
% Mmat(5,1) = Mmat(5,1) - 0.3;

end
