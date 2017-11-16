clear
clc
close all

% desired
x_d = 480; y_d = 0; z_d = 0; phi_d = 0; theta_d = 0; psi_d = 0;
u_d = 0; v_d = 0; w_d = 0; p_d = 0; q_d = 0; r_d = 0;

% desired dot
xdot_d = 0; ydot_d = 0; zdot_d = 0; 
phidot_d = 0; thetadot_d = 0; psidot_d = 0;
udot_d = 0; vdot_d = 0; wdot_d = 0; 
pdot_d = 0; qdot_d = 0; rdot_d = 0;

% desired dot dot
xdotdot_d = 0; ydotdot_d = 0; zdotdot_d = 0; 
phidotdot_d = 0; thetadotdot_d = 0; psidotdot_d = 0;
udotdot_d = 0; vdotdot_d = 0; wdotdot_d = 0; 
pdotdot_d = 0; qdotdot_d = 0; rdotdot_d = 0;

global Mmat Props X Y Z K M N inputs Fctrl;

save_msg = 0; test_msg = 'PositiveB_1en5pcnt'; case_msg = 'RollStab';

% starting 
x = 0; y = 0; z = 0;
phi = 0; theta = 0; psi = 0;

u = 0; v = 0; w = 0;
p = 0; q = 0; r = 0;

% dots 
xdot = 0; ydot = 0; zdot = 0; 
phidot = 0; thetadot = 0; psidot = 0;
udot = 0; vdot = 0; wdot = 0; 
pdot = 0; qdot = 0; rdot = 0;

% double dots
xdotdot = 0; ydotdot = 0; zdotdot = 0; 
phidotdot = 0; thetadotdot = 0; psidotdot = 0;
udotdot = 0; vdotdot = 0; wdotdot = 0; 
pdotdot = 0; qdotdot = 0; rdotdot = 0;

props = 0; delR = 0; delS = 0;
Xprops = 0;
inputs = [Xprops;delR;delS];

[Mmat,Props,X,Y,Z,K,M,N] = RemusAUV();

Kp = 20*eye(6);
Kd = 0.5*eye(1);

ndotdot_d = [xdotdot_d;ydotdot_d;zdotdot_d;phidotdot_d;thetadotdot_d;psidotdot_d];
ndot_d = [xdot_d;ydot_d;zdot_d;phidot_d;thetadot_d;psidot_d];
n_d = [x_d;y_d;z_d;phi_d;theta_d;psi_d];



told = 0;
tend = 600;
tspan = told:1:tend;

init = [u,v,w,p,q,r,x,y,z,phi,theta,psi];

StateVec = [init];
TVec = told;

while told <= tend
    told
    tspan = told:0.1:told + 1;
    ndot = [xdot;ydot;zdot;phidot;thetadot;psidot];
    n = [x;y;z;phi;theta;psi];
    an = ndotdot_d - Kd*(ndot - ndot_d) - Kp*(n - n_d);
    [Jmat,Jmat_dot] = JdotMatrix([phi,theta,psi],[phidot,thetadot,psidot]);
    av = inv(Jmat)*(an - Jmat_dot*[u;v;w;p;q;r]);
    Fext = ExternalAppliedForces(X,Y,Z,K,M,N,Props,[u;v;w;p;q;r],[phi;theta;psi],inputs);
    FB = BodyForce(Props,[u;v;w;p;q;r]);
    Fctrl = Mmat*av + FB - Fext;
    if abs(Fctrl(1)) > 10
        if Fctrl(1) >= 0
            sign = 1;
        else
            sign = -1;
        end
        Fctrl(1) = sign*10;
        Xprops = Fctrl(1);
    end
    if abs(Fctrl(2)) > 9.6
        if Fctrl(2) >= 0
            sign = 1;
        else
            sign = -1;
        end
        Fctrl(2) = sign*9.6;
        Fctrl(2) = 0;
    end
    if abs(Fctrl(3)) > 9.6
        if Fctrl(3) >= 0
            sign = 1;
        else
            sign = -1;
        end
        Fctrl(3) = sign*9.6;
        Fctrl(3) = 0;
    end
    Fctrl(4) = 0 ;
    Fctrl(5) = M.Mds*delS*u^2;
    Fctrl(6) = N.Ndr*delR*u^2;
    [t,states] = ode45('odefunccontrol',tspan,init);
    u = states(end,1);
    v = states(end,2);
    w = states(end,3);
    p = states(end,4);
    q = states(end,5);
    r = states(end,6);
    x = states(end,7);
    y = states(end,8);
    z = states(end,9);
    phi = states(end,10);
    theta = states(end,11);
    psi = states(end,12);
    told = t(end);
    init = [u,v,w,p,q,r,x,y,z,phi,theta,psi];
    StateVec = vertcat(StateVec,init);
    TVec = vertcat(TVec,told);
end

figure(1)
plot(TVec,StateVec(:,1),'LineWidth',2);
xlabel('time(s)')
ylabel('velocity(m/s)')
title('Surge Velocity');
grid on
set(gca, 'FontName', 'Calibri');
set(gca, 'FontSize', 17);   
set(gcf, 'Color', [1, 1, 1])
if (save_msg == 1)
    fpath = 'C:\Users\sdivi\Documents\AUV\Matlab\control test\Dynamic Model\Remus\Plots\';
    filename = [test_msg,'_',case_msg,'_','SurgeVel'];
    saveas(gcf,fullfile(fpath,filename),'jpeg');
end

figure(2)
plot(TVec,StateVec(:,2),'LineWidth',2);
xlabel('time(s)')
ylabel('velocity(m/s)')
title('Sway Velocity');
grid on
set(gca, 'FontName', 'Calibri');
set(gca, 'FontSize', 17);   
set(gcf, 'Color', [1, 1, 1])
if (save_msg == 1)
    fpath = 'C:\Users\sdivi\Documents\AUV\Matlab\control test\Dynamic Model\Remus\Plots\';
    filename = [test_msg,'_',case_msg,'_','SwayVel'];
    saveas(gcf,fullfile(fpath,filename),'jpeg');
end

figure(3)
plot(TVec,StateVec(:,3),'LineWidth',2);
xlabel('time(s)')
ylabel('velocity(m/s)')
title('Heave Velocity');
grid on
set(gca, 'FontName', 'Calibri');
set(gca, 'FontSize', 17);   
set(gcf, 'Color', [1, 1, 1])
if (save_msg == 1)
    fpath = 'C:\Users\sdivi\Documents\AUV\Matlab\control test\Dynamic Model\Remus\Plots\';
    filename = [test_msg,'_',case_msg,'_','HeaveVel'];
    saveas(gcf,fullfile(fpath,filename),'jpeg');
end

figure(4)
plot(TVec,StateVec(:,4),'LineWidth',2);
xlabel('time(s)')
ylabel('p(rad/s)')
title('Roll Rate');
grid on
if (max(abs(StateVec(:,4))) < 0.1)
    ylim([-0.5 0.5])
end
set(gca, 'FontName', 'Calibri');
set(gca, 'FontSize', 17);   
set(gcf, 'Color', [1, 1, 1])
if (save_msg == 1)
    fpath = 'C:\Users\sdivi\Documents\AUV\Matlab\control test\Dynamic Model\Remus\Plots\';
    filename = [test_msg,'_',case_msg,'_','RollRate'];
    saveas(gcf,fullfile(fpath,filename),'jpeg');
end

figure(5)
plot(TVec,StateVec(:,5),'LineWidth',2);
xlabel('time(s)')
ylabel('q(rad/s)')
title('Pitch Rate');
grid on
if (max(abs(StateVec(:,5))) < 0.1)
    ylim([-0.5 0.5])
end
set(gca, 'FontName', 'Calibri');
set(gca, 'FontSize', 17);   
set(gcf, 'Color', [1, 1, 1])
if (save_msg == 1)
    fpath = 'C:\Users\sdivi\Documents\AUV\Matlab\control test\Dynamic Model\Remus\Plots\';
    filename = [test_msg,'_',case_msg,'_','PitchRate'];
    saveas(gcf,fullfile(fpath,filename),'jpeg');
end

figure(6)
plot(TVec,StateVec(:,6),'LineWidth',2);
xlabel('time(s)')
ylabel('yaw(rad/s)')
title('Yaw Rate');
grid on
if (max(abs(StateVec(:,6))) < 0.1)
    ylim([-0.5 0.5])
end
set(gca, 'FontName', 'Calibri');
set(gca, 'FontSize', 17);   
set(gcf, 'Color', [1, 1, 1])
if (save_msg == 1)
    fpath = 'C:\Users\sdivi\Documents\AUV\Matlab\control test\Dynamic Model\Remus\Plots\';
    filename = [test_msg,'_',case_msg,'_','YawRate'];
    saveas(gcf,fullfile(fpath,filename),'jpeg');
end

figure(7)
plot(TVec,StateVec(:,7),'LineWidth',2);
hold on
xlabel('time(s)')
ylabel('distance(m)')
title('X Position');
grid on
set(gca, 'FontName', 'Calibri');
set(gca, 'FontSize', 17);   
set(gcf, 'Color', [1, 1, 1])
if (save_msg == 1)
    fpath = 'C:\Users\sdivi\Documents\AUV\Matlab\control test\Dynamic Model\Remus\Plots\';
    filename = [test_msg,'_',case_msg,'_','XPos'];
    saveas(gcf,fullfile(fpath,filename),'jpeg');
end

figure(8)
plot(TVec,StateVec(:,8),'LineWidth',2);
xlabel('time(s)')
ylabel('distance(m)')
title('Y Position');
grid on
set(gca, 'FontName', 'Calibri');
set(gca, 'FontSize', 17);   
set(gcf, 'Color', [1, 1, 1])
if (save_msg == 1)
    fpath = 'C:\Users\sdivi\Documents\AUV\Matlab\control test\Dynamic Model\Remus\Plots\';
    filename = [test_msg,'_',case_msg,'_','YPos'];
    saveas(gcf,fullfile(fpath,filename),'jpeg');
end

figure(9)
plot(TVec,StateVec(:,9),'LineWidth',2);
xlabel('time(s)')
ylabel('position(m)')
title('Z position');
grid on
set(gca, 'FontName', 'Calibri');
set(gca, 'FontSize', 17);   
set(gcf, 'Color', [1, 1, 1])
if (save_msg == 1)
    fpath = 'C:\Users\sdivi\Documents\AUV\Matlab\control test\Dynamic Model\Remus\Plots\';
    filename = [test_msg,'_',case_msg,'_','ZPos'];
    saveas(gcf,fullfile(fpath,filename),'jpeg');
end

figure(10)
plot(TVec,StateVec(:,10),'LineWidth',2);
xlabel('time(s)')
ylabel('phi(rad)')
title('Roll Angle');
grid on
if (max(abs(StateVec(:,10))) < 0.1)
    ylim([-0.5 0.5])
end
set(gca, 'FontName', 'Calibri');
set(gca, 'FontSize', 17);   
set(gcf, 'Color', [1, 1, 1])
if (save_msg == 1)
    fpath = 'C:\Users\sdivi\Documents\AUV\Matlab\control test\Dynamic Model\Remus\Plots\';
    filename = [test_msg,'_',case_msg,'_','RollAngle'];
    saveas(gcf,fullfile(fpath,filename),'jpeg');
end

figure(11)
plot(TVec,StateVec(:,11),'LineWidth',2);
xlabel('time(s)')
ylabel('theta(rad)')
title('Pitch Angle');
grid on
if (max(abs(StateVec(:,11))) < 0.1)
    ylim([-0.5 0.5])
end
set(gca, 'FontName', 'Calibri');
set(gca, 'FontSize', 17);   
set(gcf, 'Color', [1, 1, 1])
if (save_msg == 1)
    fpath = 'C:\Users\sdivi\Documents\AUV\Matlab\control test\Dynamic Model\Remus\Plots\';
    filename = [test_msg,'_',case_msg,'_','PitchAngle'];
    saveas(gcf,fullfile(fpath,filename),'jpeg');
end


figure(12)
plot(TVec,StateVec(:,12),'LineWidth',2);
xlabel('time(s)')
ylabel('psi(rad)')
title('Yaw Angle');
grid on
if (max(abs(StateVec(:,12))) < 0.1)
    ylim([-0.5 0.5])
end
set(gca, 'FontName', 'Calibri');
set(gca, 'FontSize', 17);   
set(gcf, 'Color', [1, 1, 1])
if (save_msg == 1)
    fpath = 'C:\Users\sdivi\Documents\AUV\Matlab\control test\Dynamic Model\Remus\Plots\';
    filename = [test_msg,'_',case_msg,'_','YawAngle'];
    saveas(gcf,fullfile(fpath,filename),'jpeg');
end


figure(13)
plot3(StateVec(:,7),StateVec(:,8),StateVec(:,9),'LineWidth',2);
xlabel('X');
ylabel('Y');
zlabel('Z');
grid on
xlim([0 600])
ylim([-100 100])
zlim([-100 100])
title('Global Position')
set(gca, 'FontName', 'Calibri');
set(gca, 'FontSize', 17);   
set(gcf, 'Color', [1, 1, 1])
if (save_msg == 1)
    fpath = 'C:\Users\sdivi\Documents\AUV\Matlab\control test\Dynamic Model\Remus\Plots\';
    filename = [test_msg,'_',case_msg,'_','GlobalPos'];
    saveas(gcf,fullfile(fpath,filename),'jpeg');
end

% figure(14)
% plot(TVec,StateVec(:,13),'LineWidth',2);
% xlabel('time(s)')
% ylabel('Propeller Speed(rpm)')
% title('Propeller RPM')
% grid on
% set(gca, 'FontName', 'Calibri');
% set(gca, 'FontSize', 17);   
% set(gcf, 'Color', [1, 1, 1])
% if (save_msg == 1)
%     fpath = 'C:\Users\sdivi\Documents\AUV\Matlab\control test\Dynamic Model\Remus\Plots\';
%     filename = [test_msg,'_',case_msg,'_','ControlInput'];
%     saveas(gcf,fullfile(fpath,filename),'jpeg');
% end
% 
% figure(15)
% plot(TVec,StateVec(:,14),'LineWidth',2);
% xlabel('time(s)')
% ylabel('sigma')
% title('sigma variable')
% grid on
% set(gca, 'FontName', 'Calibri');
% set(gca, 'FontSize', 17);   
% set(gcf, 'Color', [1, 1, 1])
