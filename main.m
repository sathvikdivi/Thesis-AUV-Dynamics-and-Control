
clear
clc
close all

global Mmat Props X Y Z K M N inputs;
global var1;

var1 = 1;

save_msg = 0; test_msg = 'PositiveB_1en5pcnt'; case_msg = 'RollStab';

x = 0; y = 0; z = 0;
phi = 0; theta = 0; psi = 0;

u = 0; v = 0; w = 0;
p = 0; q = 0; r = 0;

props = 0; delR = 0; delS = 0;
Xprops = 4.0;
inputs = [Xprops;delR;delS];

[Mmat,Props,X,Y,Z,K,M,N] = RemusAUV();

told = 0;
tend = 500;
tspan = told:1:tend;

init = [u,v,w,p,q,r,x,y,z,phi,theta,psi];

StateVec = [init];
TVec = told;

while (told <= tend)
    told
    tspan = told:0.1:told + 1;
    [t,states] = ode45('odefunc',tspan,init);
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
% if (max(abs(StateVec(:,1))) < 0.01)
%     ylim([-1 1]);
% end
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
% if (max(abs(StateVec(:,2))) < 0.01)
%     ylim([-1 1]);
% end
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
% if (max(abs(StateVec(:,3))) < 0.01)
%     ylim([-1 1]);
% end
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
limfactor = max([max(abs(StateVec(:,7))),max(abs(StateVec(:,8))),max(abs(StateVec(:,9)))]);
xlim([-limfactor limfactor])
ylim([-limfactor limfactor])
zlim([-limfactor limfactor])
title('Global Position')
set(gca, 'FontName', 'Calibri');
set(gca, 'FontSize', 17);   
set(gcf, 'Color', [1, 1, 1])
if (save_msg == 1)
    fpath = 'C:\Users\sdivi\Documents\AUV\Matlab\control test\Dynamic Model\Remus\Plots\';
    filename = [test_msg,'_',case_msg,'_','GlobalPos'];
    saveas(gcf,fullfile(fpath,filename),'jpeg');
end




