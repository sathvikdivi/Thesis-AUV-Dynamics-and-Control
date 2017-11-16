clear
clc
close all

desired_depth = 150;

global Mmat Props X Y Z K M N inputs;

save_msg = 0; test_msg = 'Control'; case_msg = 'Dive';

for i = 1:length(desired_depth)

    x = 0; y = 0; z = 0;
    phi = 0; theta = 0; psi = 0;

    u = 0; v = 0; w = 0;
    p = 0; q = 0; r = 0;

    u_d = 1.5; udot_d = 0; x_d = 0;
    w_d = 0; q_d = 0; theta_d = 0; z_d = desired_depth(i);
    wdot_d = 0; qdot_d = 0; thetadot_d = 0; zdot_d = 0;

    told = 0;
    tend = 1500;
    tspan = told:1:tend;

    init = [u,v,w,p,q,r,x,y,z,phi,theta,psi];

    Xprops = 2700.00;
    props = sqrt(abs(Xprops/(4.4444*1e-6))); 
    delR = 0.0; delS = 0;
    inputs = [Xprops;delR;delS];

    [Mmat,Props,X,Y,Z,K,M,N] = AUV();

    StateVec{i} = [init,delS,0,props];
    TVec{i} = told;

    while (told <= tend)
        told
        tnew = told + 1;
        tspan = told:0.1:told + 1;
        props = sqrt(abs(Xprops/(4.4444*1e-6)));
        [Adive,Bdive,dFdive] = DiveControlMatrices(Z,M,Props,u_d,init);
        [delS,sigmaDive] = DivingController(Adive,Bdive,dFdive,[w,q,theta,z],[w_d,q_d,theta_d,z_d,wdot_d,qdot_d,thetadot_d,zdot_d]);
%         delS = senserror(delS,0.01);
        inputs = [Xprops;delR;delS];
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
        StateVec{i} = vertcat(StateVec{i},[init,delS,sigmaDive,props]);
        TVec{i} = vertcat(TVec{i},told);
    end
    
end

lines = {'-','--','-.'};
clrs = [0 0 0;0 0.7 1;0.5 0 0.5;1 0 0;0 1 1;1 0 0;1 0.5 0.5;1 0.5 0;0.5 0.5 0.5;0.5 0.5 1];


figure(1)
for i = 1:length(desired_depth)
    plot(TVec{i},StateVec{i}(:,1),'Color',clrs(i,:),'LineWidth',2);
    hold on
    leg(i) = cellstr(['depth = ',num2str(desired_depth(i)),' m']);
end
xlabel('time(s)')
ylabel('velocity(m/s)')
title('Surge Velocity');
legend(leg,'location','southeast');
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
for i = 1:length(desired_depth)
    plot(TVec{i},StateVec{i}(:,2),'Color',clrs(i,:),'LineWidth',2);
    hold on
    leg(i) = cellstr(['depth = ',num2str(desired_depth(i)),' m']);
end
xlabel('time(s)')
ylabel('velocity(m/s)')
title('Sway Velocity');
legend(leg,'location','southeast');
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
for i = 1:length(desired_depth)
    plot(TVec{i},StateVec{i}(:,3),'Color',clrs(i,:),'LineWidth',2);
    hold on
    leg(i) = cellstr(['depth = ',num2str(desired_depth(i)),' m']);
end
xlabel('time(s)')
ylabel('velocity(m/s)')
title('Heave Velocity');
legend(leg,'location','southeast');
grid on
ylim([-0.5 0.5])
set(gca, 'FontName', 'Calibri');
set(gca, 'FontSize', 17);   
set(gcf, 'Color', [1, 1, 1])
if (save_msg == 1)
    fpath = 'C:\Users\sdivi\Documents\AUV\Matlab\control test\Dynamic Model\Remus\Plots\';
    filename = [test_msg,'_',case_msg,'_','HeaveVel'];
    saveas(gcf,fullfile(fpath,filename),'jpeg');
end

figure(4)
for i = 1:length(desired_depth)
    plot(TVec{i},StateVec{i}(:,4),'Color',clrs(i,:),'LineWidth',2);
    hold on
    leg(i) = cellstr(['depth = ',num2str(desired_depth(i)),' m']);
end
xlabel('time(s)')
ylabel('p(rad/s)')
title('Roll Rate');
legend(leg,'location','southeast');
grid on
if (max(abs(StateVec{i}(:,4))) < 0.1)
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
for i = 1:length(desired_depth)
    plot(TVec{i},StateVec{i}(:,5),'Color',clrs(i,:),'LineWidth',2);
    hold on
    leg(i) = cellstr(['depth = ',num2str(desired_depth(i)),' m']);
end
xlabel('time(s)')
ylabel('q(rad/s)')
title('Pitch Rate');
legend(leg,'location','southeast');
grid on
if (max(abs(StateVec{i}(:,5))) < 0.2)
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
for i = 1:length(desired_depth)
    plot(TVec{i},StateVec{i}(:,6),'Color',clrs(i,:),'LineWidth',2);
    hold on
    leg(i) = cellstr(['depth = ',num2str(desired_depth(i)),' deg']);
end
xlabel('time(s)')
ylabel('yaw(rad/s)')
title('Yaw Rate');
legend(leg,'location','southeast');
grid on
if (max(abs(StateVec{i}(:,6))) < 0.1)
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
for i = 1:length(desired_depth)
    plot(TVec{i},StateVec{i}(:,7),'Color',clrs(i,:),'LineWidth',2);
    hold on
    leg(i) = cellstr(['depth = ',num2str(desired_depth(i)),' deg']);
end
hold on
xlabel('time(s)')
ylabel('distance(m)')
title('X Position');
legend(leg,'location','northwest');
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
for i = 1:length(desired_depth)
    plot(TVec{i},StateVec{i}(:,8),'Color',clrs(i,:),'LineWidth',2);
    hold on
    leg(i) = cellstr(['depth = ',num2str(desired_depth(i)),' deg']);
end
xlabel('time(s)')
ylabel('distance(m)')
title('Y Position');
legend(leg,'location','northwest');
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
for i = 1:length(desired_depth)
    plot(TVec{i},StateVec{i}(:,9),'Color',clrs(i,:),'LineWidth',2);
    hold on
    leg(i) = cellstr(['depth = ',num2str(desired_depth(i)),' deg']);
end
xlabel('time(s)')
ylabel('depth (m)')
title('Depth');
legend(leg,'location','northeast');
grid on
h = gca;
set(h,'YDir','reverse');
set(gca, 'FontName', 'Calibri');
set(gca, 'FontSize', 17);   
set(gcf, 'Color', [1, 1, 1])
if (save_msg == 1)
    fpath = 'C:\Users\sdivi\Documents\AUV\Matlab\control test\Dynamic Model\Remus\Plots\';
    filename = [test_msg,'_',case_msg,'_','ZPos'];
    saveas(gcf,fullfile(fpath,filename),'jpeg');
end

figure(10)
for i = 1:length(desired_depth)
    plot(TVec{i},rad2deg(StateVec{i}(:,10)),'Color',clrs(i,:),'LineWidth',2);
    hold on
    leg(i) = cellstr(['depth = ',num2str(desired_depth(i)),' deg']);
end
xlabel('time(s)')
ylabel('\phi(deg)')
title('Roll Angle');
legend(leg,'location','southeast');
grid on
if (max(abs(StateVec{i}(:,10))) < 0.1)
    ylim([-rad2deg(0.5) rad2deg(0.5)])
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
for i = 1:length(desired_depth)
    plot(TVec{i},rad2deg(StateVec{i}(:,11)),'Color',clrs(i,:),'LineWidth',2);
    hold on
    leg(i) = cellstr(['depth = ',num2str(desired_depth(i)),' deg']);
end
xlabel('time(s)')
ylabel('\theta(deg)')
title('Pitch Angle');
legend(leg,'location','southeast');
grid on
if (max(abs(StateVec{i}(:,11))) < 0.1)
    ylim([-rad2deg(0.5) rad2deg(0.5)])
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
for i = 1:length(desired_depth)
    plot(TVec{i},rad2deg(StateVec{i}(:,12)),'Color',clrs(i,:),'LineWidth',2);
    hold on
    leg(i) = cellstr(['depth = ',num2str(desired_depth(i)),' deg']);
end
xlabel('time(s)')
ylabel('\psi(deg)')
title('Yaw Angle');
legend(leg,'location','southeast');
grid on
if (max(abs(StateVec{i}(:,12))) < 0.1)
    ylim([rad2deg(-0.5) rad2deg(0.5)])
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
for i = 1:length(desired_depth)
    plot3(StateVec{i}(:,7),StateVec{i}(:,8),StateVec{i}(:,9),'Color',clrs(i,:),'LineWidth',2);
    hold on
    leg(i) = cellstr(['depth = ',num2str(desired_depth(i)),' deg']);
end
xlabel('X');
ylabel('Y');
zlabel('Z');
grid on
limfactor = max([max(abs(StateVec{i}(:,7))),max(abs(StateVec{i}(:,8))),max(abs(StateVec{i}(:,9)))]);
xlim([-(limfactor+100) (limfactor+100)])
ylim([-(limfactor+100) (limfactor+100)])
zlim([-(limfactor+100) (limfactor+100)])
title('Global Position')
plot3(StateVec{i}(1,7),StateVec{i}(1,8),StateVec{i}(1,9),'kO');
text(StateVec{i}(1,7),StateVec{i}(1,8),StateVec{i}(1,9),['(',num2str(round(StateVec{i}(1,7),2)),',',num2str(round(StateVec{i}(1,8),2)),',',num2str(round(StateVec{i}(1,9),2)),')']);
for i = 1:length(desired_depth)
    plot3(StateVec{i}(end,7),StateVec{i}(end,8),StateVec{i}(end,9),'kO');
    hold on
    text(StateVec{i}(end,7),StateVec{i}(end,8),StateVec{i}(end,9),['(',num2str(round(StateVec{i}(end,7),2)),',',num2str(round(StateVec{i}(end,8),2)),',',num2str(round(StateVec{i}(end,9),2)),')'])
end
legend(leg,'location','southeast');
set(gca, 'FontName', 'Calibri');
set(gca, 'FontSize', 17);   
set(gcf, 'Color', [1, 1, 1])
if (save_msg == 1)
    fpath = 'C:\Users\sdivi\Documents\AUV\Matlab\control test\Dynamic Model\Remus\Plots\';
    filename = [test_msg,'_',case_msg,'_','GlobalPos'];
    saveas(gcf,fullfile(fpath,filename),'jpeg');
end


figure(14)
for i = 1:length(desired_depth)
    plot(TVec{i},rad2deg(StateVec{i}(:,13)),'Color',clrs(i,:),'LineWidth',2);
    hold on
    leg(i) = cellstr(['depth = ',num2str(desired_depth(i)),' deg']);
end
xlabel('time(s)')
ylabel('\delta_{S}(deg)')
title('Stern Angle')
% ylim([-30 30]);
grid on
legend(leg,'location','northeast');
set(gca, 'FontName', 'Calibri');
set(gca, 'FontSize', 17);   
set(gcf, 'Color', [1, 1, 1])
if (save_msg == 1)
    fpath = 'C:\Users\sdivi\Documents\AUV\Matlab\control test\Dynamic Model\Remus\Plots\';
    filename = [test_msg,'_',case_msg,'_','DiveControlInput'];
    saveas(gcf,fullfile(fpath,filename),'jpeg');
end

figure(15)
for i = 1:length(desired_depth)
    plot(TVec{i},StateVec{i}(:,14),'LineWidth',2);
    hold on
    leg(i) = cellstr(['depth = ',num2str(desired_depth(i)),' deg']);
end
xlabel('time(s)')
ylabel('sigma')
grid on
legend(leg,'location','southeast')
% if (abs(max(StateVec{i}(:,14))) < 0.1)
%     ylim([-1,1]);
% end
title('sigma variable - dive')
set(gca, 'FontName', 'Calibri');
set(gca, 'FontSize', 17);   
set(gcf, 'Color', [1, 1, 1])
if (save_msg == 1)
    fpath = 'C:\Users\sdivi\Documents\AUV\Matlab\control test\Dynamic Model\Remus\Plots\';
    filename = [test_msg,'_',case_msg,'_','SteerSigma'];
    saveas(gcf,fullfile(fpath,filename),'jpeg');
end

figure(16)
for i = 1:length(desired_depth)
    plot(TVec{i},StateVec{i}(:,15),'Color',clrs(i,:),'LineWidth',2);
    hold on
    leg(i) = cellstr(['depth = ',num2str(desired_depth(i)),' deg']);
end
xlabel('time(s)')
ylabel('Propeller Speed(rpm)')
title('Propeller Input')
grid on
legend(leg,'location','southeast');
set(gca, 'FontName', 'Calibri');
set(gca, 'FontSize', 17);   
set(gcf, 'Color', [1, 1, 1])
if (save_msg == 1)
    fpath = 'C:\Users\sdivi\Documents\AUV\Matlab\control test\Dynamic Model\Remus\Plots\';
    filename = [test_msg,'_',case_msg,'_','PropellerInput'];
    saveas(gcf,fullfile(fpath,filename),'jpeg');
end

figure(17)
for i = 1:length(desired_depth)
    plot(StateVec{i}(:,7),StateVec{i}(:,8),'Color',clrs(i,:),'LineWidth',2);
    hold on
    leg(i) = cellstr(['depth = ',num2str(desired_depth(i)),' deg']);
end
xlabel('X')
ylabel('Y')
title('XY Plane')
grid on
legend(leg,'location','northeast');
set(gca, 'FontName', 'Calibri');
set(gca, 'FontSize', 17);   
set(gcf, 'Color', [1, 1, 1])
if (save_msg == 1)
    fpath = 'C:\Users\sdivi\Documents\AUV\Matlab\control test\Dynamic Model\Remus\Plots\';
    filename = [test_msg,'_',case_msg,'_','XYMap'];
    saveas(gcf,fullfile(fpath,filename),'jpeg');
end


figure(18)
for i = 1:length(desired_depth)
    plot(StateVec{i}(:,7),StateVec{i}(:,9),'Color',clrs(i,:),'LineWidth',2);
    hold on
    leg(i) = cellstr(['depth = ',num2str(desired_depth(i)),' deg']);
end
xlabel('X')
ylabel('Z')
title('XZ Plane')
grid on
h = gca;
set(h,'YDir','reverse');
legend(leg,'location','northeast');
set(gca, 'FontName', 'Calibri');
set(gca, 'FontSize', 17);   
set(gcf, 'Color', [1, 1, 1])
if (save_msg == 1)
    fpath = 'C:\Users\sdivi\Documents\AUV\Matlab\control test\Dynamic Model\Remus\Plots\';
    filename = [test_msg,'_',case_msg,'_','XZMap'];
    saveas(gcf,fullfile(fpath,filename),'jpeg');
end