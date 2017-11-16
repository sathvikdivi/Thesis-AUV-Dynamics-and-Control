clear
clc
close all


desired_heading = [10;30;60;90];

Xprops = 00.00;

global Mmat Props X Y Z K M N inputs;

save_msg = 1; test_msg = 'Control'; case_msg = 'SurgeSteer2'; legcommand = 0;
err = 0.0;
errdR = 0.0;

for i = 1:length(desired_heading)
    
    x = 0; y = 0; z = 0;
    phi = 0; theta = 0; psi = 0;

    u = 0; v = 0; w = 0;
    p = 0; q = 0; r = 0;

    u_d = 1.40; udot_d = 0; x_d = 0; 
    v_d = 0; r_d = 0; psi_d = deg2rad(desired_heading(i));
    vdot_d = 0; rdot_d = 0; psidot_d = 0;

    told = 0;
    tend = 250;
    tspan = told:1:tend;
    
        
    phiSurge = 0.4;
    etaSurge = 0.25; Kd = 0;
    
    init = [u,v,w,p,q,r,x,y,z,phi,theta,psi];
    
    props = sqrt(abs(Xprops/(4.4444*1e-6))); 
    delR = 0.0; delS = 0;
    inputs = [Xprops;delR;delS];

    [Mmat,Props,X,Y,Z,K,M,N] = AUV();

    StateVec{i} = [init,delR,0,Xprops,0];
    TVec{i} = told;

    while (told <= tend)
        told
        tnew = told + 1;
        tspan = told:0.1:told + 1;
        props = sqrt(abs(Xprops/(4.4444*1e-6)));
        sigmaSurge = u - u_d;
        Xprops = (Props.mass - X.Xud)*(udot_d - Kd*sigmaSurge -  etaSurge*tanh(sigmaSurge/phiSurge)) - X.Xuu*abs(u)*u;
        [Asteer,Bsteer,dFsteer] = SteerControlMatrices(Y,N,Props,u_d,init);
        [delR,sigmaSteer] = SteeringController(Asteer,Bsteer,dFsteer,[v,r,psi],[v_d,r_d,psi_d,vdot_d,rdot_d,psidot_d]);
        delR = senserror(delR,errdR);
        inputs = [Xprops;delR;delS];
        [t,states] = ode45('odefunc',tspan,init);
        u = states(end,1);
        u = senserror(u,err);
        v = states(end,2);
        w = states(end,3);
        w = senserror(w,err);
        p = states(end,4);
        p = senserror(p,err);
        q = states(end,5);
        q = senserror(q,err);
        r = states(end,6);
        r = senserror(r,err);
        x = states(end,7);
        y = states(end,8);
        z = states(end,9);
        phi = states(end,10);
        phi = senserror(phi,err);
        theta = states(end,11);
        theta = senserror(theta,err);
        psi = states(end,12);
        psi = senserror(psi,err);
        told = t(end);
        init = [u,v,w,p,q,r,x,y,z,phi,theta,psi];
        StateVec{i} = vertcat(StateVec{i},[init,delR,sigmaSteer,Xprops,sigmaSurge]);
        TVec{i} = vertcat(TVec{i},told);
    end
    
end

lines = {'-','--','-.'};
clrs = [0 0 0;0 0.7 1;0.5 0 0.5;1 0 0;0 1 1;1 0 0;1 0.5 0.5;1 0.5 0;0.5 0.5 0.5;0.5 0.5 1];

udes = u_d*ones(length(TVec{1}),1);

figure(1)
for i = 1:length(desired_heading)
    plot(TVec{i},StateVec{i}(:,1),'Color',clrs(i,:),'LineWidth',2);
    hold on
    leg(i) = cellstr(['\psi_{des} = ',num2str(desired_heading(i)),' deg']);
end
xlabel('time(s)')
ylabel('velocity(m/s)')
title('Surge Velocity');
hleg = legend(leg,'location','southeast');
if legcommand == 1
    htitle = get(hleg,'Title');
    set(htitle,'String','Commanded Heading (deg)')
end
grid on
set(gca, 'FontName', 'Calibri');
set(gca, 'FontSize', 17);   
set(gcf, 'Color', [1, 1, 1])
if (save_msg == 1)
    fpath = 'C:\Users\sdivi\Documents\AUV\Matlab\control test\Dynamic Model\Modified MUTS\Plots\';
    filename = [test_msg,'_',case_msg,'_','SurgeVel'];
    saveas(gcf,fullfile(fpath,filename),'jpeg');
end

figure(2)
for i = 1:length(desired_heading)
    plot(TVec{i},StateVec{i}(:,2),'Color',clrs(i,:),'LineWidth',2);
    hold on
    leg(i) = cellstr(['\psi_{des} = ',num2str(desired_heading(i)),' deg']);
end
xlabel('time(s)')
ylabel('velocity(m/s)')
title('Sway Velocity');
hleg = legend(leg,'location','southeast');
if legcommand == 1
    htitle = get(hleg,'Title');
    set(htitle,'String','Commanded Heading (deg)')
end
grid on
set(gca, 'FontName', 'Calibri');
set(gca, 'FontSize', 17);   
set(gcf, 'Color', [1, 1, 1])
if (save_msg == 1)
    fpath = 'C:\Users\sdivi\Documents\AUV\Matlab\control test\Dynamic Model\Modified MUTS\Plots\';
    filename = [test_msg,'_',case_msg,'_','SwayVel'];
    saveas(gcf,fullfile(fpath,filename),'jpeg');
end

figure(3)
for i = 1:length(desired_heading)
    plot(TVec{i},StateVec{i}(:,3),'Color',clrs(i,:),'LineWidth',2);
    hold on
    leg(i) = cellstr(['\psi_{des} = ',num2str(desired_heading(i)),' deg']);
end
xlabel('time(s)')
ylabel('velocity(m/s)')
title('Heave Velocity');
ylim([-0.5 0.5]);
hleg = legend(leg,'location','southeast');
if legcommand == 1
    htitle = get(hleg,'Title');
    set(htitle,'String','Commanded Heading (deg)')
end
grid on
set(gca, 'FontName', 'Calibri');
set(gca, 'FontSize', 17);   
set(gcf, 'Color', [1, 1, 1])
if (save_msg == 1)
    fpath = 'C:\Users\sdivi\Documents\AUV\Matlab\control test\Dynamic Model\Modified MUTS\Plots\';
    filename = [test_msg,'_',case_msg,'_','HeaveVel'];
    saveas(gcf,fullfile(fpath,filename),'jpeg');
end

figure(4)
for i = 1:length(desired_heading)
    plot(TVec{i},StateVec{i}(:,4),'Color',clrs(i,:),'LineWidth',2);
    hold on
    leg(i) = cellstr(['\psi_{des} = ',num2str(desired_heading(i)),' deg']);
end
xlabel('time(s)')
ylabel('p(rad/s)')
title('Roll Rate');
hleg = legend(leg,'location','southeast');
if legcommand == 1
    htitle = get(hleg,'Title');
    set(htitle,'String','Commanded Heading (deg)')
end
grid on
if (max(abs(StateVec{i}(:,4))) < 0.1)
    ylim([-0.5 0.5])
end
set(gca, 'FontName', 'Calibri');
set(gca, 'FontSize', 17);   
set(gcf, 'Color', [1, 1, 1])
if (save_msg == 1)
    fpath = 'C:\Users\sdivi\Documents\AUV\Matlab\control test\Dynamic Model\Modified MUTS\Plots\';
    filename = [test_msg,'_',case_msg,'_','RollRate'];
    saveas(gcf,fullfile(fpath,filename),'jpeg');
end

figure(5)
for i = 1:length(desired_heading)
    plot(TVec{i},StateVec{i}(:,5),'Color',clrs(i,:),'LineWidth',2);
    hold on
    leg(i) = cellstr(['\psi_{des} = ',num2str(desired_heading(i)),' deg']);
end
xlabel('time(s)')
ylabel('q(rad/s)')
title('Pitch Rate');
hleg = legend(leg,'location','southeast');
if legcommand == 1
    htitle = get(hleg,'Title');
    set(htitle,'String','Commanded Heading (deg)')
end
grid on
if (max(abs(StateVec{i}(:,5))) < 0.1)
    ylim([-0.5 0.5])
end
set(gca, 'FontName', 'Calibri');
set(gca, 'FontSize', 17);   
set(gcf, 'Color', [1, 1, 1])
if (save_msg == 1)
    fpath = 'C:\Users\sdivi\Documents\AUV\Matlab\control test\Dynamic Model\Modified MUTS\Plots\';
    filename = [test_msg,'_',case_msg,'_','PitchRate'];
    saveas(gcf,fullfile(fpath,filename),'jpeg');
end

figure(6)
for i = 1:length(desired_heading)
    plot(TVec{i},StateVec{i}(:,6),'Color',clrs(i,:),'LineWidth',2);
    hold on
    leg(i) = cellstr(['\psi_{des} = ',num2str(desired_heading(i)),' deg']);
end
xlabel('time(s)')
ylabel('yaw(rad/s)')
title('Yaw Rate');
hleg = legend(leg,'location','northeast');
if legcommand == 1
    htitle = get(hleg,'Title');
    set(htitle,'String','Commanded Heading (deg)')
end
grid on
if (max(abs(StateVec{i}(:,6))) < 0.1)
    ylim([-0.5 0.5])
end
set(gca, 'FontName', 'Calibri');
set(gca, 'FontSize', 17);   
set(gcf, 'Color', [1, 1, 1])
if (save_msg == 1)
    fpath = 'C:\Users\sdivi\Documents\AUV\Matlab\control test\Dynamic Model\Modified MUTS\Plots\';
    filename = [test_msg,'_',case_msg,'_','YawRate'];
    saveas(gcf,fullfile(fpath,filename),'jpeg');
end

figure(7)
for i = 1:length(desired_heading)
    plot(TVec{i},StateVec{i}(:,7),'Color',clrs(i,:),'LineWidth',2);
    hold on
    leg(i) = cellstr(['\psi_{des} = ',num2str(desired_heading(i)),' deg']);
end
hold on
xlabel('time(s)')
ylabel('distance(m)')
title('X Position');
hleg = legend(leg,'location','northwest');
grid on
if legcommand == 1
    htitle = get(hleg,'Title');
    set(htitle,'String','Commanded Heading (deg)')
end
set(gca, 'FontName', 'Calibri');
set(gca, 'FontSize', 17);   
set(gcf, 'Color', [1, 1, 1])
if (save_msg == 1)
    fpath = 'C:\Users\sdivi\Documents\AUV\Matlab\control test\Dynamic Model\Modified MUTS\Plots\';
    filename = [test_msg,'_',case_msg,'_','XPos'];
    saveas(gcf,fullfile(fpath,filename),'jpeg');
end

figure(8)
for i = 1:length(desired_heading)
    plot(TVec{i},StateVec{i}(:,8),'Color',clrs(i,:),'LineWidth',2);
    hold on
    leg(i) = cellstr(['\psi_{des} = ',num2str(desired_heading(i)),' deg']);
end
xlabel('time(s)')
ylabel('distance(m)')
title('Y Position');
hleg = legend(leg,'location','northwest');
if legcommand == 1
    htitle = get(hleg,'Title');
    set(htitle,'String','Commanded Heading (deg)')
end
grid on
set(gca, 'FontName', 'Calibri');
set(gca, 'FontSize', 17);   
set(gcf, 'Color', [1, 1, 1])
if (save_msg == 1)
    fpath = 'C:\Users\sdivi\Documents\AUV\Matlab\control test\Dynamic Model\Modified MUTS\Plots\';
    filename = [test_msg,'_',case_msg,'_','YPos'];
    saveas(gcf,fullfile(fpath,filename),'jpeg');
end

figure(9)
for i = 1:length(desired_heading)
    plot(TVec{i},StateVec{i}(:,9),'Color',clrs(i,:),'LineWidth',2);
    hold on
    leg(i) = cellstr(['\psi_{des} = ',num2str(desired_heading(i)),' deg']);
end
xlabel('time(s)')
ylabel('position(m)')
title('Z position');
hleg = legend(leg,'location','northeast');
if legcommand == 1
    htitle = get(hleg,'Title');
    set(htitle,'String','Commanded Heading (deg)')
end
grid on
h = gca;
set(h,'YDir','reverse');
set(gca, 'FontName', 'Calibri');
set(gca, 'FontSize', 17);   
set(gcf, 'Color', [1, 1, 1])
if (save_msg == 1)
    fpath = 'C:\Users\sdivi\Documents\AUV\Matlab\control test\Dynamic Model\Modified MUTS\Plots\';
    filename = [test_msg,'_',case_msg,'_','ZPos'];
    saveas(gcf,fullfile(fpath,filename),'jpeg');
end

figure(10)
for i = 1:length(desired_heading)
    plot(TVec{i},rad2deg(StateVec{i}(:,10)),'Color',clrs(i,:),'LineWidth',2);
    hold on
    leg(i) = cellstr(['\psi_{des} = ',num2str(desired_heading(i)),' deg']);
end
xlabel('time(s)')
ylabel('\phi(deg)')
title('Roll Angle');
hleg = legend(leg,'location','southeast');
if legcommand == 1
    htitle = get(hleg,'Title');
    set(htitle,'String','Commanded Heading (deg)')
end
grid on
if (max(abs(StateVec{i}(:,10))) < 0.1)
    ylim([-rad2deg(0.5) rad2deg(0.5)])
end
set(gca, 'FontName', 'Calibri');
set(gca, 'FontSize', 17);   
set(gcf, 'Color', [1, 1, 1])
if (save_msg == 1)
    fpath = 'C:\Users\sdivi\Documents\AUV\Matlab\control test\Dynamic Model\Modified MUTS\Plots\';
    filename = [test_msg,'_',case_msg,'_','RollAngle'];
    saveas(gcf,fullfile(fpath,filename),'jpeg');
end

figure(11)
for i = 1:length(desired_heading)
    plot(TVec{i},rad2deg(StateVec{i}(:,11)),'Color',clrs(i,:),'LineWidth',2);
    hold on
    leg(i) = cellstr(['\psi_{des} = ',num2str(desired_heading(i)),' deg']);
end
xlabel('time(s)')
ylabel('\theta(deg)')
title('Pitch Angle');
hleg = legend(leg,'location','southeast');
if legcommand == 1
    htitle = get(hleg,'Title');
    set(htitle,'String','Commanded Heading (deg)')
end
grid on
if (max(abs(StateVec{i}(:,11))) < 0.1)
    ylim([-rad2deg(0.5) rad2deg(0.5)])
end
set(gca, 'FontName', 'Calibri');
set(gca, 'FontSize', 17);   
set(gcf, 'Color', [1, 1, 1])
if (save_msg == 1)
    fpath = 'C:\Users\sdivi\Documents\AUV\Matlab\control test\Dynamic Model\Modified MUTS\Plots\';
    filename = [test_msg,'_',case_msg,'_','PitchAngle'];
    saveas(gcf,fullfile(fpath,filename),'jpeg');
end


figure(12)
for i = 1:length(desired_heading)
    plot(TVec{i},rad2deg(StateVec{i}(:,12)),'Color',clrs(i,:),'LineWidth',2);
    hold on
    leg(i) = cellstr(['\psi_{des} = ',num2str(desired_heading(i)),' deg']);
end
xlabel('time(s)')
ylabel('\psi(deg)')
title('Yaw Angle');
hleg = legend(leg,'location','southeast');
if legcommand == 1
    htitle = get(hleg,'Title');
    set(htitle,'String','Commanded Heading (deg)')
end
grid on
if (max(abs(StateVec{i}(:,12))) < 0.1)
    ylim([rad2deg(-0.5) rad2deg(0.5)])
end
set(gca, 'FontName', 'Calibri');
set(gca, 'FontSize', 17);   
set(gcf, 'Color', [1, 1, 1])
if (save_msg == 1)
    fpath = 'C:\Users\sdivi\Documents\AUV\Matlab\control test\Dynamic Model\Modified MUTS\Plots\';
    filename = [test_msg,'_',case_msg,'_','YawAngle'];
    saveas(gcf,fullfile(fpath,filename),'jpeg');
end


figure(13)
for i = 1:length(desired_heading)
    plot3(StateVec{i}(:,7),StateVec{i}(:,8),StateVec{i}(:,9),'Color',clrs(i,:),'LineWidth',2);
    hold on
    leg(i) = cellstr(['\psi_{des} = ',num2str(desired_heading(i)),' deg']);
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
for i = 1:length(desired_heading)
    plot3(StateVec{i}(end,7),StateVec{i}(end,8),StateVec{i}(end,9),'kO');
    hold on
    text(StateVec{i}(end,7),StateVec{i}(end,8),StateVec{i}(end,9),['(',num2str(round(StateVec{i}(end,7),2)),',',num2str(round(StateVec{i}(end,8),2)),',',num2str(round(StateVec{i}(end,9),2)),')'])
end
hleg = legend(leg,'location','southeast');
if legcommand == 1
    htitle = get(hleg,'Title');
    set(htitle,'String','Commanded Heading (deg)')
end
set(gca, 'FontName', 'Calibri');
set(gca, 'FontSize', 17);   
set(gcf, 'Color', [1, 1, 1])
if (save_msg == 1)
    fpath = 'C:\Users\sdivi\Documents\AUV\Matlab\control test\Dynamic Model\Modified MUTS\Plots\';
    filename = [test_msg,'_',case_msg,'_','GlobalPos'];
    saveas(gcf,fullfile(fpath,filename),'jpeg');
end


figure(14)
for i = 1:length(desired_heading)
    plot(TVec{i},rad2deg(StateVec{i}(:,13)),'Color',clrs(i,:),'LineWidth',2);
    hold on
    leg(i) = cellstr(['\psi_{des} = ',num2str(desired_heading(i)),' deg']);
end
xlabel('time(s)')
ylabel('\delta_{R}(deg)')
title('Rudder Angle')
grid on
hleg = legend(leg,'location','southeast');
if legcommand == 1
    htitle = get(hleg,'Title');
    set(htitle,'String','Commanded Heading (deg)')
end
set(gca, 'FontName', 'Calibri');
set(gca, 'FontSize', 17);   
set(gcf, 'Color', [1, 1, 1])
if (save_msg == 1)
    fpath = 'C:\Users\sdivi\Documents\AUV\Matlab\control test\Dynamic Model\Modified MUTS\Plots\';
    filename = [test_msg,'_',case_msg,'_','SteerControlInput'];
    saveas(gcf,fullfile(fpath,filename),'jpeg');
end

figure(15)
for i = 1:length(desired_heading)
    plot(TVec{i},StateVec{i}(:,14),'Color',clrs(i,:),'LineWidth',2);
    hold on
    leg(i) = cellstr(['\psi_{des} = ',num2str(desired_heading(i)),' deg']);
end
xlabel('time(s)')
ylabel('sigma')
grid on
hleg = legend(leg,'location','northeast');
if legcommand == 1
    htitle = get(hleg,'Title');
    set(htitle,'String','Commanded Heading (deg)')
end
if (abs(max(StateVec{i}(:,14))) < 0.1)
    ylim([-1,1]);
end
title('sigma variable - steer')
set(gca, 'FontName', 'Calibri');
set(gca, 'FontSize', 17);   
set(gcf, 'Color', [1, 1, 1])
if (save_msg == 1)
    fpath = 'C:\Users\sdivi\Documents\AUV\Matlab\control test\Dynamic Model\Modified MUTS\Plots\';
    filename = [test_msg,'_',case_msg,'_','SteerSigma'];
    saveas(gcf,fullfile(fpath,filename),'jpeg');
end

figure(16)
for i = 1:length(desired_heading)
    plot(TVec{i},StateVec{i}(:,15),'Color',clrs(i,:),'LineWidth',2);
    hold on
    leg(i) = cellstr(['\psi_{des} = ',num2str(desired_heading(i)),' deg']);
end
xlabel('time(s)')
ylabel('Propeller Force(N)')
title('Propeller Input')
grid on
hleg = legend(leg,'location','southeast');
if legcommand == 1
    htitle = get(hleg,'Title');
    set(htitle,'String','Commanded Heading (deg)')
end
set(gca, 'FontName', 'Calibri');
set(gca, 'FontSize', 17);   
set(gcf, 'Color', [1, 1, 1])
if (save_msg == 1)
    fpath = 'C:\Users\sdivi\Documents\AUV\Matlab\control test\Dynamic Model\Modified MUTS\Plots\';
    filename = [test_msg,'_',case_msg,'_','PropellerInput'];
    saveas(gcf,fullfile(fpath,filename),'jpeg');
end

figure(17)
for i = 1:length(desired_heading)
    plot(StateVec{i}(:,7),StateVec{i}(:,8),'Color',clrs(i,:),'LineWidth',2);
    hold on
    leg(i) = cellstr(['\psi_{des} = ',num2str(desired_heading(i)),' deg']);
end
xlabel('X')
ylabel('Y')
title('XY Plane')
grid on
hleg = legend(leg,'location','northeast');
if legcommand == 1
    htitle = get(hleg,'Title');
    set(htitle,'String','Commanded Heading (deg)')
end
set(gca, 'FontName', 'Calibri');
set(gca, 'FontSize', 17);   
set(gcf, 'Color', [1, 1, 1])
if (save_msg == 1)
    fpath = 'C:\Users\sdivi\Documents\AUV\Matlab\control test\Dynamic Model\Modified MUTS\Plots\';
    filename = [test_msg,'_',case_msg,'_','XYMap'];
    saveas(gcf,fullfile(fpath,filename),'jpeg');
end


figure(18)
for i = 1:length(desired_heading)
    plot(StateVec{i}(:,7),StateVec{i}(:,9),'Color',clrs(i,:),'LineWidth',2);
    hold on
    leg(i) = cellstr(['\psi_{des} = ',num2str(desired_heading(i)),' deg']);
end
xlabel('X')
ylabel('Z')
title('XZ Plane')
ylim([-10 10]);
grid on
h = gca;
set(h,'YDir','reverse');
hleg = legend(leg,'location','northeast');
if legcommand == 1
    htitle = get(hleg,'Title');
    set(htitle,'String','Commanded Heading (deg)')
end
set(gca, 'FontName', 'Calibri');
set(gca, 'FontSize', 17);   
set(gcf, 'Color', [1, 1, 1])
if (save_msg == 1)
    fpath = 'C:\Users\sdivi\Documents\AUV\Matlab\control test\Dynamic Model\Modified MUTS\Plots\';
    filename = [test_msg,'_',case_msg,'_','XZMap'];
    saveas(gcf,fullfile(fpath,filename),'jpeg');
end

figure(19)
for i = 1:length(desired_heading)
    plot(TVec{i},StateVec{i}(:,16),'Color',clrs(i,:),'LineWidth',2);
    hold on
    leg(i) = cellstr(['\psi_{des} = ',num2str(desired_heading(i)),' deg']);
end
xlabel('time(s)')
ylabel('sigma')
grid on
hleg = legend(leg,'location','northeast');
if legcommand == 1
    htitle = get(hleg,'Title');
    set(htitle,'String','Commanded Heading (deg)')
end
if (abs(max(StateVec{i}(:,14))) < 0.1)
    ylim([-1,1]);
end
title('sigma variable - speed')
set(gca, 'FontName', 'Calibri');
set(gca, 'FontSize', 17);   
set(gcf, 'Color', [1, 1, 1])
if (save_msg == 1)
    fpath = 'C:\Users\sdivi\Documents\AUV\Matlab\control test\Dynamic Model\Modified MUTS\Plots\';
    filename = [test_msg,'_',case_msg,'_','SurgeSigma'];
    saveas(gcf,fullfile(fpath,filename),'jpeg');
end