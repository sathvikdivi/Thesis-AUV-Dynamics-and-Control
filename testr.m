clear
clc
close all

% init = [50,100];

init = [-50,50];


tspan = 0:0.1:200;
global eta K;
eta = 1.0;
K = 0;


for i = 1:length(init)
    [t,s(:,i)] = ode45('dtestr',tspan,init(i));
end

clrs = [0 0 0;1 0 0];

figure(1)
for i = 1:length(init)
    plot(t,s(:,i),'Color',clrs(i,:),'LineWidth',2);
    hold on
    leg(i) = cellstr(['\sigma_{init} = ',num2str(init(i))]);
end
xlabel('t')
ylabel('\sigma')
legend(leg,'location','southeast');
grid on
title('Sliding Mode Response - Example')
set(gca, 'FontName', 'Calibri');
set(gca, 'FontSize', 17);   
set(gcf, 'Color', [1, 1, 1])