function [delS,sigma] = DivingController(Adive,Bdive,delFdive,states,desired)

etaDive = 2.5;
phiDive = 0.01;

w = states(1);  w_d = desired(1); wdot_d = desired(5);
q = states(2); q_d = desired(2); qdot_d = desired(6);
theta = states(3);  theta_d = desired(3); thetadot_d = desired(7);
z = states(4); z_d = desired(4); zdot_d = desired(8); 

% poles = [0,-0.5,-0.6,-0.9];
% poles = [0,-0.7,-1.0,-1.25];
poles = [0,-0.4,-0.42,-0.45];
% poles = [0,-0.2,-0.25,-0.4];
gains = transpose(place(Adive,Bdive,poles));
Ac = Adive - Bdive*transpose(gains);
[V,D] = eig(transpose(Ac));
temp = round(diag(D),3);
ind = find(temp == 0);
h = V(:,ind);
h = 10*h;
beta0 = transpose(h)*Bdive;

sigma = transpose(h)*[w - w_d;q - q_d;theta - theta_d;z - z_d];

if abs(z - z_d)/z_d < 0.01
    delS = 0.005*(-gains(1)*w - gains(2)*q - gains(4)*z + 1/beta0*(transpose(h)*[wdot_d;qdot_d;thetadot_d;zdot_d] - transpose(h)*delFdive - etaDive*tanh(sigma/phiDive)));
else
    delS = (-gains(1)*w - gains(2)*q - gains(4)*z + 1/beta0*(transpose(h)*[wdot_d;qdot_d;thetadot_d;zdot_d] - transpose(h)*delFdive - etaDive*tanh(sigma/phiDive)));
end

if abs(delS) > 0.35
    if delS >= 0
        sign = 1;
    else
        sign = -1;
    end
    delS = sign*0.35;
end

% if (abs(z - z_d)/z_d < 1e-2)
%         delS = 0;
% % end
% else
%     if abs(z - z_d)/z_d < 05e-2
%         if z_d >= 0
%             sign = 1;
%         else
%             sign = -1;
%         end
%             delS = sign*0.1;
%     end
% end

a = 1;

end