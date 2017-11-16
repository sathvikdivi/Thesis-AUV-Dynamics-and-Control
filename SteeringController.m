function [delR,sigma] = SteeringController(Asteer,Bsteer,delFsteer,states,desired)

etaSteer = 0.5;
phiSteer = 0.4;

v = states(1);  v_d = desired(1); vdot_d = desired(4);
r = states(2); r_d = desired(2); rdot_d = desired(5);
psi = states(3);  psi_d = desired(3); psidot_d = desired(6);

poles = [0,-0.4,-0.45];
gains = transpose(place(Asteer,Bsteer,poles));
Ac = Asteer - Bsteer*transpose(gains);
[V,D] = eig(transpose(Ac));
temp = round(diag(D),3);
ind = find(temp == 0);
h = V(:,ind);
beta0 = transpose(h)*Bsteer;

sigma = transpose(h)*[v - v_d;r - r_d;psi - psi_d];

delR = -gains(1)*v - gains(2)*r + 1/beta0*(transpose(h)*[vdot_d;rdot_d;psidot_d] - transpose(h)*delFsteer - etaSteer*tanh(sigma/phiSteer));

if abs(delR) > 0.35
    if delR >= 0
        sign = 1;
    else
        sign = -1;
    end
    delR = sign*0.35;
end

a = 1;

end