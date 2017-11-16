function [dstates] = odefunc(t,states)

dstates = zeros(size(states,1),1);

% states(1) = 1.5;

global X Y Z K M N Props inputs Mmat;

velocities = [states(1:6)]; % velocities measured in the body frame
angles = [states(10:12)]; % orientation measured in the inertial frame

Fext = ExternalAppliedForces(X,Y,Z,K,M,N,Props,velocities,angles,inputs);
FB = BodyForce(Props,velocities);
Xprops = inputs(1); delR = inputs(2); delS = inputs(3); Kprops = 0;
Fctrl = [Xprops;Y.Ydr*delR*states(1)^2;Z.Zds*delS*states(1)^2;Kprops;M.Mds*delS*states(1)^2;N.Ndr*delR*states(1)^2];

dstates(1:6)  = inv(Mmat)*(Fext - FB + Fctrl);
dstates(7:9) = OCBMatrix(angles)*[states(1);states(2);states(3)];
dstates(10:12) = JMatrix(angles)*[states(4);states(5);states(6)];

% dstates(1) = 0;
dstates(find(abs(dstates) < 1e-6)) = 0;

end