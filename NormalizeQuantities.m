function [X_norm,Y_norm,Z_norm,K_norm,M_norm,N_norm,RemusProps] = NormalizeQuantities()

global var1 ; 

[RemusMmat,RemusProps,X,Y,Z,K,M,N] = RemusAUV();

Props = RemusProps;

rho = Props.rho; % Remus data
L = Props.L; % Remus data
D = Props.D; % Remus data
LDRatio = L/D ;

Norm3 = 0.5*rho*L^3;
Norm4 = 0.5*rho*L^4;
Norm5 = 0.5*rho*L^5;

Xuu_norm = X.Xuu/Norm3;
Xud_norm = X.Xud/Norm3;
Xvd_norm = X.Xvd/Norm3;
Xwd_norm = X.Xwd/Norm3;
Xpd_norm = X.Xpd/Norm4;
Xqd_norm = X.Xqd/Norm4;
Xrd_norm = X.Xrd/Norm4;
Xvr_norm = X.Xvr/Norm3;
Xrr_norm = X.Xrr/Norm4;
Xwq_norm = X.Xwq/Norm3;
Xqq_norm = X.Xqq/Norm4;

X_norm = struct('Xud_norm',Xud_norm,'Xvd_norm',Xvd_norm,'Xwd_norm',Xwd_norm,...
           'Xpd_norm',Xpd_norm,'Xqd_norm',Xqd_norm,'Xrd_norm',Xrd_norm,...
           'Xvr_norm',Xvr_norm,'Xrr_norm',Xrr_norm,'Xqq_norm',Xqq_norm,...
           'Xwq_norm',Xwq_norm,'Xuu_norm',Xuu_norm);

Yud_norm = Y.Yud/Norm3;
Yvd_norm = Y.Yvd/Norm3;
Ywd_norm = Y.Ywd/Norm3;
Ypd_norm = Y.Ypd/Norm4;
Yqd_norm = Y.Yqd/Norm4;
Yrd_norm = Y.Yrd/Norm4;
Yuv_norm = Y.Yuv/Norm3;
Yvv_norm = Y.Yvv/Norm3;
Yrr_norm = Y.Yrr/Norm4;
Yur_norm = Y.Yur/Norm3;
Ywp_norm = Y.Ywp/Norm3;
Ypq_norm = Y.Ypq/Norm4;
Ydr_norm = Y.Ydr/Norm3;

Y_norm = struct('Yud_norm',Yud_norm,'Yvd_norm',Yvd_norm,'Ywd_norm',Ywd_norm,...
           'Ypd_norm',Ypd_norm,'Yqd_norm',Yqd_norm,'Yrd_norm',Yrd_norm,...
           'Yvv_norm',Yvv_norm,'Yrr_norm',Yrr_norm,'Yur_norm',Yur_norm,...
           'Ywp_norm',Ywp_norm,'Ypq_norm',Ypq_norm,'Yuv_norm',Yuv_norm,...
           'Ydr_norm',Ydr_norm);

Zud_norm = Z.Zud/Norm3;
Zvd_norm = Z.Zvd/Norm3;
Zwd_norm = Z.Zwd/Norm3;
Zpd_norm = Z.Zpd/Norm4;
Zqd_norm = Z.Zqd/Norm4;
Zrd_norm = Z.Zrd/Norm4;
Zuq_norm = Z.Zuq/Norm3;
Zww_norm = Z.Zww/Norm3;
Zqq_norm = Z.Zqq/Norm4;
Zuw_norm = Z.Zuw/Norm3;
Zvp_norm = Z.Zvp/Norm3;
Zrp_norm = Z.Zrp/Norm4;
Zds_norm = Z.Zds/Norm3;

Z_norm = struct('Zud_norm',Zud_norm,'Zvd_norm',Zvd_norm,'Zwd_norm',Zwd_norm,...
           'Zpd_norm',Zpd_norm,'Zqd_norm',Zqd_norm,'Zrd_norm',Zrd_norm,...
           'Zww_norm',Zww_norm,'Zqq_norm',Zqq_norm,'Zuq_norm',Zuq_norm,...
           'Zvp_norm',Zvp_norm,'Zrp_norm',Zrp_norm,'Zuw_norm',Zuw_norm,...
           'Zds_norm',Zds_norm);

Kpd_norm = K.Kpd/Norm5;
Kud_norm = K.Kud/Norm4;
Kvd_norm = K.Kvd/Norm4;
Kwd_norm = K.Kwd/Norm4;
Kqd_norm = K.Kqd/Norm5;
Krd_norm = K.Krd/Norm5;
Kpp_norm = 0;

K_norm = struct('Kud_norm',Kud_norm,'Kvd_norm',Kvd_norm,'Kwd_norm',Kwd_norm,...
           'Kpd_norm',Kpd_norm,'Kqd_norm',Kqd_norm,'Krd_norm',Krd_norm,...
           'Kpp_norm',Kpp_norm);

Mud_norm = M.Mud/Norm4;
Mvd_norm = M.Mvd/Norm4;
Mwd_norm = M.Mwd/Norm4;
Mpd_norm = M.Mpd/Norm5;
Mqd_norm = M.Mqd/Norm5;
Mrd_norm = M.Mrd/Norm5;
Mww_norm = M.Mww/Norm4;
Mqq_norm = M.Mqq/Norm5;
Muw_norm = M.Muw/Norm4;
Muq_norm = M.Muq/Norm4;
Mvp_norm = M.Mvp/Norm4;
Mrp_norm = M.Mrp/Norm5;
Mds_norm = M.Mds/Norm4;

M_norm = struct('Mud_norm',Mud_norm,'Mvd_norm',Mvd_norm,'Mwd_norm',Mwd_norm,...
           'Mpd_norm',Mpd_norm,'Mqd_norm',Mqd_norm,'Mrd_norm',Mrd_norm,...
           'Mww_norm',Mww_norm,'Mqq_norm',Mqq_norm,'Muq_norm',Muq_norm,...
           'Muw_norm',Muw_norm,'Mvp_norm',Mvp_norm,'Mrp_norm',Mrp_norm,...
           'Mds_norm',Mds_norm);

Nud_norm = N.Nud/Norm4;
Nvd_norm = N.Nvd/Norm4;
Nwd_norm = N.Nwd/Norm4;
Npd_norm = N.Npd/Norm5;
Nqd_norm = N.Nqd/Norm5;
Nrd_norm = N.Nrd/Norm5;
Nvv_norm = N.Nvv/Norm4;
Nrr_norm = N.Nrr/Norm5;
Nuv_norm = N.Nuv/Norm4;
Nur_norm = N.Nur/Norm4;
Nwp_norm = N.Nwp/Norm4;
Npq_norm = N.Npq/Norm5;
Ndr_norm = N.Ndr/Norm4;

N_norm = struct('Nud_norm',Nud_norm,'Nvd_norm',Nvd_norm,'Nwd_norm',Nwd_norm,...
           'Npd_norm',Npd_norm,'Nqd_norm',Nqd_norm,'Nrd_norm',Nrd_norm,...
           'Nrr_norm',Nrr_norm,'Nvv_norm',Nvv_norm,'Nuv_norm',Nuv_norm,...
           'Nur_norm',Nur_norm,'Nwp_norm',Nwp_norm,'Npq_norm',Npq_norm,...
           'Ndr_norm',Ndr_norm);

end