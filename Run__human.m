clc
clearvars
%% basic parameters and initial conditions
dt = 5e-3; % in seconds
tEnd = 1.; % in seconds

thetaSH_0 = 1.2;
omegaSH_0 = 0;
thetaEL_0 = 1.2;
omegaEL_0 = 0;
a_0 = zeros(6,1);

armDamping = [0;0];
numberOfStationarySteps = 20;

Fpert = [0;0];
targetPos_rel = [0.3;0]; % relative to initial hand position

hmn = Human(dt, tEnd, thetaEL_0, omegaEL_0, thetaSH_0, omegaSH_0, ...
            a_0, targetPos_rel, armDamping, numberOfStationarySteps, Fpert,'synergies.mat');

[ofcResult,mskResults] = hmn.simulateHuman(0);

%%
Y_msk = mskResults.Ydata;
U_msk = mskResults.Udata;

nStep = hmn.generalParamSet.nStep;

figure(200); clf
subplot(2,2,1)
plot(1:nStep,hmn.getOutputs_msk(Y_msk).q,'LineWidth',2)
title('joint angles')
legend('elbow','shoulder')

subplot(2,2,2); 
plot(1:nStep,hmn.getOutputs_msk(Y_msk).hand_p,'LineWidth',2)
hold all
plot(1:nStep,hmn.getOutputs_msk(Y_msk).hand_v,'LineWidth',2)
title("task space")
legend('x','y','v_x','v_y','Location','best')

subplot(2,2,3); 
plot(1:nStep-1,U_msk,'LineWidth',2)
title('muscle activations')
legend('ext bi','flx bi','ext elb','flx elb','ext shd','flx shd','NumColumns',3,'location','best')

subplot(2,2,4);
plot(1:nStep,hmn.getOutputs_msk(Y_msk).muscleF,'LineWidth',2)
 title('muscle force')

X_ofc = ofcResult.Xdata;
Xest_ofc = ofcResult.XEstdata;
targetPos_abs = hmn.generalParamSet.targetPos_abs;

figure(300); clf
subplot(2,1,1); hold all
plot(X_ofc(:,1:2)+targetPos_abs','LineWidth',2)
plot(X_ofc(:,3:4),'LineWidth',2)
plot(Xest_ofc(:,1:2)+targetPos_abs','--')
plot(Xest_ofc(:,3:4),'--')
title("task space")

subplot(2,1,2); hold all
plot(X_ofc(:,5:6),'LineWidth',2)
plot(Xest_ofc(:,5:6),'--')
title("OFC muscle activation")