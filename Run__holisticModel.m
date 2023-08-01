clc
clearvars
%%
hf10 = figure(10); clf
ha10 = axes(hf10);
axis equal
hold(ha10,"on")


%% basic parameters and initial conditions
dt = 5e-3; % in seconds
tEnd = 1.; % in seconds

thetaSH_0 = 1.2;
thetaEL_0 = 1.2;

sys = getParametersPlanarArm;
hand_0 = getHandPosition(thetaSH_0,thetaEL_0,sys);

targetPos = hand_0 + [0.3;0];
scatter(ha10, targetPos(1),targetPos(2),500, 'filled')


Fpert = 0*[0; -10]; % the perturbation force

%% setup synergies
load("synergies.mat")
Nmuscle = 6;
NoSyn = size(synergySet,2);

%% setup MSK model
param =  [0; 0]; % [Elbow damping; Shoulder damping]

[IC, Y] = getPlanarArmIC(thetaEL_0, 0, thetaSH_0, 0, zeros(6,1), param);

%% setup ofc
numberOfStationarySteps = 20;

startPosition_OFC = hand_0 - targetPos;
f_ofc = Fpert; % OFC's knowledge about pert. could be zero for unlearned pert
[A,B,H,Q,R,simSetting,noiseStructure,xInit,modelParam] = setupOFCmodel_pointMass_effort(startPosition_OFC, dt, tEnd, numberOfStationarySteps,f_ofc);
ofc = OFC(A,B,H,Q,R,simSetting,noiseStructure);
[L,K] = ofc.getOptimalGains;
nStep = simSetting.nStep;


n = ofc.systemEq.numberOfStates;
m = ofc.systemEq.numberOfControls;
p = ofc.systemEq.numberOfOutputs;

h = ofc.simSetting.delay;
A = ofc.systemEq.A;
A_sim = ofc.systemEq.A_sim;
B = ofc.systemEq.B;
H = ofc.systemEq.H;

Q = ofc.costFunction.Q;
R = ofc.costFunction.R;


C = ofc.noiseConstructors.controlDependentConstructor;
D = ofc.noiseConstructors.stateDependentConstructor;
Omega_xi = ofc.noiseConstructors.additiveProcessNoiseCovar;
Omega_omega = ofc.noiseConstructors.sensoryNoiseCovar;
Omega_eta = ofc.noiseConstructors.internalNoiseCovar;



currentX_ofc = repmat(xInit,h+1,1);
currentXEst_ofc = currentX_ofc;
X_ofc = zeros(nStep,n);
X_ofc(1,:) = currentX_ofc';
Xest_ofc = X_ofc;
U_ofc = zeros(nStep-1,m);


X_msk = nan(nStep, length(IC));
X_msk(1,:) = IC';
Y_msk = nan(nStep, length(Y));
Y_msk(1,:) = Y(:,1)';
U_msk = nan(nStep-1, Nmuscle);

for i = 1:nStep-1
    %% Measurements
%     if i>h+1
%         hand_x   = Y_msk(i-h-1,14);
%         hand_y   = Y_msk(i-h-1,12);
%         hand_v_x = Y_msk(i-h-1,11);
%         hand_v_y = Y_msk(i-h-1,9);
%     end
%     hand_x = Y(14);
%     hand_y = Y(12);
%     hand_v_x = Y(11);
%     hand_v_y = Y(9);
% 
%     handPos = [hand_x  ; hand_y] - targetPos;
%     handVel = [hand_v_x; hand_v_y];
    
%     q_SH = Y(4);
%     q_EL = Y(2);
%     qdot_SH = Y(5);
%     qdot_EL = Y(3);
% 
%     Fpert_est = currentXEst_ofc(7:8);

    %% OFC's integral step
    sensoryNoise = mvnrnd(zeros(p,1),Omega_omega)';
    processNoise = mvnrnd(zeros(n,1),Omega_xi)';
    internalNoise = mvnrnd(zeros(n,1),Omega_eta)';
    
    % use this to decouple OFC from MSK. good for OFC debugging
%     stateDependentNoise = 0;
%     for isdn = 1:size(D,3)
%         stateDependentNoise = stateDependentNoise + randn*D(:,:,isdn)*X_ofc(i,:)';
%     end
%     yz = H*currentX_ofc + sensoryNoise + stateDependentNoise;


    U_ofc(i,:) = (-L(:,:,i)*currentXEst_ofc)';

    controlDependentNoise = 0;
    for icdn = 1:m
        controlDependentNoise = controlDependentNoise + randn*C(:,:,icdn)*U_ofc(i,:)';
    end
    
    newX_ofc = A_sim(:,:,i)*currentX_ofc + B*U_ofc(i,:)' + processNoise + controlDependentNoise;
    diff_X_ofc = (newX_ofc - currentX_ofc)/dt;
    currentX_ofc = newX_ofc;
    X_ofc(i+1,:) = currentX_ofc';

%     newXEst_ofc = A(:,:,i)*currentXEst_ofc + B*U_ofc(i,:)' + K(:,:,i)*(yz-H*currentXEst_ofc) + internalNoise;
%     diff_XEst_ofc = (newXEst_ofc - currentXEst_ofc)/dt;
%     currentXEst_ofc = newXEst_ofc;
%     Xest_ofc(i+1,:) = currentXEst_ofc';
    %% Generate high-level commands

    a_ref_ofc = diff_X_ofc(3:4); % this works better
%     a_ref_ofc = diff_XEst_ofc(3:4);
    F_ref_ofc = X_ofc(i,5:6)';
%     a_ref_ofc = U_ofc(i,:)';
    %% MSK's step
    q_SH = getPlanarArmOutputs(Y).q(1); 
    q_EL = getPlanarArmOutputs(Y).q(2);
    qdot_SH = getPlanarArmOutputs(Y).qdot(1);
    qdot_EL = getPlanarArmOutputs(Y).qdot(2);

    Fpert_est = currentXEst_ofc(7:8);

    syn = interpn(1:6,1:NoSyn,thetaSHset,thetaELset,  synergySet  ,1:Nmuscle,1:NoSyn,q_SH,q_EL);
    [basis_acc,basis_F] = calcBasis(syn, q_SH,q_EL,param);

    a_ref_tilde = CorrectAccRef(a_ref_ofc,q_SH,q_EL,qdot_SH,qdot_EL,sys,Fpert_est);

    coeff = lsqnonneg(basis_acc,a_ref_tilde);
%     coeff = lsqnonneg(basis_F,F_ref_ofc);
    u_msk = syn*coeff;
    u_msk = min(1,max(0,u_msk));
    u_msk = u_msk .* (1+0.05*randn(size(u_msk)));
    

    [Xdot, xcv, Y] = PlanarArm(0,X_msk(i,:)',[Fpert(2);Fpert(1);u_msk],param);
    U_msk(i,:) = u_msk';
    Y_msk(i,:) = Y';
    X_msk(i+1,:) = X_msk(i,:) + Xdot' * dt;

    %% Estimator's integration step
    handPos = getPlanarArmOutputs(Y).hand_p - targetPos;
    handVel = getPlanarArmOutputs(Y).hand_v;

    stateDependentNoise = 0;
    for isdn = 1:size(D,3)
        stateDependentNoise = stateDependentNoise + randn*D(:,:,isdn)*X_ofc(i+1,:)';
    end
    yz = [handPos; handVel; currentX_ofc(5:6); Fpert] + sensoryNoise + stateDependentNoise;

    newXEst_ofc = A(:,:,i)*currentXEst_ofc + B*U_ofc(i,:)' + K(:,:,i)*(yz-H*currentXEst_ofc) + internalNoise;
    diff_XEst_ofc = (newXEst_ofc - currentXEst_ofc)/dt;
    currentXEst_ofc = newXEst_ofc;
    Xest_ofc(i+1,:) = currentXEst_ofc';

    %% plot this arm path
    if mod(i,10)==1
        cmap = copper(nStep);
        [hnd, elb] = getHandPosition(q_SH,q_EL,sys);
        plot([0,elb(1),hnd(1)], [0,elb(2),hnd(2)],'-o','LineWidth',2,'Color',cmap(i,:))
        scatter(X_ofc(i,1)+targetPos(1), X_ofc(i,2)+targetPos(2),'s','filled')
        scatter(Xest_ofc(i,1)+targetPos(1), Xest_ofc(i,2)+targetPos(2),100,'x','MarkerEdgeColor','r','LineWidth',2)
    end
     
end

X_ofc = X_ofc(:,1:ofc.systemEq.numberOfOriginalStates);
Xest_ofc = Xest_ofc(:,1:ofc.systemEq.numberOfOriginalStates);


figure(2); clf
subplot(2,2,1)
plot(1:nStep,getPlanarArmOutputs(Y_msk).q,'LineWidth',2)
title('joint angles')
legend('elbow','shoulder')

subplot(2,2,2); 
plot(1:nStep,getPlanarArmOutputs(Y_msk).hand_p,'LineWidth',2)
hold all
plot(1:nStep,getPlanarArmOutputs(Y_msk).hand_v,'LineWidth',2)
title("task space")
legend('x','y','v_x','v_y','Location','best')

subplot(2,2,3); 
plot(1:nStep-1,U_msk,'LineWidth',2)
title('muscle activations')
legend('ext bi','flx bi','ext elb','flx elb','ext shd','flx shd','NumColumns',3,'location','best')

subplot(2,2,4);
plot(1:nStep,getPlanarArmOutputs(Y_msk).muscleF,'LineWidth',2)
 title('muscle force')



figure(3); clf
subplot(2,1,1); hold all
plot(X_ofc(:,1:2)+targetPos','LineWidth',2)
plot(X_ofc(:,3:4),'LineWidth',2)
plot(Xest_ofc(:,1:2)+targetPos','--')
plot(Xest_ofc(:,3:4),'--')
title("task space")

subplot(2,1,2); hold all
plot(X_ofc(:,5:6),'LineWidth',2)
plot(Xest_ofc(:,5:6),'--')
title("OFC muscle activation")



% *    y[ 1] = empty
% *    y[ 2] = `Main.Model1.MuscleArm1.zElbow.theta`(t)
% *    y[ 3] = diff(`Main.Model1.MuscleArm1.zElbow.theta`(t),t)
% *    y[ 4] = `Main.Model1.MuscleArm1.zShoulder.theta`(t)
% *    y[ 5] = diff(`Main.Model1.MuscleArm1.zShoulder.theta`(t),t)
% *    y[ 6] = `Main.Model1.axyz[1]`(t)
% *    y[ 7] = `Main.Model1.axyz[2]`(t)
% *    y[ 8] = `Main.Model1.axyz[3]`(t)
% *    y[ 9] = `Main.Model1.vxyz[1]`(t)
% *    y[10] = `Main.Model1.vxyz[2]`(t)
% *    y[11] = `Main.Model1.vxyz[3]`(t)
% *    y[12] = `Main.Model1.xyz[1]`(t)
% *    y[13] = `Main.Model1.xyz[2]`(t)
% *    y[14] = `Main.Model1.xyz[3]`(t)
% *    y[15] = `Main.Model1.FBiExt`(t)
% *    y[16] = `Main.Model1.FBiFlex`(t)
% *    y[17] = `Main.Model1.FElExt`(t)
% *    y[18] = `Main.Model1.FElFlex`(t)
% *    y[19] = `Main.Model1.FShExt`(t)
% *    y[20] = `Main.Model1.FShFlex`(t)