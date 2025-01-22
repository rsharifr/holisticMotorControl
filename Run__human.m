clc
% clearvars
addpath("MATLAB_handy_functions\")
%% basic parameters and initial conditions
dt = 5e-3; % in seconds
tEnd = 1.; % in seconds

thetaSH_0 = 0.5; % 0.2 human-aware control. 0.5 for center out
omegaSH_0 = 0;
thetaEL_0 = 1.9; % 1.5 human-aware control. 1.9 for center out
omegaEL_0 = 0;
a_0 = zeros(6,1);

armDamping = [0;0];
numberOfStationarySteps = 21;

Fpert = [0;0];
% targetPos_rel = [-0.25;0]; % relative to initial hand position
theta = 4*45; % target angle in degrees
targetPos_rel = 0.12*[cosd(theta);sind(theta)]; % relative to initial hand position % for center out 12 cm is good. for cup task 25 cm

%% the control-oriented model for the robot MPC
withRobot_com = 1;
runTimeNoiseFactor_com = 0;
hmn_com = Human_COM(dt, tEnd, thetaEL_0, omegaEL_0, thetaSH_0, omegaSH_0, ...
            a_0, targetPos_rel, armDamping, numberOfStationarySteps, Fpert,'synergies.mat',withRobot_com, runTimeNoiseFactor_com);

%% Actual human-robot model
withRobot = 1;
runTimeNoiseFactor = 1;
hmn = Human(dt, tEnd, thetaEL_0, omegaEL_0, thetaSH_0, omegaSH_0, ...
            a_0, targetPos_rel, armDamping, numberOfStationarySteps, Fpert,'synergies.mat',withRobot, runTimeNoiseFactor, hmn_com);

for iter = 1:1
    hmn.resetStates(thetaEL_0, omegaEL_0, thetaSH_0, omegaSH_0,a_0);
    results = hmn.simulateHuman(figure(999));

    % save("human-"+string(datetime('now'),'yyyy-MM-d HH-mm-ss')+".mat",'hmn')


%     centerOutResults.("deg"+theta)(iter) = copy(hmn);
end
%%
Y_msk = results.msk_Ydata;
U_msk = results.msk_Udata;
X_msk = results.msk_Xdata;

nStep = hmn.generalParamSet.nStep;
dt = hmn.generalParamSet.dt;

figure(200); clf
ha200 = tightSubplot(3,3,0.1,0.1,0.1,0.1,0.1,0.1);
hold(ha200, "on")
box(ha200,"on")
% xlim(ha200,[0 1])
plot(ha200(1,1),(1:nStep)*dt,180/pi*hmn.msk.getOutputs(Y_msk).q,'LineWidth',2)
% plot(ha200(1,1),(1:nStep)*dt,180/pi*hmn.msk.getOutputs(Y_msk).qdot,'LineWidth',2)
title(ha200(1,1),'Joint angles (deg)')
xlabel(ha200(1,1),'Time (s)')
legend(ha200(1,1),'Elbow','Shoulder','Location','best','color','white','box','on')


plot(ha200(1,2),(1:nStep)*dt,hmn.msk.getOutputs(Y_msk).hand_p,'LineWidth',2)
plot(ha200(1,2),(1:nStep)*dt,hmn.msk.getOutputs(Y_msk).hand_v,'LineWidth',2)
title(ha200(1,2),"Physical task space")
xlabel(ha200(1,2),'Time (s)')
legend(ha200(1,2),'Pos_x','Pos_y','Vel_x','Vel_y','Location','best','NumColumns',2,'color','white','box','on')


plot(ha200(2,1),(1:nStep-1)*dt,U_msk,'LineWidth',2)
xlabel(ha200(2,1),'Time (s)')
title(ha200(2,1),'Muscle activations')
legend(ha200(2,1),'TRI bi','BIC','Tri uni','BRD','DLT pst','DLT ant','NumColumns',2,'Location','best','color','white','box','on')


plot(ha200(2,2),(1:nStep)*dt,hmn.msk.getOutputs(Y_msk).muscleF,'LineWidth',2)
xlabel(ha200(2,2),'Time (s)')
title(ha200(2,2),'Muscle forces (N)')
legend(ha200(2,2),'TRI bi','BIC','Tri uni','BRD','DLT pst','DLT ant','NumColumns',2,'Location','best','color','white','box','on')


plot(ha200(3,1),(1:nStep)*dt,results.msk_synergyData,'LineWidth',2)
xlabel(ha200(3,1),'Time (s)')
title(ha200(3,1),'Synerg activations')
legend(ha200(3,1),'Syn 1','Syn 2','Syn 3','Syn 4','NumColumns',2,'Location','best','color','white','box','on')


X_ofc = results.mc_Xdata;
Xest_ofc = results.mc_XEstdata;
targetPos_abs = hmn.generalParamSet.targetPos_abs;



% plot(ha200(1,3),(1:nStep)*dt, X_ofc(:,1:2)+targetPos_abs','-','LineWidth',2)
% plot(ha200(1,3),(1:nStep)*dt, X_ofc(:,3:4),'-','LineWidth',2)
plot(ha200(1,3),(1:nStep)*dt, Xest_ofc(:,1:2)+targetPos_abs','LineWidth',2)
plot(ha200(1,3),(1:nStep)*dt, Xest_ofc(:,3:4),'-','LineWidth',2)
% plot(ha200(1,3),(1:nStep)*dt, hmn.msk.getOutputs(Y_msk).hand_p)
% plot(ha200(1,3),(1:nStep)*dt, hmn.msk.getOutputs(Y_msk).hand_v)
xlabel(ha200(1,3),'Time (s)')
legend(ha200(1,3),'Pos_x','Pos_y','Vel_x','Vel_y','Location','best','NumColumns',2,'color','white','box','on')
title(ha200(1,3),"High-level 'abstract' task space")


% plot(ha300(2,3),(1:nStep)*dt, X_ofc(:,5:6),'--')
plot(ha200(2,3),(1:nStep)*dt, Xest_ofc(:,5:6),'LineWidth',2)
xlabel(ha200(2,3),'Time (s)')
title(ha200(2,3),"High-level muscle activation")
legend(ha200(2,3),'Muscle x','Muscle y','Location','best','color','white','box','on')


plot(ha200(3,2),(1:nStep)*dt,[Xest_ofc(:,7:8),results.rc_com_XEstData(:,7:8)])
legend(ha200(3,2),"internal model x","internal model y", "com internal model x", "com internal model y")
title(ha200(3,2),"estimated perturbation force");


plot(ha200(3,3),(1:nStep)*dt, results.msk_F_interData)
title(ha200(3,3),"actual interaction force");
% plot(ha200(3,3),(1:nStep)*dt,[X_ofc(:,[9:10]),Xest_ofc(:,[9:10])])
% plot(ha200(3,3),(1:nStep)*dt,results.pendulumStates(:,1:2),LineWidth=2)
% title(ha200(3,3),"estimated pendulum states");


figure(400)
plot((1:nStep)*dt,results.rc_Udata)
% plot((1:nStep)*dt,results.pendulumStates)
title('robot torques')
% title('pendulum states')
xlim([0,tEnd])



return

