%% Plotting Cetner out and force field.
clc
clearvars

fontsize = 12;

load('centerOutResults_null.mat')
% load('humanWithRobotOn.mat')


figure(100)
clf
ha = axes;
hold(ha,"on")
ha.DataAspectRatio = [1,1,1];
ha.XTick = [-0.15 0 0.15];
ha.YTick = [-0.15 0 0.15];
xlim(ha,[-0.15,0.15])
ha.XLabel.String = 'x (m)';
ha.YLabel.String = 'y (m)';
for theta = 0:45:315
    scatter(ha, 0.12*cosd(theta), 0.12*sind(theta),100,'filled','s','MarkerFaceColor','k')

    for iter = 1:10
    hmn = centerOutResults.("deg"+theta)(iter);
    Y = hmn.msk.getOutputs(hmn.results.msk_Ydata);
    hand_p = Y.hand_p;
    plot(ha,hand_p(:,1)-hand_p(1,1),hand_p(:,2)-hand_p(1,2),'Color',0.5*[1,1,1]);
    end
end


%%
hmn = centerOutResults.deg0(1);
%%
results = hmn.results;

Y_msk = results.msk_Ydata;
U_msk = results.msk_Udata;
X_msk = results.msk_Xdata;

nStep = hmn.generalParamSet.nStep;
dt = hmn.generalParamSet.dt;

X_ofc = results.mc_Xdata;
Xest_ofc = results.mc_XEstdata;
targetPos_abs = hmn.generalParamSet.targetPos_abs;


hf200 = figure(200); clf
ha200 = tightSubplot(7,1,0.05,0.02,0.12,0.05,0.1,0.05);
hold(ha200, "on")
box(ha200,"on")
xlim(ha200,[0 1])




% plot(hf300(1,1),(1:nStep)*dt, X_ofc(:,1:2)+targetPos_abs','-','LineWidth',2)
% plot(hf300(1,1),(1:nStep)*dt, X_ofc(:,3:4),'-','LineWidth',2)
plot(ha200(1,1),(1:nStep)*dt, Xest_ofc(:,1:2)+targetPos_abs','LineWidth',2)
plot(ha200(1,1),(1:nStep)*dt, Xest_ofc(:,3:4),'-','LineWidth',2)
% plot(hf300(1,1),(1:nStep)*dt, hmn.msk.getOutputs(Y_msk).hand_p)
% plot(hf300(1,1),(1:nStep)*dt, hmn.msk.getOutputs(Y_msk).hand_v)
legend(ha200(1,1),'Pos_x','Pos_y','Vel_x','Vel_y','Location','best','NumColumns',2,'color','none','box','off')
title(ha200(1,1),"'Abstract' task space")
% ylim(ha200(1,1),[-0.5 0.5])



% plot(ha300(2,1),(1:nStep)*dt, X_ofc(:,5:6),'--')
plot(ha200(2,1),(1:nStep)*dt, Xest_ofc(:,5:6),'LineWidth',2)
title(ha200(2,1),"'Abstract' neural excitation")
legend(ha200(2,1),'Muscle x','Muscle y','Location','best','color','none','box','off')
% ylim(ha200(2,1),[-5 15]*1e-3)


plot(ha200(3,1),(1:nStep)*dt,results.msk_synergyData,'LineWidth',2)
title(ha200(3,1),'Synerg activations')
legend(ha200(3,1),'Syn 1','Syn 2','Syn 3','Syn 4','NumColumns',2,'Location','best','color','none','box','off')
% ylim(ha200(3,1),[0 0.3])



plot(ha200(4,1),(1:nStep-1)*dt,U_msk,'LineWidth',2)
title(ha200(4,1),'Neural excitations')
legend(ha200(4,1),'TRI bi','BIC','Tri uni','BRD','DLT pst','DLT ant','NumColumns',2,'Location','best','color','none','box','off')
% ylim(ha200(4,1),[0 0.1])


plot(ha200(5,1),(1:nStep)*dt,hmn.msk.getOutputs(Y_msk).muscleF,'LineWidth',2)
title(ha200(5,1),'Muscle forces (N)')
legend(ha200(5,1),'TRI bi','BIC','Tri uni','BRD','DLT pst','DLT ant','NumColumns',2,'Location','best','color','none','box','off')
% ylim(ha200(5,1),[0 100])



plot(ha200(6,1),(1:nStep)*dt,hmn.msk.getOutputs(Y_msk).hand_p,'LineWidth',2)
plot(ha200(6,1),(1:nStep)*dt,hmn.msk.getOutputs(Y_msk).hand_v,'LineWidth',2)
title(ha200(6,1),"Physical task space")
legend(ha200(6,1),'Pos_x','Pos_y','Vel_x','Vel_y','Location','best','NumColumns',2,'color','none','box','off')
% ylim(ha200(6,1),[-0.5 0.5])


% if contains(fileName,'WithRobot')
%     plot(ha200(7,1),(1:nStep)*dt,results.rc_Udata,'LineWidth',2)
%     title(ha200(7,1),"Robot torques")
%     legend(ha200(7,1),'T_1','T_2','Location','best','NumColumns',2,'color','none','box','off')
%     xlabel(ha200(7,1),'Time (s)')
%     ylim(ha200(7,1),[-4,2])
% else
    xlabel(ha200(6,1),'Time (s)')
    delete(ha200(7,1))
    ha200 = ha200(1:end-1);
% end

% if ~ (strcmpi(fileName,'humanAlone'))
%     [ha200.YTickLabel] = deal([]);
% end
% if strcmpi(fileName,'humanWithRobotOff')
%      ha200(end,1).YTickLabelMode ="auto";
% end

[ha200(1:end-1,:).XTickLabel] = deal([]);

% fontsize = 12;
setFontSize(hf200,fontsize)



%% plotting the synergies
load("synergies.mat")
muscleNames = ["Bi ext","Bi flx","El ext","El flx","SH ext","Sh flx"];

[el,sh] = meshgrid(thetaELset*180/pi,thetaSHset*180/pi);

hf2000 = figure(2000);
ha2000 = tightSubplot(size(synergySet,1),size(synergySet,2),0.05,0.05,0.1,0.05,0.05,0.05);
hold(ha2000,'on');

for s = 1:size(synergySet,2) 
    for m = 1:size(synergySet,1)
        coeff = squeeze(synergySet(m,s,:,:));
        surface(ha2000(m,s),el,sh,coeff);
        view(ha2000(m,s), -35,35)
        if s == 1
            zlabel(ha2000(m,s),muscleNames(m),FontWeight="bold")
        end
        if m == 1
            title(ha2000(m,s), sprintf("Synergy %d",s))
        end
    end
    
end
xlabel(ha2000(end,1),'Shoulder angle (deg)',Position=[-30,-60,0],Rotation=10)
ylabel(ha2000(end,1),'Elbow angle (deg)',Position=[-60,-50,0],Rotation=-25)

saveFig(8.5,8.5,'plots','synergies','png',hf2000,false)


%% Plotting the human-aware control results
fileName = 'humanAlone';
% fileName = 'humanWithRobotOff';
% fileName = 'humanWithRobotOn';
% fileName = 'humanWithRobotOnPertOnUnexpected';
% fileName = 'humanWithRobotOnPertOnExpected';
% fileName = 'humanWithRobotImpedance';
load([fileName '.mat']);
results = hmn.results;

Y_msk = results.msk_Ydata;
U_msk = results.msk_Udata;
X_msk = results.msk_Xdata;

nStep = hmn.generalParamSet.nStep;
dt = hmn.generalParamSet.dt;

X_ofc = results.mc_Xdata;
Xest_ofc = results.mc_XEstdata;
targetPos_abs = hmn.generalParamSet.targetPos_abs;


hf300 = figure(300); clf
ha300 = tightSubplot(7,1,0.05,0.02,0.12,0.05,0.1,0.05);
hold(ha300, "on")
box(ha300,"on")
xlim(ha300,[0 1])




% plot(hf300(1,1),(1:nStep)*dt, X_ofc(:,1:2)+targetPos_abs','-','LineWidth',2)
% plot(hf300(1,1),(1:nStep)*dt, X_ofc(:,3:4),'-','LineWidth',2)
plot(ha300(1,1),(1:nStep)*dt, Xest_ofc(:,1:2)+targetPos_abs','LineWidth',2)
plot(ha300(1,1),(1:nStep)*dt, Xest_ofc(:,3:4),'-','LineWidth',2)
% plot(hf300(1,1),(1:nStep)*dt, hmn.msk.getOutputs(Y_msk).hand_p)
% plot(hf300(1,1),(1:nStep)*dt, hmn.msk.getOutputs(Y_msk).hand_v)
legend(ha300(1,1),'Pos_x','Pos_y','Vel_x','Vel_y','Location','best','NumColumns',2,'color','none','box','off')
title(ha300(1,1),"'Abstract' task space")
ylim(ha300(1,1),[-0.5 0.5])



% plot(ha300(2,1),(1:nStep)*dt, X_ofc(:,5:6),'--')
plot(ha300(2,1),(1:nStep)*dt, Xest_ofc(:,5:6),'LineWidth',2)
title(ha300(2,1),"'Abstract' neural excitation")
legend(ha300(2,1),'Muscle x','Muscle y','Location','best','color','none','box','off')
ylim(ha300(2,1),[-5 15]*1e-3)


plot(ha300(3,1),(1:nStep)*dt,results.msk_synergyData,'LineWidth',2)
title(ha300(3,1),'Synerg activations')
legend(ha300(3,1),'Syn 1','Syn 2','Syn 3','Syn 4','NumColumns',2,'Location','best','color','none','box','off')
ylim(ha300(3,1),[0 0.3])



plot(ha300(4,1),(1:nStep-1)*dt,U_msk,'LineWidth',2)
title(ha300(4,1),'Neural excitations')
legend(ha300(4,1),'TRI bi','BIC','Tri uni','BRD','DLT pst','DLT ant','NumColumns',2,'Location','best','color','none','box','off')
ylim(ha300(4,1),[0 0.1])


plot(ha300(5,1),(1:nStep)*dt,hmn.msk.getOutputs(Y_msk).muscleF,'LineWidth',2)
title(ha300(5,1),'Muscle forces (N)')
legend(ha300(5,1),'TRI bi','BIC','Tri uni','BRD','DLT pst','DLT ant','NumColumns',2,'Location','best','color','none','box','off')
ylim(ha300(5,1),[0 100])



plot(ha300(6,1),(1:nStep)*dt,hmn.msk.getOutputs(Y_msk).hand_p,'LineWidth',2)
plot(ha300(6,1),(1:nStep)*dt,hmn.msk.getOutputs(Y_msk).hand_v,'LineWidth',2)
title(ha300(6,1),"Physical task space")
legend(ha300(6,1),'Pos_x','Pos_y','Vel_x','Vel_y','Location','best','NumColumns',2,'color','none','box','off')
ylim(ha300(6,1),[-0.5 0.5])


if contains(fileName,'WithRobot')
    plot(ha300(7,1),(1:nStep)*dt,results.rc_Udata,'LineWidth',2)
    title(ha300(7,1),"Robot torques")
    legend(ha300(7,1),'T_1','T_2','Location','best','NumColumns',2,'color','none','box','off')
    xlabel(ha300(7,1),'Time (s)')
    ylim(ha300(7,1),[-4,2])
else
    xlabel(ha300(6,1),'Time (s)')
    delete(ha300(7,1))
    ha300 = ha300(1:end-1);
end

if ~ (strcmpi(fileName,'humanAlone'))
    [ha300.YTickLabel] = deal([]);
end
if strcmpi(fileName,'humanWithRobotOff')
     ha300(end,1).YTickLabelMode ="auto";
end

[ha300(1:end-1,:).XTickLabel] = deal([]);

fontsize = 7;
setFontSize(hf300,fontsize)

% saveFig(1.55,7,'plots',fileName,'pdf',hf300,false)

% plot(hf300(3,2),(1:nStep)*dt,[Xest_ofc(:,7:8),results.rc_com_XEstData(:,7:8)])
% 
% plot(hf300(3,3),(1:nStep)*dt, results.msk_F_interData)

