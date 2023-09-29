 classdef Human < matlab.mixin.SetGet

    properties
        generalParamSet
        mc
        msk
        results
    end

    methods
    %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % METHODS
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
        
        %% CONSTRUCTOR        
        function hmn = Human(dt, tEnd, thetaEL_0, omegaEL_0, thetaSH_0, omegaSH_0, a_0, targetPos_rel, armDamping, numberOfStationarySteps, Fpert, pathToSynergyData, withRobot)
            hmn.generalParamSet.dt = dt;
            hmn.generalParamSet.tEnd = tEnd;
            hmn.generalParamSet.nStep = round(tEnd/dt) + numberOfStationarySteps;
            hmn.generalParamSet.Fpert = Fpert;
            hmn.generalParamSet.targetPos_rel = targetPos_rel;

            hmn.msk = MusculoskeletalArm(thetaEL_0, omegaEL_0, thetaSH_0, omegaSH_0, a_0, armDamping, pathToSynergyData, Fpert, withRobot);           
            
            startPosition = -targetPos_rel;
            hmn.mc = MotorController(dt,tEnd, numberOfStationarySteps, startPosition, Fpert);

            hmn.generalParamSet.initialHandPos = hmn.msk.getHandPosition(thetaSH_0,thetaEL_0);
            hmn.generalParamSet.targetPos_abs = targetPos_rel + hmn.generalParamSet.initialHandPos;

            hmn.initiateResultsStruct();
        end
        
        %% SETUP DATA LOGGER STRUCT
        function hmn = initiateResultsStruct(hmn)
            nStep = hmn.generalParamSet.nStep;
            dt = hmn.generalParamSet.dt;

            % MSK data logger
            Nmuscle = hmn.msk.param.Nmuscle;
            IC = hmn.msk.IC;
            Y = hmn.msk.Y;

            hmn.results.msk_currentX = IC;
            hmn.results.msk_currentXdot = zeros(size(IC));
            hmn.results.msk_Xdata = nan(nStep, length(IC));
            hmn.results.msk_Xdata(1,:) = IC';

            hmn.results.msk_currentY = Y;
            hmn.results.msk_Ydata = nan(nStep, length(Y));
            hmn.results.msk_Ydata(1,:) = Y(:,1)';

            hmn.results.msk_currentU = zeros(Nmuscle,1);
            hmn.results.msk_Udata = nan(nStep-1, Nmuscle);

            hmn.results.msk_t = (0:nStep-1)*dt;

            % MotorControl data logger
            n = hmn.mc.ofc.systemEq.numberOfStates;
            m = hmn.mc.ofc.systemEq.numberOfControls;
            h = hmn.mc.ofc.simSetting.delay;

            hmn.results.mc_currentX = repmat(hmn.mc.ofc.simSetting.xInit,h+1,1);
            hmn.results.mc_Xdata = zeros(nStep,n);
            hmn.results.mc_Xdata(1,:) = hmn.results.mc_currentX';

            hmn.results.mc_currentXEst = hmn.results.mc_currentX;
            hmn.results.mc_XEstdata = hmn.results.mc_Xdata;
            
            hmn.results.mc_currentU = zeros(m,1);
            hmn.results.mc_Udata = zeros(nStep-1,m);

            hmn.results.mc_t = (0:nStep-1)*dt;

        end

        %% SIMULATE HUMAN MODEL
        function results = simulateHuman(hmn, figureHandle)
            plotResults = false;
            if exist('figureHandle','var') 
                if ishandle(figureHandle)
                    plotResults = true;
                    figure(figureHandle)
                    clf("reset");
                    figureHandle.Name = "Hand path";
                    ha = axes;
                    hold(ha,"on");
                    ha.DataAspectRatio = [1,1,1];
                    cmap_arm = copper(hmn.generalParamSet.nStep);
                    cmap_rob = winter(hmn.generalParamSet.nStep);
                else
                    warning("A figure handle was expected. Skipping plotting hand path")
                end
            end
               

            dt = hmn.generalParamSet.dt;

            Fpert = hmn.generalParamSet.Fpert;

            for i = 1:hmn.generalParamSet.nStep-1
                %%%%%% OFC's integral step
                U_mc = hmn.mc.getControlCommand();

                X_mc_old = hmn.mc.internalStates.X;
                hmn.mc.updateInternalModel();
                X_mc_new = hmn.mc.internalStates.X;
                X_mc_diff = (X_mc_new - X_mc_old)/dt;
                
                hmn.results.mc_currentX = X_mc_new;
                hmn.results.mc_Xdata(i+1,:) = X_mc_new';
                hmn.results.mc_currentU = U_mc;

                %%%%%%% Generate high-level commands
                a_ref_mc = X_mc_diff(3:4); 

                %%%%%%% MSK's step
                hmn.results.msk_Udata(i,:) = hmn.msk.U';
                hmn.results.msk_Ydata(i,:) = hmn.msk.Y';
                hmn.results.msk_Xdata(i,:) = hmn.msk.X';

                Fpert_est = hmn.mc.internalStates.XEst(7:8);
                hmn.msk.acceleration2activation(hmn.results.msk_currentY, a_ref_mc, Fpert_est);
                                
                hmn.msk.solveModel(dt);
                
                hmn.results.msk_currentU = hmn.msk.U;
                hmn.results.msk_currentX = hmn.msk.X;
                hmn.results.msk_currentXdot = hmn.msk.XDOT;
                hmn.results.msk_currentY = hmn.msk.Y;

                %%%%%%% Estimator's integration step
                mskOutputs = hmn.msk.getOutputs(hmn.results.msk_currentY);
                
                sensoryFeedback = hmn.mc.updateFeedback(mskOutputs, hmn.generalParamSet.targetPos_abs, Fpert);
                XEst_mc_new = hmn.mc.updateEstimator(sensoryFeedback);

                hmn.results.mc_XEstdata(i,:) = XEst_mc_new';
                hmn.results.mc_currentXEst = XEst_mc_new; 

                hmn.mc.incrementInternalTime();

                %%%%%%% plot this arm path
                if plotResults && mod(i,10)==1
                    q_SH = mskOutputs.q(1);
                    q_EL = mskOutputs.q(2);
                    [hnd, elb] = hmn.msk.getHandPosition(q_SH,q_EL);
                    plot(ha,[0,elb(1),hnd(1)], [0,elb(2),hnd(2)],'-o','LineWidth',2,'Color',cmap_arm(i,:))
                    scatter(ha,hmn.results.mc_currentX(1)+hmn.generalParamSet.targetPos_abs(1), hmn.results.mc_currentX(2)+hmn.generalParamSet.targetPos_abs(2),'s','filled')
                    scatter(ha,hmn.results.mc_currentXEst(1)+hmn.generalParamSet.targetPos_abs(1), hmn.results.mc_currentXEst(2)+hmn.generalParamSet.targetPos_abs(2),100,'x','MarkerEdgeColor','r','LineWidth',2)

%                     if hmn.msk.param.withRobot
%                         q1 = mskOutputs.q_robot(1) + pi;
%                         q2 = mskOutputs.q_robot(2);
%                         param_robot = getParametersRobot; 
%                         ee = hnd;
%                         middleJoint = ee - [param_robot.a2.*cos(q1+q2); param_robot.a2.*sin(q1+q2)];
%                         base = middleJoint - [param_robot.a1.*cos(q1); param_robot.a1.*sin(q1)];
% 
%                         plot(ha,[ee(1),middleJoint(1),base(1)], [ee(2),middleJoint(2),base(2)],'-o','LineWidth',2,'Color',cmap_rob(i,:));
%                     end
                end

            end

            hmn.results.mc_Xdata = hmn.results.mc_Xdata(:,1:hmn.mc.ofc.systemEq.numberOfOriginalStates);
            hmn.results.mc_XEstdata = hmn.results.mc_XEstdata(:,1:hmn.mc.ofc.systemEq.numberOfOriginalStates);

            results = hmn.results;
        end
        %% REST STATES
        function resetStates(hmn,thetaEL_0, omegaEL_0, thetaSH_0, omegaSH_0, a_0)
            % TODO: there is a bug here. if initial angles change, the
            % xInit of OFC will not change. 
            hmn.mc.resetStates();
            hmn.msk.resetStates(thetaEL_0, omegaEL_0, thetaSH_0, omegaSH_0, a_0);
            hmn.initiateResultsStruct();
        end

    %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % END OF METHODS
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end
end