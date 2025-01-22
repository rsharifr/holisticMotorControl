 classdef Human < matlab.mixin.Copyable

    properties
        generalParamSet
        mc
        msk
        rc
        results
    end

    methods
    %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % METHODS
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
        
        %% CONSTRUCTOR        
        function hmn = Human(dt, tEnd, thetaEL_0, omegaEL_0, thetaSH_0, omegaSH_0, a_0, targetPos_rel, armDamping, numberOfStationarySteps, Fpert, pathToSynergyData, withRobot, runTimeNoiseFactor, controlOrientedModel)
            hmn.generalParamSet.dt = dt;
            hmn.generalParamSet.tEnd = tEnd;
            hmn.generalParamSet.nStep = round(tEnd/dt) + numberOfStationarySteps;
            hmn.generalParamSet.Fpert = Fpert;
            hmn.generalParamSet.targetPos_rel = targetPos_rel;

            hmn.msk = MusculoskeletalArm(thetaEL_0, omegaEL_0, thetaSH_0, omegaSH_0, a_0, armDamping, pathToSynergyData, Fpert, withRobot, runTimeNoiseFactor);           
            
            startPosition = -targetPos_rel;
            hmn.mc = MotorController(dt,tEnd, numberOfStationarySteps, startPosition, Fpert, runTimeNoiseFactor);

            hmn.generalParamSet.initialHandPos = hmn.msk.getHandPosition(thetaSH_0,thetaEL_0);
            hmn.generalParamSet.targetPos_abs = targetPos_rel + hmn.generalParamSet.initialHandPos;

            hmn.rc = RobotController(controlOrientedModel);

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

            hmn.results.msk_synergyData = nan(nStep, hmn.msk.param.NoSyn);
            hmn.results.msk_F_interData = nan(nStep,2);
            
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

            hmn.results.rc_Udata = zeros(nStep,2);
            hmn.results.rc_com_XEstData = zeros(nStep,n);

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
            
            fprintf("Simulation started... "); 
            tic
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
                [~,~,coeff] = hmn.msk.acceleration2activation(hmn.results.msk_currentY, a_ref_mc, Fpert_est);
                

%%%%%%%%%%%%%%%%%         THIS IS OPTIMAL MOTOR TORQUE
                % j = mod(i-1,1);
                % if j==0
                %     [tau,com_cp] = hmn.rc.solveRobotTorque_optimal(i, hmn.getCurrentStates());
                % end
                % hmn.msk.environment.robotTorque = tau(:,j+1);
                
%%%%%%%%%%%%%%%%%         THIS IS THE EXTERNAL PERTURBATION
                % if i>=80 % a perturbation
                %     hmn.msk.environment.Fhand = [2;0];
                % end

%%%%%%%%%%%%%%%%%         THIS IS ADDITIONAL ENVIRONMENT
                % mskOutputs_tmp = hmn.msk.getOutputs();
                % hand_acc = mskOutputs_tmp.hand_a;
                % if ~exist('pendulum_x_Ang','var')% first loop iteration 
                %     pendulum_x_Ang = 0;
                %     pendulum_x_Vel = 0;
                %     pendulum_x_Acc = 0;
                %     pendulum_y_Ang = 0;
                %     pendulum_y_Vel = 0;
                %     pendulum_y_Acc = 0;
                %     pendStatesLog = zeros(hmn.generalParamSet.nStep,4);
                %     pendStatesLog(i,:) = [pendulum_x_Ang;pendulum_y_Ang;pendulum_x_Vel;pendulum_y_Vel];
                %     m_cart = hmn.mc.internalModel.parameters.mc; % total of m_hand+m_cart
                %     m_pend = hmn.mc.internalModel.parameters.mp;
                %     l_pend = hmn.mc.internalModel.parameters.l;
                %     gravity = hmn.mc.internalModel.parameters.g;
                %     pengulum_G = hmn.mc.internalModel.parameters.G; % "agility factor" for the pendulum
                % end
                % % pendulum_x_Force = (m_pend+m_cart)*hand_acc(1) - m_pend*l_pend*(pendulum_x_Vel^2*sin(pendulum_x_Ang) - pendulum_x_Acc*cos(pendulum_x_Ang)); % nonlinear pendulum
                % % pendulum_x_Acc = -gravity/l_pend*sin(pendulum_x_Ang) - pengulum_G/l_pend*hand_acc(1)*cos(pendulum_x_Ang); % nonlinear pendulum
                % pendulum_x_Force = (m_pend+m_cart)*hand_acc(1) + m_pend*l_pend*pendulum_x_Acc; % linear pendulum
                % pendulum_x_Acc = -gravity/l_pend*pendulum_x_Ang - pengulum_G/l_pend*hand_acc(1); % linear pendulum
                % 
                % % pendulum_y_Force = (m_pend+m_cart)*hand_acc(2) - m_pend*l_pend*(pendulum_y_Vel^2*sin(pendulum_y_Ang) - pendulum_y_Acc*cos(pendulum_y_Ang)); % nonlinear pendulum
                % % pendulum_y_Acc = -gravity/l_pend*sin(pendulum_y_Ang) - pengulum_G/l_pend*hand_acc(2)*cos(pendulum_y_Ang); % nonlinear pendulum
                % pendulum_y_Force = (m_pend+m_cart)*hand_acc(2) + m_pend*l_pend*pendulum_y_Acc; % linear pendulum
                % pendulum_y_Acc = -gravity/l_pend*pendulum_y_Ang - pengulum_G/l_pend*hand_acc(2); % linear pendulum
                % 
                % hmn.msk.environment.Fhand = -[pendulum_x_Force;pendulum_y_Force];
                % 
                % pendulum_x_Ang = pendulum_x_Ang + pendulum_x_Vel*dt;
                % pendulum_x_Vel = pendulum_x_Vel + pendulum_x_Acc*dt;
                % pendulum_y_Ang = pendulum_y_Ang + pendulum_y_Vel*dt;
                % pendulum_y_Vel = pendulum_y_Vel + pendulum_y_Acc*dt;                
                % 
                % pendStatesLog(i+1,:) = [pendulum_x_Ang;pendulum_y_Ang;pendulum_x_Vel;pendulum_y_Vel];

%%%%%%%%%%%%%%%%%           THIS IS IMPEDANCE CONTROL
%                 if ~exist('trajectory','var'), load("normalTrajectory.mat"); end
%                 try 
%                     sp_p = trajectory.hand_p(i+1,:)';
%                     sp_v = trajectory.hand_v(i+1,:)';
%                 catch
%                     sp_v = [0;0];
%                 end
%                 tau = hmn.rc.solveRobotTorque_imedanceControl(sp_p,sp_v,hmn.msk);
%                 if i>hmn.generalParamSet.nStep-20, tau = [0;0]; end
%                 hmn.msk.environment.robotTorque = tau;
                

%%%%%%%%%%%%%%                 THIS IS OTHER HAPTIC EFFECT / FORCE FIELD
                F_robot_des = [0;0]; % fully transparent robot
                % hand_v = hmn.msk.getOutputs(hmn.results.msk_currentY).hand_v;
                % F_robot_des = 3*[0 -1 ; 1 0]*hand_v; % velocity-dependent force field
                tau = hmn.rc.solveRobotTorque_hapticForce(F_robot_des, hmn.msk);
                hmn.msk.environment.robotTorque = tau;

%%%%%%%%%%%%%% END OF ALL ROBOT CONTROL MODES

                F_inter = hmn.msk.getInteractionForce();

                hmn.msk.solveModel(dt);
                
                
                hmn.results.msk_currentU = hmn.msk.U;
                hmn.results.msk_currentX = hmn.msk.X;
                hmn.results.msk_currentXdot = hmn.msk.XDOT;
                hmn.results.msk_currentY = hmn.msk.Y;
                hmn.results.msk_synergyData(i,:) = coeff';
                hmn.results.msk_F_interData(i,:) = F_inter';
                hmn.results.rc_Udata(i,:) = hmn.msk.environment.robotTorque';
%                 hmn.results.rc_com_XEstData(i,:) = com_cp.results.mc_XEstdata(i,:);

                %%%%%%% Estimator's integration step                
                
                if i<hmn.mc.ofc.simSetting.delay
                    mskOutputs_delayed = hmn.msk.getOutputs(hmn.results.msk_Ydata(1,:)');
                else
                    mskOutputs_delayed = hmn.msk.getOutputs(hmn.results.msk_Ydata(i - hmn.mc.ofc.simSetting.delay+1,:)');
                end
                mskOutputs = hmn.msk.getOutputs(hmn.results.msk_currentY);

                sensoryFeedback = hmn.mc.updateFeedback(mskOutputs_delayed, hmn.generalParamSet.targetPos_abs, Fpert);
                XEst_mc_new = hmn.mc.updateEstimator(sensoryFeedback);

                hmn.results.mc_XEstdata(i+1,:) = XEst_mc_new';
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

                    if hmn.msk.param.withRobot
                        q1 = mskOutputs.q_robot(1) + pi; % the robot's frame is flipped 180 degrees
                        q2 = mskOutputs.q_robot(2) ;
                        param_robot = getParametersRobot; 
                        ee = hnd;
                        middleJoint = ee - [param_robot.a2.*cos(q1+q2); param_robot.a2.*sin(q1+q2)];
                        base = middleJoint - [param_robot.a1.*cos(q1); param_robot.a1.*sin(q1)];

                        plot(ha,[ee(1),middleJoint(1),base(1)], [ee(2),middleJoint(2),base(2)],'-o','LineWidth',2,'Color',cmap_rob(i,:));
                    end
                    drawnow
                end

            end
            fprintf("Simulation done! Elapsed time %d min %0.3f sec\n", floor(toc/60), mod(toc,60)); 
            

            hmn.results.mc_Xdata = hmn.results.mc_Xdata(:,1:hmn.mc.ofc.systemEq.numberOfOriginalStates);
            hmn.results.mc_XEstdata = hmn.results.mc_XEstdata(:,1:hmn.mc.ofc.systemEq.numberOfOriginalStates);
            hmn.results.rc_com_XEstData = hmn.results.rc_com_XEstData(:,1:hmn.mc.ofc.systemEq.numberOfOriginalStates);
            results = hmn.results; 
            if exist('pendStatesLog','var'), results.pendulumStates = pendStatesLog; end
        end
        %% GET FULL STATE OF THE HUMAN MODEL
        function currentStates = getCurrentStates(hmn)
            currentStates.msk = hmn.msk.getStates();
            currentStates.mc = hmn.mc.internalStates;
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
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    
    %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % METHODS - PROTECTED
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
    
    methods (Access = protected)
        function obj_cp = copyElement(obj)
            obj_cp = copyElement@matlab.mixin.Copyable(obj);
            obj_cp.mc = copy(obj.mc); %Deep copy of object
            obj_cp.msk = copy(obj.msk); %Deep copy of object
        end
    end
end