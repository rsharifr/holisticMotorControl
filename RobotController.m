classdef RobotController < matlab.mixin.Copyable

    properties
        com
        param
%         plantStates
        optimalCoeff
    end

    methods 
        %% CONSTRUCTOR
        function rc = RobotController(controlOrientedModel)
            rc.com = controlOrientedModel;

            rc.param.predictionHorizon = 50;
            rc.param.nStep = controlOrientedModel.mc.ofc.simSetting.nStep;
            
            rc.param.controParameterizationType = "poly";
            rc.param.impedance_k = 20;
            rc.param.impedance_b = 20;
%             rc.param.optimizationOptions = optimoptions("fmincon","Display","iter");
            
            rc.optimalCoeff = [];
        end
    %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % METHODS
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
        %% Calculate the objective function given motor torque
        function [obj, tauData, com_cp] = robotObjFun(rc, tauParam, currentTimeIndex, currentPlantStates, figureHandle)
            simulationRange = [currentTimeIndex, min(rc.param.nStep, currentTimeIndex+rc.param.predictionHorizon)-1];
            N = simulationRange(2)-simulationRange(1) + 1;

            tauData = rc.getTauData(tauParam, N);
            tauInput(:,simulationRange(1):simulationRange(2)) = tauData;
            
            com_cp = copy(rc.com);
            com_cp.predictHumanMotion(simulationRange, currentPlantStates, tauInput, figureHandle);
            
            mskResults = com_cp.msk.getOutputs(com_cp.results.msk_Ydata(simulationRange(1):simulationRange(2),:));
            muscleForceCost = sum(mskResults.muscleF.^2,'all')/N;
            
            muscleU = com_cp.results.msk_Udata(simulationRange(1):simulationRange(2),:);
            muscleUCost = sum(muscleU.^2,'all')/N;

            muscleF = com_cp.msk.getOutputs(com_cp.results.msk_Ydata).muscleF;
            muscleF = muscleF(simulationRange(1):simulationRange(2),:);
            muscleF(isnan(muscleF)) = 1000;
            muscleFCost = sum(muscleF.^2,'all')/N;
            
            F_inter = com_cp.results.msk_F_interData(simulationRange(1):simulationRange(2),:);
            interForceCost = sum(F_inter.^2,'all')/N;

            obj = 1e4*muscleUCost + 1e-1*muscleFCost + 0*muscleForceCost + 0*interForceCost;
            if isnan(obj) || isinf(obj)
                obj = 100;
            end

        end
        %% Optimize the robot motor torques at current time
        function [tau_opt,com_cp] = solveRobotTorque_optimal(rc, currentIndex, currentPlantStates)
            order = 2;
            if isempty(rc.optimalCoeff)
                X0 = zeros(2,order+1);
            else
                X0 = rc.optimalCoeff;
            end

            obj = @(X)rc.robotObjFun(X,currentIndex,currentPlantStates,false);
            A = [];
            b = [];
            Aeq = [];
            beq = [];
            lb = -50*ones(size(X0));
            ub =  50*ones(size(X0));
            nlconst = [];
%             options = rc.param.optimizationOptions;
            X_opt = patternsearch(obj,X0, A,b,Aeq,beq,lb,ub,nlconst,optimoptions("patternsearch","Display","iter","UseParallel",true,"MaxIterations",50))
            X_opt = fmincon(obj,X_opt, A,b,Aeq,beq,lb,ub,nlconst,optimoptions("fmincon",'Algorithm','sqp','Display','iter',"UseParallel",true))
%             tau_opt = [0;0];
%             tau_opt = [-1;1] + tau0;
            rc.optimalCoeff = X_opt;
            [~, tau_opt, com_cp] = rc.robotObjFun(X_opt,currentIndex, currentPlantStates, figure(1000));            

        end
        %% Get robot motor torques for haptic effects (force applied onto hand)
        function tau = solveRobotTorque_hapticForce(rc, Fint_des, msk)
            robotParam = getParametersRobot();
            states = msk.getStates();
            qdot_r = states.qdot_robot;
            q_r = states.q_robot;
            [~,qddot_r] = msk.getqddot();

            M = sysEQ_massMatrix(q_r,robotParam);
            C = sysEQ_CMatrix([q_r;qdot_r],robotParam);
            J = sysEQ_J(q_r,robotParam);

            tau = J'*-Fint_des + M*qddot_r + C*qdot_r;
        end
        %% Get robot motor torques for trajectory control
        function tau = solveRobotTorque_imedanceControl(rc,setpoint_p, setpoint_v,msk)
            k = rc.param.impedance_k;
            b = rc.param.impedance_b;

            mskOutputs = msk.getOutputs();
            hand_p = mskOutputs.hand_p;
            hand_v = mskOutputs.hand_v;
            e_p = hand_p-setpoint_p;
            e_v = hand_v-setpoint_v;
            F_k = e_p*k;
            F_d = e_v*b;
            F_des = F_k + F_d;
            tau = rc.solveRobotTorque_hapticForce(-F_des, msk);
            
        end
        %% CALCULATE TORQUE PATTERN FROM PARAMETERS
        function tauData = getTauData(rc, tauParam, vectorLength)
            t = linspace(0,1,vectorLength);
            if rc.param.controParameterizationType=="poly"
                tau1 = polyval(tauParam(1,:),t);
                tau2 = polyval(tauParam(2,:),t);
                tauData = [tau1;tau2];
            else
                error("unknown torque parameterization")
            end
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
            obj_cp.com = copy(obj.com); %Deep copy of object
        end
    end
end