classdef MotorController < matlab.mixin.Copyable

    properties
        ofc % contains control parameters and discreatized delayed system eqs
        param % high-level parameters for the motor controller
        internalModel % contains parameters and continious-time system eqs
        internalStates % states of the internal model (including delayed copies)
    end

    methods 
        %% CONSTRUCTOR
        function mc = MotorController(dt,tEnd, numberOfStationarySteps, startPosition, Fpert, runTimeNoiseFactor)
            mc = setupInternalModel(mc,dt,tEnd, numberOfStationarySteps, startPosition, Fpert);
            mc = setupOFC(mc);
            mc.param.runTimeNoiseFactor = runTimeNoiseFactor;
        end
    %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % METHODS
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
        %% SETUP INTERNAL MODEL
        function mc = setupInternalModel(mc,dt,tEnd, numberOfStationarySteps, startPosition, Fpert)
            [A,B,H,Q,R,simSetting,noiseStructure,xInit,modelParam] = setupInternalModel_pointMass_effort(startPosition, dt, tEnd, numberOfStationarySteps,Fpert);
            % [A,B,H,Q,R,simSetting,noiseStructure,xInit,modelParam] = setupInternalModel_cupTask_effort(startPosition, dt, tEnd, numberOfStationarySteps,Fpert);

            mc.internalModel.parameters = modelParam;
            mc.internalModel.xInit = xInit;            
            mc.internalModel.A = A;
            mc.internalModel.B = B;
            mc.internalModel.H = H;
            mc.param.Q = Q;
            mc.param.R = R;
            mc.param.simSetting = simSetting;
            mc.param.noiseStructure = noiseStructure;
        end
        %% SETUP OFC
        function mc = setupOFC(mc)
            mc.ofc = OFC(mc.internalModel.A,mc.internalModel.B,mc.internalModel.H,mc.param.Q,mc.param.R,mc.param.simSetting,mc.param.noiseStructure);
            [mc.param.L,mc.param.K] = mc.ofc.getOptimalGains;
            
            mc.internalStates.X = repmat(mc.internalModel.xInit,1+mc.ofc.simSetting.delay,1);
            mc.internalStates.XEst = mc.internalStates.X;
            mc.internalStates.U = zeros(mc.ofc.systemEq.numberOfControls,1);
            mc.internalStates.timeIndex = 1;
        end

        %% UPDATE ESTIMATOR
        function [XEst_new, mc] = updateEstimator(mc, yz)
            p = mc.ofc.systemEq.numberOfOutputs;
            n = mc.ofc.systemEq.numberOfStates;
            Omega_omega = mc.ofc.noiseConstructors.sensoryNoiseCovar;
            Omega_eta = mc.ofc.noiseConstructors.internalNoiseCovar;
            D = mc.ofc.noiseConstructors.stateDependentConstructor;
            sensoryNoise = mvnrnd(zeros(p,1),Omega_omega)';
            internalNoise = mvnrnd(zeros(n,1),Omega_eta)';


            noiseFactor = mc.param.runTimeNoiseFactor;
            stateDependentNoise = 0;
            for isdn = 1:size(D,3)
                stateDependentNoise = stateDependentNoise + randn*D(:,:,isdn)*mc.internalStates.X;
            end
            
            yz = yz + noiseFactor*(sensoryNoise + stateDependentNoise);
            
            i = mc.internalStates.timeIndex;
            A = mc.ofc.systemEq.A;
            B = mc.ofc.systemEq.B;
            H = mc.ofc.systemEq.H;
            K = mc.param.K;
            XEst_new = A(:,:,i)*mc.internalStates.XEst + B*mc.internalStates.U + K(:,:,i)*(yz-H*mc.internalStates.XEst) + noiseFactor*internalNoise;
            mc.internalStates.XEst = XEst_new; 

        end

        %% UPDATE INTERNAL MODEL STATES
        function [X_new, mc] = updateInternalModel(mc)
            C = mc.ofc.noiseConstructors.controlDependentConstructor;
            n = mc.ofc.systemEq.numberOfStates;
            Omega_xi = mc.ofc.noiseConstructors.additiveProcessNoiseCovar;
            
            noiseFactor = mc.param.runTimeNoiseFactor;

            processNoise = mvnrnd(zeros(n,1),Omega_xi)';
            controlDependentNoise = 0;
            for icdn = 1:size(C,3)
                controlDependentNoise = controlDependentNoise + randn*C(:,:,icdn)*mc.internalStates.U;
            end

            A_sim = mc.ofc.systemEq.A_sim;
            B = mc.ofc.systemEq.B;
            i = mc.internalStates.timeIndex;

            X_new = A_sim(:,:,i)*mc.internalStates.X + B*mc.internalStates.U + noiseFactor*(processNoise + controlDependentNoise);
            mc.internalStates.X = X_new;
        end
        %% CALCULATE CONTROL
        function [U, mc] = getControlCommand(mc)
            i = mc.internalStates.timeIndex;
            L = mc.param.L;
            U = -L(:,:,i)*mc.internalStates.XEst;
            mc.internalStates.U = U;
        end        
        %% INCREMENT INTERNAL TIME
        function [timeIndex_new, mc] = incrementInternalTime(mc)
            timeIndex_new = mc.internalStates.timeIndex + 1;
            mc.internalStates.timeIndex = timeIndex_new;
        end
        %% CONSTRUCT MEASUREMENT FEEDBACK VECTOR
        function y = updateFeedback(mc, mskOutputs, targetPos_abs, Fpert)
            handPos = mskOutputs.hand_p - targetPos_abs;
            handVel = mskOutputs.hand_v;
            y = [handPos; handVel; mc.internalStates.X(5:6); Fpert]; % for point mass
            % y = [handPos; handVel; mc.internalStates.X(5:6); Fpert; mc.internalStates.X(9:12)]; % for cup task

            % reading from the internal model
%             H = mc.ofc.systemEq.H;
%             y = H*mc.internalStates.X;
        end
        %% REST TO INITIAL CONDITION
        function resetStates(mc)
            h = mc.ofc.simSetting.delay;
            m = mc.ofc.systemEq.numberOfControls;
            mc.internalStates.X = repmat(mc.internalModel.xInit,h+1,1);
            mc.internalStates.XEst = mc.internalStates.X;
            mc.internalStates.U = zeros(m,1);
            mc.internalStates.timeIndex = 1;
        end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % END OF METHODS
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    
    %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % METHODS - PROTECTED
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
    
    methods (Access = protected)
        function obj_cp = copyElement(obj)
            obj_cp = copyElement@matlab.mixin.Copyable(obj);
            obj_cp.ofc = copy(obj.ofc); %Deep copy of object
        end
    end
end