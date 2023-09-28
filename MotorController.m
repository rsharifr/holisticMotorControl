classdef MotorController < matlab.mixin.SetGet

    properties
        ofc
        param
        internalStates
    end

    methods 
        %% CONSTRUCTOR
        function mc = MotorController(dt,tEnd, numberOfStationarySteps, startPosition, Fpert)
            mc = setupOFC(mc,dt,tEnd, numberOfStationarySteps, startPosition, Fpert);
        end
    %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % METHODS
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
        %% SETUP OFC
        function mc = setupOFC(mc,dt,tEnd, numberOfStationarySteps, startPosition, Fpert)
            [A,B,H,Q,R,simSetting,noiseStructure,xInit,modelParam] = setupInternalModel_pointMass_effort(startPosition, dt, tEnd, numberOfStationarySteps,Fpert);
            mc.ofc = OFC(A,B,H,Q,R,simSetting,noiseStructure);
            mc.param = modelParam;
            mc.param.xInit = xInit;

            [mc.param.L,mc.param.K] = mc.ofc.getOptimalGains;
            
            h = mc.ofc.simSetting.delay;
            m = mc.ofc.systemEq.numberOfControls;
            mc.internalStates.X = repmat(xInit,h+1,1);
            mc.internalStates.XEst = mc.internalStates.X;
            mc.internalStates.U = zeros(m,1);
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

            stateDependentNoise = 0;
            for isdn = 1:size(D,3)
                stateDependentNoise = stateDependentNoise + randn*D(:,:,isdn)*mc.internalStates.X;
            end

            yz = yz + sensoryNoise + stateDependentNoise;
            
            i = mc.internalStates.timeIndex;
            A = mc.ofc.systemEq.A;
            B = mc.ofc.systemEq.B;
            H = mc.ofc.systemEq.H;
            K = mc.param.K;
            XEst_new = A(:,:,i)*mc.internalStates.XEst + B*mc.internalStates.U + K(:,:,i)*(yz-H*mc.internalStates.XEst) + internalNoise;
            mc.internalStates.XEst = XEst_new; 

        end

        %% UPDATE INTERNAL MODEL STATES
        function [X_new, mc] = updateInternalModel(mc)
            C = mc.ofc.noiseConstructors.controlDependentConstructor;
            n = mc.ofc.systemEq.numberOfStates;
            Omega_xi = mc.ofc.noiseConstructors.additiveProcessNoiseCovar;
            
            processNoise = mvnrnd(zeros(n,1),Omega_xi)';
            controlDependentNoise = 0;
            for icdn = 1:size(C,3)
                controlDependentNoise = controlDependentNoise + randn*C(:,:,icdn)*mc.internalStates.U;
            end

            A_sim = mc.ofc.systemEq.A_sim;
            B = mc.ofc.systemEq.B;
            i = mc.internalStates.timeIndex;

            X_new = A_sim(:,:,i)*mc.internalStates.X + B*mc.internalStates.U + processNoise + controlDependentNoise;
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
            y = [handPos; handVel; mc.internalStates.X(5:6); Fpert];
            
            % reading from the internal model
%             H = mc.ofc.systemEq.H;
%             y = H*mc.internalStates.X;
        end
    end
end