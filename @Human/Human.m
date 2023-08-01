classdef Human
    %UNTITLED6 Summary of this class goes here
    %   Detailed explanation goes here

    properties
        ofc
        ofcParamSet
        msk
        mskParamSet
    end

    methods
        function obj = Human(inputArg1,inputArg2)
            %UNTITLED6 Construct an instance of this class
            %   Detailed explanation goes here
            obj.ofc = inputArg1 + inputArg2;
        end

        function obj = setOFC(obj,inputArg)
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
        end
    end
end