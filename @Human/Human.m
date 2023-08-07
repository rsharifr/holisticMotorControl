classdef Human

    properties
        ofc
        ofcParamSet
        ofcResults
        
        msk
        mskParamSet
        mskResults

        generalParamSet
    end

    methods
    %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % METHODS
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
        
        %% CONSTRUCTOR        
        function obj = Human(dt, tEnd, thetaEL_0, omegaEL_0, thetaSH_0, omegaSH_0, a_0, targetPos_rel, armDamping, numberOfStationarySteps, Fpert, pathToSynergyData)
            obj.generalParamSet.dt = dt;
            obj.generalParamSet.tEnd = tEnd;
            obj.generalParamSet.nStep = round(tEnd/dt) + numberOfStationarySteps;
            obj.generalParamSet.Fpert = Fpert;
            obj.generalParamSet.targetPos_rel = targetPos_rel;

            obj = obj.setupMSK(thetaEL_0, omegaEL_0, thetaSH_0, omegaSH_0, a_0, armDamping, pathToSynergyData);
            
            startPosition = -targetPos_rel;
            obj = obj.setupOFC(dt,tEnd, numberOfStationarySteps, startPosition, Fpert);

        end
        
        %% SETUP MSK
        function obj = setupMSK(obj, thetaEL_0, omegaEL_0, thetaSH_0, omegaSH_0, a_0, armDamping, pathToSynergyData)
            obj.mskParamSet = getParametersPlanarArm();
            obj.mskParamSet.Nmuscle = 6;
            obj.mskParamSet.armDamping =  armDamping; % [Elbow damping; Shoulder damping]

            obj.generalParamSet.initialHandPos = obj.getHandPosition(thetaSH_0,thetaEL_0);
            obj.generalParamSet.targetPos_abs = obj.generalParamSet.targetPos_rel + obj.generalParamSet.initialHandPos;

            [IC,Y] = obj.getIC_msk(thetaEL_0, omegaEL_0, thetaSH_0, omegaSH_0, a_0);

            obj.msk.IC = IC;

            obj.mskResults.currentX = IC;
            obj.mskResults.currentXdot = zeros(size(IC));
            obj.mskResults.Xdata = nan(obj.generalParamSet.nStep, length(IC));
            obj.mskResults.Xdata(1,:) = IC';

            obj.mskResults.currentY = Y;
            obj.mskResults.Ydata = nan(obj.generalParamSet.nStep, length(Y));
            obj.mskResults.Ydata(1,:) = Y(:,1)';

            obj.mskResults.currentU = zeros(obj.mskParamSet.Nmuscle,1);
            obj.mskResults.Udata = nan(obj.generalParamSet.nStep-1, obj.mskParamSet.Nmuscle);

            obj.mskResults.t = (0:obj.generalParamSet.nStep-1)*obj.generalParamSet.dt;

            obj = obj.loadSynergies(pathToSynergyData);

        end

        %% GET MSK IC
        function [IC,Y] = getIC_msk(obj, thetaEL_0, omegaEL_0, thetaSH_0, omegaSH_0, a_0)
            armDamping = obj.mskParamSet.armDamping;
            [IC,Y] = getPlanarArmIC(thetaEL_0, omegaEL_0, thetaSH_0, omegaSH_0, a_0, armDamping);            
        end

        %% GET MSK outputs
        function outputs = getOutputs_msk(obj,Y)
            if exist('Y','var')
                outputs = getPlanarArmOutputs(Y);
            else
                outputs = getPlanarArmOutputs(obj.mskResults.Ydata);
            end
        end

        %% SOLVE MSK MODEL 
        function [Xdot,X,Y] = solveModel_msk(obj,F_hand)
            if ~exist('F_hand','var')
                F_hand = [0;0];
            end

            X = obj.mskResults.currentX;
            u = obj.mskResults.currentU;
            armDamping = obj.mskParamSet.armDamping;

            [Xdot,X,Y] =PlanarArm(0,X,[F_hand;u],armDamping);
            obj.mskResults.currentX = X;
            obj.mskResults.currentXdot = Xdot;
        end

        %% LOAD SYNERGIES
        function obj = loadSynergies(obj,pathToSynergy)
            data = load(pathToSynergy);
            obj.mskParamSet.synergySet = data.synergySet;
            obj.mskParamSet.thetaELset = data.thetaELset;
            obj.mskParamSet.thetaSHset= data.thetaSHset;
            
            obj.mskParamSet.NoSyn = size(data.synergySet,2);
        end

        %% SETUP OFC
        function obj = setupOFC(obj,dt,tEnd, numberOfStationarySteps, startPosition, Fpert)
            [A,B,H,Q,R,simSetting,noiseStructure,xInit,modelParam] = setupOFCmodel_pointMass_effort(startPosition, dt, tEnd, numberOfStationarySteps,Fpert);
            obj.ofc = OFC(A,B,H,Q,R,simSetting,noiseStructure);
            obj.ofcParamSet = modelParam;

            [obj.ofcParamSet.L,obj.ofcParamSet.K] = obj.ofc.getOptimalGains;
            
            n = obj.ofc.systemEq.numberOfStates;
            m = obj.ofc.systemEq.numberOfControls;
            h = obj.ofc.simSetting.delay;

            obj.ofcResults.currentX = repmat(xInit,h+1,1);
            obj.ofcResults.Xdata = zeros(simSetting.nStep,n);
            obj.ofcResults.Xdata(1,:) = obj.ofcResults.currentX';

            obj.ofcResults.currentXEst = obj.ofcResults.currentX;
            obj.ofcResults.XEstdata = obj.ofcResults.Xdata;
            
            obj.ofcResults.currentU = zeros(m,1);
            obj.ofcResults.Udata = zeros(simSetting.nStep-1,m);

            obj.ofcResults.t = (0:simSetting.nStep-1)*dt;
        end

        %%
        function [hand, elbow] = getHandPosition(obj, q_SH,q_EL)

            shoulder = [0;0];
            elbow = shoulder + [obj.mskParamSet.a1.*cos(q_SH); obj.mskParamSet.a1.*sin(q_SH)];
            hand = elbow + [obj.mskParamSet.a2.*cos(q_SH+q_EL); obj.mskParamSet.a2.*sin(q_SH+q_EL)];
        end

        %%
        function a_tilde = CorrectAccRef(obj, a_ref,q_SH,q_EL,qdot_SH,qdot_EL,Fhand)
            % here Fhand is the external force onto the end
            sys = getParametersPlanarArm;
            q = [q_SH;q_EL];
            qdot = [qdot_SH;qdot_EL];
            X = [q;qdot];

            M = sysEQ_massMatrix(X,sys);
            J = sysEQ_J(X,sys);
            C = sysEQ_CMatrix(X,sys);
            Jdot = sysEQ_Jdot(X,sys);

            % a_tilde = a_ref - Jdot*qdot + J*(M\C)*qdot; % this was the original which seemed to work

            qddot_ref = J\(a_ref - Jdot*qdot);

            qddot_tilde = qddot_ref + M\(-J'*Fhand + C*qdot);

            a_tilde = 0*Jdot*qdot + J*qddot_tilde; % here Jdot*qdot isn't needed becuase the "nominal" condition do not have velocities
        end

        %% SIMULATE HUMAN MODEL
        function [ofcResults,mskResults] = simulateHuman(obj, plotResults)
            if plotResults
                figure("name","hand path")
                hold all
                axis equal
            end
            nStep = obj.generalParamSet.nStep;
            dt = obj.generalParamSet.dt;

            n = obj.ofc.systemEq.numberOfStates;
            m = obj.ofc.systemEq.numberOfControls;
            p = obj.ofc.systemEq.numberOfOutputs;

            A = obj.ofc.systemEq.A;
            A_sim = obj.ofc.systemEq.A_sim;
            B = obj.ofc.systemEq.B;
            H = obj.ofc.systemEq.H;

            C = obj.ofc.noiseConstructors.controlDependentConstructor;
            D = obj.ofc.noiseConstructors.stateDependentConstructor;
            Omega_xi = obj.ofc.noiseConstructors.additiveProcessNoiseCovar;
            Omega_omega = obj.ofc.noiseConstructors.sensoryNoiseCovar;
            Omega_eta = obj.ofc.noiseConstructors.internalNoiseCovar;
            
            L = obj.ofcParamSet.L;
            K = obj.ofcParamSet.K;

            NoSyn = obj.mskParamSet.NoSyn;
            Nmuscle = obj.mskParamSet.Nmuscle;
            thetaSHset = obj.mskParamSet.thetaSHset;
            thetaELset = obj.mskParamSet.thetaELset;
            synergySet = obj.mskParamSet.synergySet;

            armDamping = obj.mskParamSet.armDamping;
            Fpert = obj.generalParamSet.Fpert;

            for i = 1:nStep-1
                %%%%%% OFC's integral step
                sensoryNoise = mvnrnd(zeros(p,1),Omega_omega)';
                processNoise = mvnrnd(zeros(n,1),Omega_xi)';
                internalNoise = mvnrnd(zeros(n,1),Omega_eta)';

                % use this to decouple OFC from MSK. good for OFC debugging
                %     stateDependentNoise = 0;
                %     for isdn = 1:size(D,3)
                %         stateDependentNoise = stateDependentNoise + randn*D(:,:,isdn)*obj.ofcResults.Xdata(i,:)';
                %     end
                %     yz = H*obj.ofcResults.currentX + sensoryNoise + stateDependentNoise;


                obj.ofcResults.currentU = -L(:,:,i)*obj.ofcResults.currentXEst;

                controlDependentNoise = 0;
                for icdn = 1:m
                    controlDependentNoise = controlDependentNoise + randn*C(:,:,icdn)*obj.ofcResults.currentU;
                end

                newX_OFC = A_sim(:,:,i)*obj.ofcResults.currentX + B*obj.ofcResults.currentU + processNoise + controlDependentNoise;
                diff_X_ofc = (newX_OFC - obj.ofcResults.currentX)/dt;
                obj.ofcResults.currentX = newX_OFC;
                obj.ofcResults.Xdata(i+1,:) = obj.ofcResults.currentX';


                %%%%%%% Generate high-level commands

                a_ref_ofc = diff_X_ofc(3:4); % this works better
                %     a_ref_ofc = diff_XEst_ofc(3:4);
%                 F_ref_ofc = obj.ofcResults.currentX(5:6);
                %     a_ref_ofc = obj.ofcResults.currentU;
                %%%%%%% MSK's step
                q_SH = obj.getOutputs_msk(obj.mskResults.currentY).q(1);
                q_EL = obj.getOutputs_msk(obj.mskResults.currentY).q(2);
                qdot_SH = obj.getOutputs_msk(obj.mskResults.currentY).qdot(1);
                qdot_EL = obj.getOutputs_msk(obj.mskResults.currentY).qdot(2);

                Fpert_est = obj.ofcResults.currentXEst(7:8);

                syn = interpn(1:Nmuscle,1:NoSyn,thetaSHset,thetaELset,  synergySet  ,1:Nmuscle,1:NoSyn,q_SH,q_EL);
                [basis_acc,basis_F] = calcBasis(syn, q_SH,q_EL,armDamping);

                a_ref_tilde = obj.CorrectAccRef(a_ref_ofc,q_SH,q_EL,qdot_SH,qdot_EL,Fpert_est);

                coeff = lsqnonneg(basis_acc,a_ref_tilde);
                % coeff = lsqnonneg(basis_F,F_ref_ofc);
                obj.mskResults.currentU = syn*coeff;
                obj.mskResults.currentU = min(1,max(0,obj.mskResults.currentU));
                obj.mskResults.currentU = obj.mskResults.currentU .* (1+0.05*randn(size(obj.mskResults.currentU)));


                [obj.mskResults.currentXdot, ~, obj.mskResults.currentY] = PlanarArm(0,obj.mskResults.currentX,[Fpert(2);Fpert(1);obj.mskResults.currentU],armDamping);
                
                obj.mskResults.Udata(i,:) = obj.mskResults.currentU';
                obj.mskResults.Ydata(i,:) = obj.mskResults.currentY';
                obj.mskResults.Xdata(i,:) = obj.mskResults.currentX';
                obj.mskResults.currentX = obj.mskResults.currentX + obj.mskResults.currentXdot * dt;

                %%%%%%% Estimator's integration step
                handPos = obj.getOutputs_msk(obj.mskResults.currentY).hand_p - obj.generalParamSet.targetPos_abs;
                handVel = obj.getOutputs_msk(obj.mskResults.currentY).hand_v;

                stateDependentNoise = 0;
                for isdn = 1:size(D,3)
                    stateDependentNoise = stateDependentNoise + randn*D(:,:,isdn)*obj.ofcResults.currentX; 
                end
                yz = [handPos; handVel; obj.ofcResults.currentX(5:6); Fpert] + sensoryNoise + stateDependentNoise;

                newXEst_ofc = A(:,:,i)*obj.ofcResults.currentXEst + B*obj.ofcResults.currentU + K(:,:,i)*(yz-H*obj.ofcResults.currentXEst) + internalNoise;
                diff_XEst_ofc = (newXEst_ofc - obj.ofcResults.currentXEst)/dt;
                obj.ofcResults.XEstdata(i,:) = obj.ofcResults.currentXEst';
                obj.ofcResults.currentXEst = newXEst_ofc; %TODO check i or i+1, both here and above

                %%%%%%% plot this arm path
                if plotResults && mod(i,10)==1
                    cmap = copper(nStep);
                    [hnd, elb] = obj.getHandPosition(q_SH,q_EL);
                    plot([0,elb(1),hnd(1)], [0,elb(2),hnd(2)],'-o','LineWidth',2,'Color',cmap(i,:))
                    scatter(obj.ofcResults.currentX(1)+obj.generalParamSet.targetPos_abs(1), obj.ofcResults.currentX(2)+obj.generalParamSet.targetPos_abs(2),'s','filled')
                    scatter(obj.ofcResults.currentXEst(1)+obj.generalParamSet.targetPos_abs(1), obj.ofcResults.currentXEst(2)+obj.generalParamSet.targetPos_abs(2),100,'x','MarkerEdgeColor','r','LineWidth',2)
                end

            end

            obj.ofcResults.Xdata = obj.ofcResults.Xdata(:,1:obj.ofc.systemEq.numberOfOriginalStates);
            obj.ofcResults.XEstdata = obj.ofcResults.XEstdata(:,1:obj.ofc.systemEq.numberOfOriginalStates);

            ofcResults = obj.ofcResults;
            mskResults = obj.mskResults;
        end

    %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % END OF METHODS
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end
end