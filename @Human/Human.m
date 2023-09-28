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
        function hmn = Human(dt, tEnd, thetaEL_0, omegaEL_0, thetaSH_0, omegaSH_0, a_0, targetPos_rel, armDamping, numberOfStationarySteps, Fpert, pathToSynergyData)
            hmn.generalParamSet.dt = dt;
            hmn.generalParamSet.tEnd = tEnd;
            hmn.generalParamSet.nStep = round(tEnd/dt) + numberOfStationarySteps;
            hmn.generalParamSet.Fpert = Fpert;
            hmn.generalParamSet.targetPos_rel = targetPos_rel;

            hmn = hmn.setupMSK(thetaEL_0, omegaEL_0, thetaSH_0, omegaSH_0, a_0, armDamping, pathToSynergyData);
            
            startPosition = -targetPos_rel;
            hmn = hmn.setupOFC(dt,tEnd, numberOfStationarySteps, startPosition, Fpert);

        end
        
        %% SETUP MSK
        function hmn = setupMSK(hmn, thetaEL_0, omegaEL_0, thetaSH_0, omegaSH_0, a_0, armDamping, pathToSynergyData)
            hmn.mskParamSet = getParametersPlanarArm();
            hmn.mskParamSet.Nmuscle = 6;
            hmn.mskParamSet.armDamping =  armDamping; % [Elbow damping; Shoulder damping]

            hmn.generalParamSet.initialHandPos = hmn.getHandPosition(thetaSH_0,thetaEL_0);
            hmn.generalParamSet.targetPos_abs = hmn.generalParamSet.targetPos_rel + hmn.generalParamSet.initialHandPos;

            [IC,Y] = hmn.getIC_msk(thetaEL_0, omegaEL_0, thetaSH_0, omegaSH_0, a_0);

            hmn.msk.IC = IC;

            hmn.mskResults.currentX = IC;
            hmn.mskResults.currentXdot = zeros(size(IC));
            hmn.mskResults.Xdata = nan(hmn.generalParamSet.nStep, length(IC));
            hmn.mskResults.Xdata(1,:) = IC';

            hmn.mskResults.currentY = Y;
            hmn.mskResults.Ydata = nan(hmn.generalParamSet.nStep, length(Y));
            hmn.mskResults.Ydata(1,:) = Y(:,1)';

            hmn.mskResults.currentU = zeros(hmn.mskParamSet.Nmuscle,1);
            hmn.mskResults.Udata = nan(hmn.generalParamSet.nStep-1, hmn.mskParamSet.Nmuscle);

            hmn.mskResults.t = (0:hmn.generalParamSet.nStep-1)*hmn.generalParamSet.dt;

            hmn = hmn.loadSynergies(pathToSynergyData);

        end

        %% GET MSK IC
        function [IC,Y] = getIC_msk(hmn, thetaEL_0, omegaEL_0, thetaSH_0, omegaSH_0, a_0)
            armDamping = hmn.mskParamSet.armDamping;
            [IC,Y] = getPlanarArmIC(thetaEL_0, omegaEL_0, thetaSH_0, omegaSH_0, a_0, armDamping);            
        end

        %% GET MSK outputs
        function outputs = getOutputs_msk(hmn,Y)
            if exist('Y','var')
                outputs = getPlanarArmOutputs(Y);
            else
                outputs = getPlanarArmOutputs(hmn.mskResults.Ydata);
            end
        end

        %% SOLVE MSK MODEL 
        function [Xdot,X,Y] = solveModel_msk(hmn,F_hand)
            if ~exist('F_hand','var')
                F_hand = [0;0];
            end

            X = hmn.mskResults.currentX;
            u = hmn.mskResults.currentU;
            armDamping = hmn.mskParamSet.armDamping;

            [Xdot,X,Y] =PlanarArm(0,X,[F_hand;u],armDamping);
            hmn.mskResults.currentX = X;
            hmn.mskResults.currentXdot = Xdot;
        end

        %% LOAD SYNERGIES
        function hmn = loadSynergies(hmn,pathToSynergy)
            data = load(pathToSynergy);
            hmn.mskParamSet.synergySet = data.synergySet;
            hmn.mskParamSet.thetaELset = data.thetaELset;
            hmn.mskParamSet.thetaSHset= data.thetaSHset;
            
            hmn.mskParamSet.NoSyn = size(data.synergySet,2);
        end

        %% SETUP OFC
        function hmn = setupOFC(hmn,dt,tEnd, numberOfStationarySteps, startPosition, Fpert)
            [A,B,H,Q,R,simSetting,noiseStructure,xInit,modelParam] = setupInternalModel_pointMass_effort(startPosition, dt, tEnd, numberOfStationarySteps,Fpert);
            hmn.ofc = OFC(A,B,H,Q,R,simSetting,noiseStructure);
            hmn.ofcParamSet = modelParam;

            [hmn.ofcParamSet.L,hmn.ofcParamSet.K] = hmn.ofc.getOptimalGains;
            
            n = hmn.ofc.systemEq.numberOfStates;
            m = hmn.ofc.systemEq.numberOfControls;
            h = hmn.ofc.simSetting.delay;

            hmn.ofcResults.currentX = repmat(xInit,h+1,1);
            hmn.ofcResults.Xdata = zeros(simSetting.nStep,n);
            hmn.ofcResults.Xdata(1,:) = hmn.ofcResults.currentX';

            hmn.ofcResults.currentXEst = hmn.ofcResults.currentX;
            hmn.ofcResults.XEstdata = hmn.ofcResults.Xdata;
            
            hmn.ofcResults.currentU = zeros(m,1);
            hmn.ofcResults.Udata = zeros(simSetting.nStep-1,m);

            hmn.ofcResults.t = (0:simSetting.nStep-1)*dt;
        end

        %% GET HAND POSITION FROM JOINT ANGLES
        function [hand, elbow] = getHandPosition(hmn, q_SH,q_EL)

            shoulder = [0;0];
            elbow = shoulder + [hmn.mskParamSet.a1.*cos(q_SH); hmn.mskParamSet.a1.*sin(q_SH)];
            hand = elbow + [hmn.mskParamSet.a2.*cos(q_SH+q_EL); hmn.mskParamSet.a2.*sin(q_SH+q_EL)];
        end

        %% COMPENSATE FOR VELOCITIES IN REFERENCE ACCELERATION
        function a_tilde = CorrectAccRef(hmn, a_ref,q_SH,q_EL,qdot_SH,qdot_EL,Fhand)
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
        function [ofcResults,mskResults] = simulateHuman(hmn, plotResults)
            if plotResults
                figure("name","hand path")
                hold all
                axis equal
            end
            nStep = hmn.generalParamSet.nStep;
            dt = hmn.generalParamSet.dt;

            n = hmn.ofc.systemEq.numberOfStates;
            m = hmn.ofc.systemEq.numberOfControls;
            p = hmn.ofc.systemEq.numberOfOutputs;

            A = hmn.ofc.systemEq.A;
            A_sim = hmn.ofc.systemEq.A_sim;
            B = hmn.ofc.systemEq.B;
            H = hmn.ofc.systemEq.H;

            C = hmn.ofc.noiseConstructors.controlDependentConstructor;
            D = hmn.ofc.noiseConstructors.stateDependentConstructor;
            Omega_xi = hmn.ofc.noiseConstructors.additiveProcessNoiseCovar;
            Omega_omega = hmn.ofc.noiseConstructors.sensoryNoiseCovar;
            Omega_eta = hmn.ofc.noiseConstructors.internalNoiseCovar;
            
            L = hmn.ofcParamSet.L;
            K = hmn.ofcParamSet.K;

            NoSyn = hmn.mskParamSet.NoSyn;
            Nmuscle = hmn.mskParamSet.Nmuscle;
            thetaSHset = hmn.mskParamSet.thetaSHset;
            thetaELset = hmn.mskParamSet.thetaELset;
            synergySet = hmn.mskParamSet.synergySet;

            armDamping = hmn.mskParamSet.armDamping;
            Fpert = hmn.generalParamSet.Fpert;

            for i = 1:nStep-1
                %%%%%% OFC's integral step
                sensoryNoise = mvnrnd(zeros(p,1),Omega_omega)';
                processNoise = mvnrnd(zeros(n,1),Omega_xi)';
                internalNoise = mvnrnd(zeros(n,1),Omega_eta)';

                % use this to decouple OFC from MSK. Useful for OFC debugging
                %     stateDependentNoise = 0;
                %     for isdn = 1:size(D,3)
                %         stateDependentNoise = stateDependentNoise + randn*D(:,:,isdn)*hmn.ofcResults.Xdata(i,:)';
                %     end
                %     yz = H*hmn.ofcResults.currentX + sensoryNoise + stateDependentNoise;


                hmn.ofcResults.currentU = -L(:,:,i)*hmn.ofcResults.currentXEst;

                controlDependentNoise = 0;
                for icdn = 1:m
                    controlDependentNoise = controlDependentNoise + randn*C(:,:,icdn)*hmn.ofcResults.currentU;
                end

                newX_OFC = A_sim(:,:,i)*hmn.ofcResults.currentX + B*hmn.ofcResults.currentU + processNoise + controlDependentNoise;
                diff_X_ofc = (newX_OFC - hmn.ofcResults.currentX)/dt;
                hmn.ofcResults.currentX = newX_OFC;
                hmn.ofcResults.Xdata(i+1,:) = hmn.ofcResults.currentX';


                %%%%%%% Generate high-level commands

                a_ref_ofc = diff_X_ofc(3:4); % this works better
                %     a_ref_ofc = diff_XEst_ofc(3:4);
%                 F_ref_ofc = hmn.ofcResults.currentX(5:6);
                %     a_ref_ofc = hmn.ofcResults.currentU;

                %%%%%%% MSK's step
                q_SH = hmn.getOutputs_msk(hmn.mskResults.currentY).q(1);
                q_EL = hmn.getOutputs_msk(hmn.mskResults.currentY).q(2);
                qdot_SH = hmn.getOutputs_msk(hmn.mskResults.currentY).qdot(1);
                qdot_EL = hmn.getOutputs_msk(hmn.mskResults.currentY).qdot(2);

                Fpert_est = hmn.ofcResults.currentXEst(7:8);

                syn = interpn(1:Nmuscle,1:NoSyn,thetaSHset,thetaELset,  synergySet  ,1:Nmuscle,1:NoSyn,q_SH,q_EL);
                [basis_acc,basis_F] = calcBasis(syn, q_SH,q_EL,armDamping);

                a_ref_tilde = hmn.CorrectAccRef(a_ref_ofc,q_SH,q_EL,qdot_SH,qdot_EL,Fpert_est);

                coeff = lsqnonneg(basis_acc,a_ref_tilde);
                % coeff = lsqnonneg(basis_F,F_ref_ofc);
                hmn.mskResults.currentU = syn*coeff;
                hmn.mskResults.currentU = min(1,max(0,hmn.mskResults.currentU));
                hmn.mskResults.currentU = hmn.mskResults.currentU .* (1+0.02*randn(size(hmn.mskResults.currentU)));


                [hmn.mskResults.currentXdot, ~, hmn.mskResults.currentY] = PlanarArm(0,hmn.mskResults.currentX,[Fpert(2);Fpert(1);hmn.mskResults.currentU],armDamping);
                
                hmn.mskResults.Udata(i,:) = hmn.mskResults.currentU';
                hmn.mskResults.Ydata(i,:) = hmn.mskResults.currentY';
                hmn.mskResults.Xdata(i,:) = hmn.mskResults.currentX';
                hmn.mskResults.currentX = hmn.mskResults.currentX + hmn.mskResults.currentXdot * dt;

                %%%%%%% Estimator's integration step
                handPos = hmn.getOutputs_msk(hmn.mskResults.currentY).hand_p - hmn.generalParamSet.targetPos_abs;
                handVel = hmn.getOutputs_msk(hmn.mskResults.currentY).hand_v;

                stateDependentNoise = 0;
                for isdn = 1:size(D,3)
                    stateDependentNoise = stateDependentNoise + randn*D(:,:,isdn)*hmn.ofcResults.currentX; 
                end
                yz = [handPos; handVel; hmn.ofcResults.currentX(5:6); Fpert] + sensoryNoise + stateDependentNoise;

                newXEst_ofc = A(:,:,i)*hmn.ofcResults.currentXEst + B*hmn.ofcResults.currentU + K(:,:,i)*(yz-H*hmn.ofcResults.currentXEst) + internalNoise;
                diff_XEst_ofc = (newXEst_ofc - hmn.ofcResults.currentXEst)/dt;
                hmn.ofcResults.XEstdata(i,:) = hmn.ofcResults.currentXEst';
                hmn.ofcResults.currentXEst = newXEst_ofc; %TODO check i or i+1, both here and above

                %%%%%%% plot this arm path
                if plotResults && mod(i,10)==1
                    cmap = copper(nStep);
                    [hnd, elb] = hmn.getHandPosition(q_SH,q_EL);
                    plot([0,elb(1),hnd(1)], [0,elb(2),hnd(2)],'-o','LineWidth',2,'Color',cmap(i,:))
                    scatter(hmn.ofcResults.currentX(1)+hmn.generalParamSet.targetPos_abs(1), hmn.ofcResults.currentX(2)+hmn.generalParamSet.targetPos_abs(2),'s','filled')
                    scatter(hmn.ofcResults.currentXEst(1)+hmn.generalParamSet.targetPos_abs(1), hmn.ofcResults.currentXEst(2)+hmn.generalParamSet.targetPos_abs(2),100,'x','MarkerEdgeColor','r','LineWidth',2)
                end

            end

            hmn.ofcResults.Xdata = hmn.ofcResults.Xdata(:,1:hmn.ofc.systemEq.numberOfOriginalStates);
            hmn.ofcResults.XEstdata = hmn.ofcResults.XEstdata(:,1:hmn.ofc.systemEq.numberOfOriginalStates);

            ofcResults = hmn.ofcResults;
            mskResults = hmn.mskResults;
        end

    %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % END OF METHODS
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end
end