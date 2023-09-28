classdef Human

    properties
        generalParamSet
%         ofc % todo: replace this with a class "MotorControl"
%         ofcParamSet
        mc
        msk
        results
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

            hmn.msk = MusculoskeletalArm(thetaEL_0, omegaEL_0, thetaSH_0, omegaSH_0, a_0, armDamping, pathToSynergyData);           
            
            startPosition = -targetPos_rel;
%             hmn = hmn.setupOFC(dt,tEnd, numberOfStationarySteps, startPosition, Fpert);
            hmn.mc = MotorController(dt,tEnd, numberOfStationarySteps, startPosition, Fpert);

            hmn.generalParamSet.initialHandPos = hmn.msk.getHandPosition(thetaSH_0,thetaEL_0);
            hmn.generalParamSet.targetPos_abs = targetPos_rel + hmn.generalParamSet.initialHandPos;

            hmn = setupResultsStruct(hmn);
        end
        
        %% SETUP RESULTS STRUCT
        function hmn = setupResultsStruct(hmn)
            nStep = hmn.generalParamSet.nStep;
            dt = hmn.generalParamSet.dt;

            % MSK results
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


            % MotorControl Results
            n = hmn.mc.ofc.systemEq.numberOfStates;
            m = hmn.mc.ofc.systemEq.numberOfControls;
            h = hmn.mc.ofc.simSetting.delay;

            hmn.results.ofc_currentX = repmat(hmn.mc.ofc.simSetting.xInit,h+1,1);
            hmn.results.ofc_Xdata = zeros(nStep,n);
            hmn.results.ofc_Xdata(1,:) = hmn.results.ofc_currentX';

            hmn.results.ofc_currentXEst = hmn.results.ofc_currentX;
            hmn.results.ofc_XEstdata = hmn.results.ofc_Xdata;
            
            hmn.results.ofc_currentU = zeros(m,1);
            hmn.results.ofc_Udata = zeros(nStep-1,m);

            hmn.results.ofc_t = (0:nStep-1)*dt;

        end

        %% SIMULATE HUMAN MODEL
        function results = simulateHuman(hmn, plotResults)
            if plotResults
                figure("name","hand path")
                hold all
                axis equal
            end
            nStep = hmn.generalParamSet.nStep;
            dt = hmn.generalParamSet.dt;

            n = hmn.mc.ofc.systemEq.numberOfStates;
            m = hmn.mc.ofc.systemEq.numberOfControls;
            p = hmn.mc.ofc.systemEq.numberOfOutputs;

            A = hmn.mc.ofc.systemEq.A;
            A_sim = hmn.mc.ofc.systemEq.A_sim;
            B = hmn.mc.ofc.systemEq.B;
            H = hmn.mc.ofc.systemEq.H;

            C = hmn.mc.ofc.noiseConstructors.controlDependentConstructor;
            D = hmn.mc.ofc.noiseConstructors.stateDependentConstructor;
            Omega_xi = hmn.mc.ofc.noiseConstructors.additiveProcessNoiseCovar;
            Omega_omega = hmn.mc.ofc.noiseConstructors.sensoryNoiseCovar;
            Omega_eta = hmn.mc.ofc.noiseConstructors.internalNoiseCovar;
            
            L = hmn.mc.param.L;
            K = hmn.mc.param.K;


            armDamping = hmn.msk.param.armDamping;
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


                hmn.results.ofc_currentU = -L(:,:,i)*hmn.results.ofc_currentXEst;

                controlDependentNoise = 0;
                for icdn = 1:m
                    controlDependentNoise = controlDependentNoise + randn*C(:,:,icdn)*hmn.results.ofc_currentU;
                end

                newX_OFC = A_sim(:,:,i)*hmn.results.ofc_currentX + B*hmn.results.ofc_currentU + processNoise + controlDependentNoise;
                diff_X_ofc = (newX_OFC - hmn.results.ofc_currentX)/dt;
                hmn.results.ofc_currentX = newX_OFC;
                hmn.results.ofc_Xdata(i+1,:) = hmn.results.ofc_currentX';


                %%%%%%% Generate high-level commands

                a_ref_ofc = diff_X_ofc(3:4); % this works better
                %     a_ref_ofc = diff_XEst_ofc(3:4);
%                 F_ref_ofc = hmn.ofcResults.currentX(5:6);
                %     a_ref_ofc = hmn.ofcResults.currentU;

                %%%%%%% MSK's step
                mskOutputs = hmn.msk.getOutputs(hmn.results.msk_currentY);
                q_SH = mskOutputs.q(1);
                q_EL = mskOutputs.q(2);
                qdot_SH = mskOutputs.qdot(1);
                qdot_EL = mskOutputs.qdot(2);

                Fpert_est = hmn.results.ofc_currentXEst(7:8);

%                 syn = interpn(1:Nmuscle,1:NoSyn,thetaSHset,thetaELset,  synergySet  ,1:Nmuscle,1:NoSyn,q_SH,q_EL);
                syn = hmn.msk.interpolateSynergies(q_SH,q_EL);
                [basis_acc,basis_F] = hmn.msk.calcBasis(syn, q_SH,q_EL);

                a_ref_tilde = hmn.msk.CorrectAccRef(a_ref_ofc,q_SH,q_EL,qdot_SH,qdot_EL,Fpert_est);

                coeff = lsqnonneg(basis_acc,a_ref_tilde);
                % coeff = lsqnonneg(basis_F,F_ref_ofc);
                hmn.results.msk_currentU = syn*coeff;
                hmn.results.msk_currentU = min(1,max(0,hmn.results.msk_currentU));
                hmn.results.msk_currentU = hmn.results.msk_currentU .* (1+0.02*randn(size(hmn.results.msk_currentU))); % todo: fix this noise


                [hmn.results.msk_currentXdot, ~, hmn.results.msk_currentY] = PlanarArm(0,hmn.results.msk_currentX,[Fpert(2);Fpert(1);hmn.results.msk_currentU],armDamping);
                
                hmn.results.msk_Udata(i,:) = hmn.results.msk_currentU';
                hmn.results.msk_Ydata(i,:) = hmn.results.msk_currentY';
                hmn.results.msk_Xdata(i,:) = hmn.results.msk_currentX';
                hmn.results.msk_currentX = hmn.results.msk_currentX + hmn.results.msk_currentXdot * dt;

                %%%%%%% Estimator's integration step
                mskOutputs = hmn.msk.getOutputs(hmn.results.msk_currentY);
                handPos = mskOutputs.hand_p - hmn.generalParamSet.targetPos_abs;
                handVel = mskOutputs.hand_v;

                stateDependentNoise = 0;
                for isdn = 1:size(D,3)
                    stateDependentNoise = stateDependentNoise + randn*D(:,:,isdn)*hmn.results.ofc_currentX; 
                end
                yz = [handPos; handVel; hmn.results.ofc_currentX(5:6); Fpert] + sensoryNoise + stateDependentNoise;

                newXEst_ofc = A(:,:,i)*hmn.results.ofc_currentXEst + B*hmn.results.ofc_currentU + K(:,:,i)*(yz-H*hmn.results.ofc_currentXEst) + internalNoise;
                diff_XEst_ofc = (newXEst_ofc - hmn.results.ofc_currentXEst)/dt;
                hmn.results.ofc_XEstdata(i,:) = hmn.results.ofc_currentXEst';
                hmn.results.ofc_currentXEst = newXEst_ofc; %TODO check i or i+1, both here and above

                %%%%%%% plot this arm path
                if plotResults && mod(i,10)==1
                    cmap = copper(nStep);
                    [hnd, elb] = hmn.msk.getHandPosition(q_SH,q_EL);
                    plot([0,elb(1),hnd(1)], [0,elb(2),hnd(2)],'-o','LineWidth',2,'Color',cmap(i,:))
                    scatter(hmn.results.ofc_currentX(1)+hmn.generalParamSet.targetPos_abs(1), hmn.results.ofc_currentX(2)+hmn.generalParamSet.targetPos_abs(2),'s','filled')
                    scatter(hmn.results.ofc_currentXEst(1)+hmn.generalParamSet.targetPos_abs(1), hmn.results.ofc_currentXEst(2)+hmn.generalParamSet.targetPos_abs(2),100,'x','MarkerEdgeColor','r','LineWidth',2)
                end

            end

            hmn.results.ofc_Xdata = hmn.results.ofc_Xdata(:,1:hmn.mc.ofc.systemEq.numberOfOriginalStates);
            hmn.results.ofc_XEstdata = hmn.results.ofc_XEstdata(:,1:hmn.mc.ofc.systemEq.numberOfOriginalStates);

            results = hmn.results;
        end

    %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % END OF METHODS
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end
end