classdef MusculoskeletalArm < matlab.mixin.Copyable
    properties
        param
        X
        Y
        XDOT
        U
        environment
        IC
    end

    methods
        function msk = MusculoskeletalArm(thetaEL_0, omegaEL_0, thetaSH_0, omegaSH_0, a_0, armDamping, pathToSynergy, Fhand0, withRobot, runTimeNoiseFactor)
            msk.param = getParametersPlanarArm();
            msk.param.armDamping = armDamping;
            msk.param.Nmuscle = 6;
            msk.param.controlDependentNoise = runTimeNoiseFactor*0.01;
            msk.param.withRobot = withRobot;

            [IC,Y0,XDOT0] = msk.getIC(thetaEL_0, omegaEL_0, thetaSH_0, omegaSH_0, a_0);
            msk.IC = IC;
            msk.X = IC;
            msk.Y = Y0;
            msk.XDOT = XDOT0;
            msk.U = zeros(msk.param.Nmuscle,1);
            msk.environment.Fhand = Fhand0;
            msk.environment.robotTorque = [0;0];

            msk = loadSynergies(msk,pathToSynergy);
        end

        %% SOLVE INITIAL CONDITION
        function [IC,Y,XDOT] = getIC(msk, thetaEL_0, omegaEL_0, thetaSH_0, omegaSH_0, a_0, robotStates)
            if ~msk.param.withRobot
                [IC,Y,XDOT] = getPlanarArmIC(thetaEL_0, omegaEL_0, thetaSH_0, omegaSH_0, a_0, msk.param.armDamping);
            else
                if exist('robotStates','var')
                    [IC,Y,XDOT] = getPlanarArmAndRobotIC(thetaEL_0, omegaEL_0, thetaSH_0, omegaSH_0, a_0, msk.param.armDamping, robotStates);
                else
                    [IC,Y,XDOT] = getPlanarArmAndRobotIC(thetaEL_0, omegaEL_0, thetaSH_0, omegaSH_0, a_0, msk.param.armDamping);
                end
            end
        end

        %% PARSE OUTPUT VECTOR
        function outputs = getOutputs(msk,Y)
            if ~exist('Y','var')
                Y = msk.Y;
            end
            
            if ~msk.param.withRobot
                outputs = getPlanarArmOutputs(Y);
            else
                outputs = getPlanarArmAndRobotOutputs(Y);
            end

        end

        %% PARSE STATE VECTOR
        function outputs = getStates(msk,X)
            if ~exist('X','var')
                X = msk.X;
            end
            if ~msk.param.withRobot
                outputs = getPlanarArmStates(X);
            else
                outputs = getPlanarArmAndRobotStates(X);
            end
        end
        %% GET Q_DDOT
        function [qddot_arm, qddot_robot] = getqddot(msk,XDOT)
            if ~exist('XDOT','var')
                XDOT = msk.XDOT;
            end
            if ~msk.param.withRobot
                qddot_arm = getPlanarArmQddot(XDOT);
                qddot_robot = [0;0];
            else
                [qddot_arm, qddot_robot] = getPlanarArmAndRobotQddot(XDOT);
            end

        end

        %% CALCULATE HAND POSITION
        function [hand, elbow] = getHandPosition(msk, q_SH,q_EL)
            shoulder = [0;0];
            elbow = shoulder + [msk.param.a1.*cos(q_SH); msk.param.a1.*sin(q_SH)];
            hand = elbow + [msk.param.a2.*cos(q_SH+q_EL); msk.param.a2.*sin(q_SH+q_EL)];
        end
        %% CALCULATE ROBOT JOINT POSITIONS
%         function [hand, elbow] = getRobotPosition(msk, q_SH,q_EL)
%             shoulder = [0;0];
%             elbow = shoulder + [msk.param.a1.*cos(q_SH); msk.param.a1.*sin(q_SH)];
%             hand = elbow + [msk.param.a2.*cos(q_SH+q_EL); msk.param.a2.*sin(q_SH+q_EL)];
%         end
        %% SOLVE MSK MODEL
        % TODO FIX: there is a mismatch between the y and x. x is one step
        % ahead of y. 
        function [xdot,x_new,y_new, msk] = solveModel(msk, dt)
            x = msk.X;
            u = msk.U;
            F_hand = msk.environment.Fhand;
            armDamping = msk.param.armDamping;

            if ~msk.param.withRobot
                [xdot,~,y_new] =PlanarArm(0,x,[F_hand(2);F_hand(1);u],armDamping);
            else
                tau = msk.environment.robotTorque;
                [xdot,~,y_new] =PlanarArmAndRobot(0,x,[F_hand(2);F_hand(1);u;tau],armDamping);
            end
            
            x_new = x + xdot * dt;
            msk.X = x_new;            
            msk.XDOT = xdot;
            msk.Y = y_new;
        end

        %% CALUCALTE TORQUE-EQUIVALENCE OF MUSCLE INPUTS
        % TODO INCOMPLETE
%         function tau = calculateMuscleTorques(msk, u, q, qdot, F, param)
% 
%             X = getPlanarArmIC(qEL, qdotEL, qSH, qdotSH, u, param);
%             [Xdot, ~, ~] = PlanarArm(0,X,[F(2);F(1);u],param);
% 
%             qddot = getPlanarArmQddot(Xdot);
%             sys = armParameters();
% 
% 
%             M = sysEQ_massMatrix([q;qdot], sys);
%             C = sysEQ_CMatrix([q;qdot],sys);
%             J = sysEQ_J([q;qdot],sys);
% 
%             tau = M*qddot + C*qdot - J'*F;
%         end

        %% LOAD SYNERGIES
        function msk = loadSynergies(msk,pathToSynergy)
            data = load(pathToSynergy);
            msk.param.synergySet = data.synergySet;
            msk.param.thetaELset = data.thetaELset;
            msk.param.thetaSHset= data.thetaSHset;
            
            msk.param.NoSyn = size(data.synergySet,2);
        end
        %% GET SYNERGIES AT CURRENT STATE
        function syn = interpolateSynergies(msk, q_SH,q_EL)
            NoSyn = msk.param.NoSyn;
            Nmuscle = msk.param.Nmuscle;
            thetaSHset = msk.param.thetaSHset;
            thetaELset = msk.param.thetaELset;
            synergySet = msk.param.synergySet;
            syn = interpn(1:Nmuscle,1:NoSyn,thetaSHset,thetaELset,  synergySet  ,1:Nmuscle,1:NoSyn,q_SH,q_EL);
            
        end
        %% CALCULATE BASIS VECTORS
        function [basis_acc, basis_F] = calcBasis(msk, synergy, qSH, qEL)
            qdotSH = 0; % must be zero to maintain linearity (i,e, double activations gives double F or acc) and should be corrected nonlinearly later
            qdotEL = 0;

            NoSyn = size(synergy,2);
            basis_acc = zeros(2,NoSyn);
            basis_F = zeros(2,NoSyn);

            for i=1:NoSyn
                u = synergy(:,i);
                states = getPlanarArmIC(qEL, qdotEL, qSH, qdotSH, u, msk.param.armDamping);
                [~, ~, y] = PlanarArm(0,states,[0;0;u],msk.param.armDamping);
                basis_acc(:,i) = getPlanarArmOutputs(y).hand_a;
                basis_F(:,i) = calculateTaskSpaceForce(u, [qSH; qEL], [qdotSH;qdotEL], [0;0], msk.param.armDamping);
            end
        end
        
        %% COMPENSATE FOR VELOCITIES IN REFERENCE ACCELERATION
        function a_tilde = CorrectAccRef(msk, a_ref,q_SH,q_EL,qdot_SH,qdot_EL,Fhand)
            % here Fhand is the external force onto the end
            q = [q_SH;q_EL];
            qdot = [qdot_SH;qdot_EL];
            states = [q;qdot];

            M = sysEQ_massMatrix(states,msk.param);
            J = sysEQ_J(states,msk.param);
            C = sysEQ_CMatrix(states,msk.param);
            Jdot = sysEQ_Jdot(states,msk.param);

            % a_tilde = a_ref - Jdot*qdot + J*(M\C)*qdot; % this was the original which seemed to work

            qddot_ref = J\(a_ref - Jdot*qdot);

            qddot_tilde = qddot_ref + M\(-J'*Fhand + C*qdot);

            a_tilde = 0*Jdot*qdot + J*qddot_tilde; % here Jdot*qdot isn't needed becuase the "nominal" condition do not have velocities
        end
        %% CALCULATE MUSCLE ACTIVITIES FROM REF ACC
        function [U, basis, coeff] = acceleration2activation(msk, Y, a_ref_mc, Fpert)
            cdn = msk.param.controlDependentNoise;
            mskOutputs = msk.getOutputs(Y);
            q_SH = mskOutputs.q(1);
            q_EL = mskOutputs.q(2);
            qdot_SH = mskOutputs.qdot(1);
            qdot_EL = mskOutputs.qdot(2);

            syn = msk.interpolateSynergies(q_SH, q_EL);
            basis = msk.calcBasis(syn, q_SH, q_EL);
            a_ref_tilde = msk.CorrectAccRef(a_ref_mc,q_SH,q_EL,qdot_SH,qdot_EL,Fpert);
            
            coeff = lsqnonneg(basis,a_ref_tilde);
            U = syn*coeff;
            U = min(1,max(0,U));
            U = U .* ( 1+cdn*randn(size(U)) );
            msk.U = U;
        end
        %% CALCULATE INTERACTION FORCE
        function F_int = getInteractionForce(msk)
            if msk.param.withRobot
                states = msk.getStates();
                q_r = states.q_robot;
                qdot_r = states.qdot_robot;
                [~,qddot_r] = msk.getqddot();

                robotParam = getParametersRobot();
                M = sysEQ_massMatrix(q_r,robotParam);
                C = sysEQ_CMatrix([q_r;qdot_r],robotParam);
                J = sysEQ_J(q_r,robotParam);
                tau = msk.environment.robotTorque;
                F_int = (J')\(M*qddot_r + C*qdot_r - tau);
            else
                F_int = [0;0];
            end
        end
        %% RESET TO INITIAL CONDITION
        function resetStates(msk, thetaEL_0, omegaEL_0, thetaSH_0, omegaSH_0, a_0, robotStates)
            if exist('robotStates','var')
                [msk.IC,msk.Y,msk.XDOT] = msk.getIC(thetaEL_0, omegaEL_0, thetaSH_0, omegaSH_0, a_0, robotStates);
            else
                [msk.IC,msk.Y,msk.XDOT] = msk.getIC(thetaEL_0, omegaEL_0, thetaSH_0, omegaSH_0, a_0);
            end
            msk.X = msk.IC;
            msk.U = zeros(msk.param.Nmuscle,1);
        end

    end


end
