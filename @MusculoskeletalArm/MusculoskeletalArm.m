classdef MusculoskeletalArm
    properties
        param
        X
        Y
        XDOT
        U
        Fhand
        IC
    end

    methods
        function msk = MusculoskeletalArm(thetaEL_0, omegaEL_0, thetaSH_0, omegaSH_0, a_0, armDamping, pathToSynergy)
            msk.param = getParametersPlanarArm();
            msk.param.armDamping = armDamping;
            msk.param.Nmuscle = 6;

            [IC,Y0,XDOT0] = msk.getIC(thetaEL_0, omegaEL_0, thetaSH_0, omegaSH_0, a_0);
            msk.IC = IC;
            msk.X = IC;
            msk.Y = Y0;
            msk.XDOT = XDOT0;
            msk.U = zeros(msk.param.Nmuscle,1);
            msk.Fhand = zeros(2,1);

            msk = loadSynergies(msk,pathToSynergy);
        end

        %% SOLVE INITIAL CONDITION
        function [IC,Y,XDOT] = getIC(msk, thetaEL_0, omegaEL_0, thetaSH_0, omegaSH_0, a_0)
            [IC,Y,XDOT] = getPlanarArmIC(thetaEL_0, omegaEL_0, thetaSH_0, omegaSH_0, a_0, msk.param.armDamping);
        end

        %% PARSE OUTPUT VECTOR
        function outputs = getOutputs(msk,Y)
            if exist('Y','var')
                outputs = getPlanarArmOutputs(Y);
            else
                outputs = getPlanarArmOutputs(msk.Y);
            end
        end

        %% PARSE STATE VECTOR
        function outputs = getStates(msk,X)
            if exist('X','var')
                outputs = getPlanarArmStates(X);
            else
                outputs = getPlanarArmStates(msk.X);
            end
        end
        %% GET Q_DDOT
        function outputs = getqddot(msk,XDOT)
            if exist('XDOT','var')
                outputs = getPlanarArmQddot(XDOT);
            else
                outputs = getPlanarArmQddot(msk.XDOT);
            end
        end

        %% CALCULATE HAND POSITION
        function [hand, elbow] = getHandPosition(msk, q_SH,q_EL)
            shoulder = [0;0];
            elbow = shoulder + [msk.param.a1.*cos(q_SH); msk.param.a1.*sin(q_SH)];
            hand = elbow + [msk.param.a2.*cos(q_SH+q_EL); msk.param.a2.*sin(q_SH+q_EL)];
        end
        %% SOLVE MSK MODEL
        function [msk,Xdot,X,Y] = solveModel_msk(msk)

            X = msk.X;
            u = msk.U;
            F_hand = msk.Fhand;
            armDamping = msk.mskParamSet.armDamping;

            [Xdot,X,Y] =PlanarArm(0,X,[F_hand(2);F_hand(1);u],armDamping);
            msk.X = X;
            msk.XDOT = Xdot;
            msk.Y = Y;

        end

        %% CALUCALTE TORQUE-EQUIVALENCE OF MUSCLE INPUTS
        % TODO INCOMPLETE
        function tau = calculateMuscleTorques(msk, u, q, qdot, F, param)

            X = getPlanarArmIC(qEL, qdotEL, qSH, qdotSH, u, param);
            [Xdot, ~, ~] = PlanarArm(0,X,[F(2);F(1);u],param);

            qddot = getPlanarArmQddot(Xdot);
            sys = armParameters();


            M = sysEQ_massMatrix([q;qdot], sys);
            C = sysEQ_CMatrix([q;qdot],sys);
            J = sysEQ_J([q;qdot],sys);

            tau = M*qddot + C*qdot - J'*F;
        end

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
                X = getPlanarArmIC(qEL, qdotEL, qSH, qdotSH, u, msk.param.armDamping);
                [~, ~, y] = PlanarArm(0,X,[0;0;u],msk.param.armDamping);
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


    end


end
