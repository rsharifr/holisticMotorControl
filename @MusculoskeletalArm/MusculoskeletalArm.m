classdef MusculoskeletalArm
    properties
        param
        X
        Y
        XDOT
        U
        Fhand
    end

    methods
        function obj = MusculoskeletalArm(thetaEL_0, omegaEL_0, thetaSH_0, omegaSH_0, a_0, armDamping)
            obj.param = getParametersPlanarArm();
            obj.param.armDamping = armDamping;
            obj.param.Nmuscle = 6;

            [IC,Y0,XDOT0] = obj.getIC(thetaEL_0, omegaEL_0, thetaSH_0, omegaSH_0, a_0);
            obj.X = IC;
            obj.Y = Y0;
            obj.XDOT = XDOT0;
            obj.U = zeros(obj.param.Nmuscle,1);
            obj.Fhand = zeros(2,1);
        end

        %% SOLVE INITIAL CONDITION
        function [IC,Y,XDOT] = getIC(obj, thetaEL_0, omegaEL_0, thetaSH_0, omegaSH_0, a_0)
            [IC,Y,XDOT] = getPlanarArmIC(thetaEL_0, omegaEL_0, thetaSH_0, omegaSH_0, a_0, obj.param.armDamping);
        end

        %% PARSE OUTPUT VECTOR
        function outputs = getOutputs(obj,Y)
            if exist('Y','var')
                outputs = getPlanarArmOutputs(Y);
            else
                outputs = getPlanarArmOutputs(obj.Y);
            end
        end

        %% PARSE STATE VECTOR
        function outputs = getStates(obj,X)
            if exist('X','var')
                outputs = getPlanarArmStates(X);
            else
                outputs = getPlanarArmStates(obj.X);
            end
        end
        %% GET Q_DDOT
        function outputs = getqddot(obj,XDOT)
            if exist('XDOT','var')
                outputs = getPlanarArmQddot(XDOT);
            else
                outputs = getPlanarArmQddot(obj.XDOT);
            end
        end

        %% CALCULATE HAND POSITION
        function [hand, elbow] = getHandPosition(obj, q_SH,q_EL)
            shoulder = [0;0];
            elbow = shoulder + [obj.param.a1.*cos(q_SH); obj.param.a1.*sin(q_SH)];
            hand = elbow + [obj.param.a2.*cos(q_SH+q_EL); obj.param.a2.*sin(q_SH+q_EL)];
        end
        %% SOLVE MSK MODEL
        function [obj,Xdot,X,Y] = solveModel_msk(obj)

            X = obj.X;
            u = obj.U;
            F_hand = obj.Fhand;
            armDamping = obj.mskParamSet.armDamping;

            [Xdot,X,Y] =PlanarArm(0,X,[F_hand(2);F_hand(1);u],armDamping);
            obj.X = X;
            obj.XDOT = Xdot;
            obj.Y = Y;

        end

        %% CALUCALTE TORQUE-EQUIVALENCE OF MUSCLE INPUTS
        % TODO INCOMPLETE
        function tau = calculateMuscleTorques(obj, u, q, qdot, F, param)

            IC = getPlanarArmIC(qEL, qdotEL, qSH, qdotSH, u, param);
            [Xdot, ~, ~] = PlanarArm(0,IC,[F(2);F(1);u],param);

            qddot = getPlanarArmQddot(Xdot);
            sys = armParameters();


            M = sysEQ_massMatrix([q;qdot], sys);
            C = sysEQ_CMatrix([q;qdot],sys);
            J = sysEQ_J([q;qdot],sys);

            tau = M*qddot + C*qdot - J'*F;
        end

    end


end
