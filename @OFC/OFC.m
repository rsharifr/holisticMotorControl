classdef OFC < handle
    properties
        simSetting
        costFunction
        systemEq
        optimalGains
        simulationResults
        noiseConstructors
    end
    properties(GetAccess = private)
        
    end
      
    methods
        function ofc = OFC(A,B,H,Q,R,simSetting,noiseStructure)
            ofc.simSetting = simSetting;
            deltaT = simSetting.deltaT;
            if isempty(ofc.simSetting.A_sim)
                A_sim = A;
            else
                A_sim = ofc.simSetting.A_sim;
            end
            
            if any(size(A)~=size(A_sim))
                error('Dimension mismatch between "A" and "A_sim"')
            end
            A_discrete = eye(size(A,2))+deltaT*A;
            A_sim_discrete = eye(size(A_sim,2)) + deltaT*A_sim;
            B_discrete = deltaT*B;

%             l = 10;
%             l_learned = 10*0;
%             A_discrete(7,3,:) = 0;
%             A_discrete(7,4,:) = l_learned;
%             A_discrete(8,3,:) = -l_learned;
%             A_discrete(8,4,:) = 0;
%             A_discrete(7,7,:) = 0;
%             A_discrete(8,8,:) = 0;
%             A_sim_discrete(7,3,:) = 0;
%             A_sim_discrete(7,4,:) = l;
%             A_sim_discrete(8,3,:) = -l;
%             A_sim_discrete(8,4,:) = 0;
%             A_sim_discrete(7,7,:) = 0;
%             A_sim_discrete(8,8,:) = 0;

            
            ofc.systemEq.A = A_discrete;
            ofc.systemEq.A_sim = A_sim_discrete;
            ofc.systemEq.B = B_discrete;
            ofc.systemEq.H = H;
            ofc.systemEq.numberOfStates = size(A_discrete,2);
            ofc.systemEq.numberOfControls = size(B_discrete,2);
            ofc.systemEq.numberOfOutputs = size(H,1);
            ofc.costFunction.Q = Q;
            ofc.costFunction.R = R;
            
            ofc.augmentSystemWithSensoryDelay();
            ofc.setupNoiseConstructors(noiseStructure);
        end
        
        [A,B,Q,H] = augmentSystemWithSensoryDelay(ofc)
        [L,K,newCost,Sx,Se,s,JC] = getOptimalGains(ofc)
        [x,xest,u,Sigma_x,Sigma_e,Sigma_ex] = simulateMotion(ofc)
        [C,D,Omega_xi,Omega_omega,Omega_eta,Sigma_e,Sigma_x] = setupNoiseConstructors(ofc,noiseStructure)
%         plotCupTaskData(ofc,ha,perturbationType,baseDataTimeShift)
%         plotCupTaskSimulations(ofc,ha,modelParam)
        plotRigidData(ofc,ha,velocityProfileShift)
    end
end