function [x_out,xest_out,u_out,Sigma_x_out,Sigma_e_out,Sigma_ex_out] = simulateMotion(ofc)

nStep = ofc.simSetting.nStep;
n = ofc.systemEq.numberOfStates;
m = ofc.systemEq.numberOfControls;
p = ofc.systemEq.numberOfOutputs;
xInit = ofc.simSetting.xInit;

h = ofc.simSetting.delay;
A = ofc.systemEq.A;
A_sim = ofc.systemEq.A_sim;
B = ofc.systemEq.B;
H = ofc.systemEq.H;

if size(A,3) == 1
    A = repmat(A,1,1,nStep);
    A_sim = repmat(A_sim,1,1,nStep);
end

C = ofc.noiseConstructors.controlDependentConstructor;
D = ofc.noiseConstructors.stateDependentConstructor;
Omega_xi = ofc.noiseConstructors.additiveProcessNoiseCovar;
Omega_omega = ofc.noiseConstructors.sensoryNoiseCovar;
Omega_eta = ofc.noiseConstructors.internalNoiseCovar;



Q = ofc.costFunction.Q;
R = ofc.costFunction.R;
K = ofc.optimalGains.K;
L = ofc.optimalGains.L;

for iter = 1:ofc.simSetting.nsimu
    cost = 0;
    currentX = repmat(xInit,h+1,1);
    currentXEst = currentX;
    x = zeros(nStep,n);
    x(1,:) = currentX(1:n)';
    xest = x;
    u = zeros(nStep-1,m);
    Sigma_ex = zeros(n,n,nStep);
    Sigma_e = repmat(ofc.noiseConstructors.errorEstimateCovar,1,1,nStep);
    Sigma_x = repmat(ofc.noiseConstructors.stateEstimateCovar,1,1,nStep);

    for i = 1:nStep-1
                
        sensoryNoise = mvnrnd(zeros(p,1),Omega_omega)';
        processNoise = mvnrnd(zeros(n,1),Omega_xi)';
%         processNoise(7:8) = zeros(2,1); % FOR 2D REACH MODEL. The perturbation is deterministic. Does not matter much
        internalNoise = mvnrnd(zeros(n,1),Omega_eta)';
        
        stateDependentNoise = 0;
        for isdn = 1:size(D,3)
            stateDependentNoise = stateDependentNoise + randn*D(:,:,isdn)*x(i,:)';
        end
        
        yz = H*currentX + sensoryNoise + stateDependentNoise;
        u(i,:) = (-L(:,:,i)*currentXEst)';
        
        currentXEst = A(:,:,i)*currentXEst + B*u(i,:)' + K(:,:,i)*(yz-H*currentXEst) + internalNoise;
        
        % Cost LQG
        cost = cost + currentX'*Q(:,:,i)*currentX + u(i,:)*R(:,:,i)*u(i,:)';
        
        %control-dependent noise
        controlDependentNoise = 0;
        for icdn = 1:m
            controlDependentNoise = controlDependentNoise + randn*C(:,:,icdn)*u(i,:)';
        end

        currentX = (A_sim(:,:,i))*currentX + B*u(i,:)' + processNoise + controlDependentNoise;
        

        
        x(i+1,:) = currentX(1:n)';
        xest(i+1,:) = currentXEst(1:n)';
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        sTemp = (Sigma_e(:,:,i) + Sigma_x(:,:,i) + Sigma_ex(:,:,i) + Sigma_ex(:,:,i)');
        
        sdn = 0; % state-dependent term
        for j = 1:size(D,3)
            sdn = sdn + D(:,:,j)*sTemp*D(:,:,j)';
        end
        
        cdn = 0; % control-dependent term
        for j = 1:size(C,3)
            cdn = cdn + C(:,:,j)*L(:,:,i)*Sigma_x(:,:,i)*L(:,:,i)'*C(:,:,j)';
        end
        
        Sigma_e(:,:,i+1) = Omega_xi+Omega_eta+(A(:,:,i)-K(:,:,i)*H)*Sigma_e(:,:,i)*A(:,:,i)'+cdn;
        term = (A(:,:,i)-B*L(:,:,i))*Sigma_ex(:,:,i)*H'*K(:,:,i)';
        Sigma_x(:,:,i+1) = Omega_eta + K(:,:,i)*H*Sigma_e(:,:,i)*A(:,:,i)'+(A(:,:,i)-B*L(:,:,i))*Sigma_x(:,:,i)*(A(:,:,i)-B*L(:,:,i))'...
            + term + term';
        Sigma_ex(:,:,i+1) = (A(:,:,i)-B*L(:,:,i))*Sigma_ex(:,:,i)*(A(:,:,i)-K(:,:,i)*H)'-Omega_eta;
        
        
    end
    
    x = x(:,1:ofc.systemEq.numberOfOriginalStates);
    xest = xest(:,1:ofc.systemEq.numberOfOriginalStates);
    
    x_out{iter} = x;
    xest_out{iter} = xest;
    u_out{iter} = u;
    Sigma_e_out{iter} = Sigma_e;
    Sigma_x_out{iter} = Sigma_x;
    Sigma_ex_out{iter} = Sigma_e;
    cost_out{iter} = cost;
    
end
ofc.simulationResults.x = x_out;
ofc.simulationResults.xest = xest_out;
ofc.simulationResults.u = u_out;