function [L,K,newCost,Sx,Se,s,JC] = getOptimalGains(ofc)

% The output matrices are:
%   L                   time series of optimal feedback gains
%   K                   time series of non adaptive Kalmen gains
%   NEWCOST             expected cost after stopping the iterations
%   SX, SE              series of cost matrices determined by the backwards
%                       recurrences for the state (SX) and error (SE) terms
%   S                   scalar component of the total expected cost
%   JC                  covariance matrix


A = ofc.systemEq.A;
B = ofc.systemEq.B;
H = ofc.systemEq.H;
Q = ofc.costFunction.Q;
R = ofc.costFunction.R;

n = ofc.systemEq.numberOfStates;
p = ofc.systemEq.numberOfOutputs;
nStep = ofc.simSetting.nStep;
xInit = ofc.simSetting.xInit;

C = ofc.noiseConstructors.controlDependentConstructor;
D = ofc.noiseConstructors.stateDependentConstructor;
Omega_xi = ofc.noiseConstructors.additiveProcessNoiseCovar;
Omega_omega = ofc.noiseConstructors.sensoryNoiseCovar;
Omega_eta = ofc.noiseConstructors.internalNoiseCovar;
Sigma_e = ofc.noiseConstructors.errorEstimateCovar;
Sigma_x = ofc.noiseConstructors.stateEstimateCovar;


h = ofc.simSetting.delay;
xInit = repmat(xInit,h+1,1);

K = zeros(n,p,nStep);

tol = 1e-14;
current = 1e6;
itmax = 500;
dCost = 1e6;
count = 0;



% The optimal control and Kalman gains are calculated iteratively (no more
% than itmax times if it does not converge)
while (dCost>tol) && (count<itmax)
    
    [L,Sx,Se,s] = getGainsController(A,B,C,D,H,Q,R,K,Omega_xi,Omega_omega,Omega_eta);
    [K,JC] = getGainsKalmanFilter(A,B,C,D,H,Omega_xi,Omega_omega,Omega_eta,L,Sigma_x,Sigma_e);

    % Expected cost
    newCost = xInit'*Sx*xInit + trace((Sx+Se)*Sigma_e)+ s;
    
    % Relative improvement
    dCost = abs(current - newCost)/newCost;
    current = newCost;

    count = count + 1;
    
end

fprintf('Number of iterations: %d\n', count); % Number of iterations 

ofc.optimalGains.L = L;
ofc.optimalGains.K = K;