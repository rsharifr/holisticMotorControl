function [C,D,Omega_xi,Omega_omega,Omega_eta,Sigma_e,Sigma_x] = setupNoiseConstructors(ofc,noiseStructure)
c = noiseStructure.c;
d = noiseStructure.d;
xi = noiseStructure.xi;
omega = noiseStructure.omega;
eta = noiseStructure.eta;

B = ofc.systemEq.B;
n = ofc.systemEq.numberOfStates;
m = ofc.systemEq.numberOfControls;
p = ofc.systemEq.numberOfOutputs;
C = zeros(n,m,m);% control-dependent noise constructor
for i = 1:m
    Cprime = zeros(m);
    Cprime(i,i) = c;
    C(:,:,i) = B*Cprime;
end
D = d*zeros(p,n,1); % state-dependent noise constructor

Omega_xi = zeros(size(B*B'));
Omega_xi((B*B')~=0) = xi; % covariance of additive process noise (only for control-affected states)
% Omega_xi(6,6) = xi; % FOR 1D cup task. this is important to allow the observer to expect disturbances
% Omega_xi(7:8,7:8) = Omega_xi(5:6,5:6); % FOR 2D REACH MODEL. this is important to allow the observer to expect disturbances
Omega_omega = omega*eye(p); % covariance of sensory noise
Omega_eta = eta*eye(n); % covariance of internal noise

Sigma_e = eye(n)*10^-4; % initial unconditional error estimation covariance
Sigma_x = eye(n)*10^-2; % initial unconditional state estimation covariance


ofc.noiseConstructors.controlDependentConstructor = C;
ofc.noiseConstructors.stateDependentConstructor = D;
ofc.noiseConstructors.additiveProcessNoiseCovar = Omega_xi;
ofc.noiseConstructors.sensoryNoiseCovar = Omega_omega;
ofc.noiseConstructors.internalNoiseCovar = Omega_eta;
ofc.noiseConstructors.errorEstimateCovar = Sigma_e;
ofc.noiseConstructors.stateEstimateCovar = Sigma_x;

end