function [A,B,H,Q,R,simSetting,noiseStructure,xInit,modelParam] = setupInternalModel_cupTask_effort(startPosition, deltaT, tEnd, numberOfStationarySteps,f_pert)
nStep = round(tEnd/deltaT) + numberOfStationarySteps;

%% %%%%%%%%% continuous system dynamics %%%%%%%%%%%
mh = 3; % hand (arm) mass
mc = 0; % cart mass
mp = 0.5; % pendulum mass
l = 0.5; % pendulum length
g = 9.81;
tau = 0.03; % muscle time constant
f_max = 1000; % maximum force; F=f_max*a
G = 1; % agility factor: G=0: pendulum is "fixed". G=1, normal pendulum. G>1 (e.g. 5) hyperactive pendulum (the system is energetically active, not passive)

alpha = (mc+mh) + mp - mp*G;

n = 12; % order of states: [1:x 2:y 3:vx 4:vy 5:actx 6:acty 7:fpertx 8:fperty 9:theta1 10:theta2 11:omega1 12:omega2]
m = 2; % number of control inputs

A = zeros(n);
A(1,3) = 1; % xdot = 1 * vx 
A(2,4) = 1; % ydot = 1 * vy

A(3,9) = mp*g/alpha; % vxdot = mpg/alpha * theta1 +...
A(3,5) = 1/alpha*f_max; % vxdot = ...+ 1/alpha*fmax * actx + ...
A(3,7) = 1/alpha; % vxdot = ...+ 1/alpha * f_pertx

A(4,10) = mp*g/alpha; % vydot = mpg/alpha * theta2 +...
A(4,6) = 1/alpha*f_max; % vydot = ...+ 1/alpha*fmax * acty + ...
A(4,8) = 1/alpha; % vydot = ...+ 1/alpha * f_perty

A(5,5) = -1/tau; % actxdot = -1/tau * actx + ...
A(6,6) = -1/tau; % actxdot = -1/tau * acty + ...

A(9,11) = 1; % thetadot1 = 1 * omega1
A(10,12) = 1; % thetadot2 = 1 * omega2

A(11,9) = -g/l*(1+G*mp/alpha); % omega1dot = -g/l*(1+G*mp/alpha) * theta1 +...
A(11,5) = -G/l/alpha*f_max; % omega1dot = ...+ -G/l/alpha * fmax*actx

A(12,10) = -g/l*(1+G*mp/alpha); % omega2dot = -g/l*(1+G*mp/alpha) * theta2 +...
A(12,6) = -G/l/alpha*f_max; % omega2dot = ...+ -G/l/alpha * fmax*acty



B = zeros(n,m);
B(5,1) = 1/tau; % actxdot = ...+1/tau ux.
B(6,2) = 1/tau; % actydot = ...+1/tau uy.

A = repmat(A,1,1,nStep-1);
A_sim = A;

H = eye(n);

%% %%%%%%%%% Setup up noise/delay levels %%%%%%%%%%%%%
sensoryDelay = 1*0.05; % in seconds
c = 0.2; % multiplicative noise factor for control-dependent noise
d = 0; % multiplicative noise factor for state-dependent noise
omega = 1e-6; % sensory noise factor 
eta = 1e-9; % internal noise factor
xi = 1e-8; % process noise (affects only control-affected states)

%% %%%%%%%%% Initial condition and penalties %%%%%%%%%%%
% order of states: % order of states: [1:x 2:y 3:vx 4:vy 5:actx 6:acty 7:fpertx 8:fperty 9:theta1 10:theta2 11:omega1 12:omega2]
v0 = [0;0]; % initial velocities
act0 = [0;0]; % initial activation values
theta0 = [0;0]; % initial condition for the pendulum
omega0 = [0;0]; % initial condition for the pendulum
xInit = [startPosition; v0; act0; f_pert; theta0; omega0];

controlPenaltyCoefficients = 1*ones(m,1); 
R = repmat(diag(controlPenaltyCoefficients),1,1,nStep-1);

FinalStatePenaltyCoefficients = [1,1,  1,1,  0,0,  0,0,  0.05,0.05, 0.01,0.01]; % final state penalties (only during the last numberOfStationarySteps)
integralStatePenaltyCoefficients = zeros(1,n); % state penalties from t=0 until nstep-numberOfStationarySteps
Q = zeros(n,n,nStep);
Q(:,:,1:end-numberOfStationarySteps) = repmat(diag(integralStatePenaltyCoefficients(1:n)),1,1,nStep-numberOfStationarySteps);
Q(:,:,1+end-numberOfStationarySteps:end) = repmat(diag(FinalStatePenaltyCoefficients(1:n)),1,1,numberOfStationarySteps);

%%

noiseStructure.c = c;
noiseStructure.d = d;
noiseStructure.xi = xi;
noiseStructure.omega = omega;
noiseStructure.eta = eta;

simSetting.tEnd = tEnd;
simSetting.nStep = nStep;
simSetting.deltaT = deltaT;
simSetting.delay = round(sensoryDelay/deltaT);
simSetting.pert = f_pert;
simSetting.nsimu = 10;
simSetting.A_sim = A_sim;
simSetting.pertStart = [];
simSetting.xInit = xInit;
simSetting.numberOfStationarySteps = numberOfStationarySteps;

modelParam.mh = mh;
modelParam.mp = mp;
modelParam.mc = mc;
modelParam.l = l;
modelParam.g = g;
modelParam.G = G;
modelParam.tau = tau;
modelParam.kp = [];
modelParam.kd = [];
modelParam.f_max = f_max;