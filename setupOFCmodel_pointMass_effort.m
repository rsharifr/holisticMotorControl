function [A,B,H,Q,R,simSetting,noiseStructure,xInit,modelParam] = setupOFCmodel_pointMass_effort(startPosition, deltaT, tEnd, numberOfStationarySteps,f_pert)
%% %%%%%%%%% continuous system dynamics %%%%%%%%%%%
nStep = round(tEnd/deltaT) + numberOfStationarySteps;

kd = 0; % environment damping.
mh = 3;
mo = 0;
m = mh+mo;
mx = m;
my = m;
tau = 0.03;%.066;

f_max = 1000; % maximum force; F=f_max*a

n=8; % number of states
A = [0 0 1 0 0 0 0 0; % d/dt(x)
    0 0 0 1 0 0 0 0; % d/dt(y)
    0 0 -kd/mx 0 1/mx*f_max 0 0 0; % d/dt(xdot)
    0 0 0 -kd/my 0 1/my*f_max 0 0; % d/dt(ydot)
    0 0 0 0 -1/tau 0 0 0; % d/dt(act_x)
    0 0 0 0 0 -1/tau 0 0; % d/dt(act_y)
    0 0 0 0 0 0 0 0; % d/dt(Fx_dist)
    0 0 0 0 0 0 0 0]; % d/dt(Fy_dist)
A = repmat(A,1,1,nStep-1);
A_sim = A;
B = [0 0;
    0 0;
    0 0;
    0 0;
    1/tau 0;
    0 1/tau;
    0 0;
    0 0];
H = eye(size(A,2));


% this is impulse/step perturbation 
A(3,7,1:end) = 1/mx;
A(4,8,1:end) = 1/my;
A_sim(3,7,1:end) = 1/mx;
A_sim(4,8,1:end) = 1/my;

% this is the force field
% l = 0; % force field term
% l_learned = 1*l;
% A(3,3,:) = A(3,3,:) + 0*l_learned/m;
% A(3,4,:) = A(3,4,:) + -l_learned/m;
% A(4,3,:) = A(4,3,:) + l_learned/m;
% A(4,4,:) = A(4,4,:) + 0*l_learned/m;
% A_sim(3,3,:) = A_sim(3,3,:) + 0*l/m;
% A_sim(3,4,:) = A_sim(3,4,:) + -l/m;
% A_sim(4,3,:) = A_sim(4,3,:) + l/m;
% A_sim(4,4,:) = A_sim(4,4,:) + 0*l/m;



%% %%%%%%%% Setup up noise levels %%%%%%%%%%%%%
sensoryDelay = 0.05; % in seconds
c = 0.2; % multiplicative noise factor for control-dependent noise
d = 0; % multiplicative noise factor for state-dependent noise 
xi = 2e-7; % process noise factor
omega = 1e-6; % sensory noise factor
eta = 1e-8; % internal noise factor


%% %%%%%%%%% Initial condition and penalties %%%%%%%%%%%
statePenaltyCoefficients = [1e5 , 1e5 , 1e4, 1e4,  0,  0,  0,  0];
statePenaltyCoefficients = statePenaltyCoefficients(1:n);
Q = zeros(n,n,nStep);
Q(:,:,1+end-numberOfStationarySteps:end) = repmat(diag(statePenaltyCoefficients),1,1,numberOfStationarySteps);

controlPenaltyCoefficients = 100*[1,1];
R = repmat(diag(controlPenaltyCoefficients),1,1,nStep-1);
xInit = [startPosition; 0; 0; -0*f_pert; f_pert]; 

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

modelParam.mh = mh;
modelParam.mo = mo;
modelParam.tau = tau;
modelParam.kd = kd;
