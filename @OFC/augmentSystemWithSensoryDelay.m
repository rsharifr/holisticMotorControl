function [A,B,Q,H] = augmentSystemWithSensoryDelay(ofc)

A0 = ofc.systemEq.A;
A0_sim = ofc.systemEq.A_sim;
B0 = ofc.systemEq.B;
H0 = ofc.systemEq.H;
if ~isempty(ofc.costFunction)
    Q0 = ofc.costFunction.Q;
else
    error('Q matrix is empty')
end
h = ofc.simSetting.delay;

n = size(A0,1);
nSteps_A = size(A0,3);
m = size(B0,2);
nStep = size(Q0,3);
p = size(H0,1);

A = zeros((h+1)*n,(h+1)*n,nSteps_A);
A_sim = zeros((h+1)*n,(h+1)*n,nSteps_A);
B = zeros((h+1)*n,m);
Q = zeros((h+1)*n,(h+1)*n,nStep);
H = zeros(p,(h+1)*n);

for i = 1:nSteps_A
    A(1:n,1:n,i) = A0(:,:,i);
    A(n+1:end,1:end-n,i) = eye(h*n);
    
    A_sim(1:n,1:n,i) = A0_sim(:,:,i);
    A_sim(n+1:end,1:end-n,i) = eye(h*n);
end
B(1:n,:) = B0;
H(:,end-n+1:end) = H0;

% Adding h times the constraint Q1:
Qaug = zeros(n,n,nStep+h);
for i = 1:h
    Qaug(:,:,i) = Q0(:,:,1);
end
for t = 1:nStep
    Qaug(:,:,t+h) = Q0(:,:,t);
end

%Filling the diagonal Q matrices
for t = 1:nStep
    for i = 0:h
        Q(i*n+1:(i+1)*n,i*n+1:(i+1)*n,t) = Qaug(:,:,t+h-i)/(h+1);
    end
end

ofc.systemEq.A = A;
ofc.systemEq.A_sim = A_sim;
ofc.systemEq.B = B;
ofc.systemEq.H = H;
ofc.costFunction.Q = Q;
ofc.systemEq.numberOfStates = size(A,2);
ofc.systemEq.numberOfControls = size(B,2);
ofc.systemEq.numberOfOriginalStates = size(A0,2);

