function uOpt = SolveOptimalActivation(thetaSH, thetaEL, Fhand, refAcc)

param =  [0, 0]; % [Elbow Damping; Shoulde damping]

% Lua = 0.31;
% Lfa = 0.34;
% SHjoint = [0,0,0];
% ELjoint = SHjoint + [Lua.*sin(thetaSH), 0, Lua.*cos(thetaSH)];
% RefPos = ELjoint + [Lfa.*sin(thetaSH+thetaEL), 0, Lfa.*cos(thetaSH+thetaEL)];



A = [];
b = [];
Aeq = [];
beq = [];
lb = zeros(6,1);
ub = ones(6,1);
nonlcon = [];
options = optimset('display','none',...
    'algorithm','sqp',...
    'TolFun',1e-6,...
    'TolX',1e-6);

X0 =  0.1*ones(6,1);
uOpt = fmincon(@(U)objFun(U, thetaSH, thetaEL, Fhand, refAcc, param), X0, A, b, Aeq, beq, lb, ub, nonlcon, options);


