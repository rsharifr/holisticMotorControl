function obj = objFun(U, thetaSH, thetaEL, Fhand, refAcc, param)

IC = getPlanarArmIC(thetaEL, 0, thetaSH, 0, U, param);

[~, ~, y] = PlanarArm(0,IC,[Fhand(2); Fhand(1); U],param);

% posX = y(12);
% posZ = y(14);
% 
% VelX = y(9);
% VelZ = y(11);

acc = getPlanarArmOutputs(y).hand_a;

% AccX = y(6);
% AccZ = y(8);

% refX = RefPos(1);
% refZ = RefPos(3);

% refAccX = RefAcc(2);
% refAccZ = RefAcc(1);


Wa = 1;
% err = Wa*( (AccX-refAccX)^2+ (AccZ-refAccZ)^2 );
err = Wa * norm(acc-refAcc)^2;

effort = sum(U.^2);

Werr = 1;
Weff = 1;

obj = Werr*err + Weff*effort;