function [basis_acc, basis_F] = calcBasis(synergy, qSH, qEL, parameters)

qdotSH = 0; % must be zero to maintain linearity (i,e, double activations gives double F or acc) and should be corrected nonlinearly later
qdotEL = 0;

NoSyn = size(synergy,2);
basis_acc = zeros(2,NoSyn);
basis_F = zeros(2,NoSyn);

for i=1:NoSyn
    u = synergy(:,i);
    IC = getPlanarArmIC(qEL, qdotEL, qSH, qdotSH, u, parameters);
    [~, ~, y] = PlanarArm(0,IC,[0;0;u],parameters);
    basis_acc(:,i) = getPlanarArmOutputs(y).hand_a;
    basis_F(:,i) = calculateTaskSpaceForce(u, [qSH; qEL], [qdotSH;qdotEL], [0;0], parameters);
end


