function tau = calculateMuscleTorques(u, q, qdot, F, param)

IC = getPlanarArmIC(qEL, qdotEL, qSH, qdotSH, u, param);
[Xdot, ~, ~] = PlanarArm(0,IC,[F(2);F(1);u],param);

qddot = getPlanarArmQddot(Xdot);
sys = armParameters();


M = sysEQ_massMatrix([q;qdot], sys);
C = sysEQ_CMatrix([q;qdot],sys);
J = sysEQ_J([q;qdot],sys);

tau = M*qddot + C*qdot - J'*F;