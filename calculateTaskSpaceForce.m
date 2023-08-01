function F_taskSpace = calculateTaskSpaceForce(u, q, qdot, F, param)

IC = getPlanarArmIC(q(2), qdot(2), q(1), qdot(1), u, param);
[Xdot, ~, ~] = PlanarArm(0,IC,[F(2);F(1);u],param, 1e-3, 1);

qddot = getPlanarArmQddot(Xdot);
sys = getParametersPlanarArm();


M = sysEQ_massMatrix([q;qdot], sys);
C = sysEQ_CMatrix([q;qdot],sys);
J = sysEQ_J([q;qdot],sys);

tau = M*qddot + C*qdot - J'*F;

F_taskSpace = (J')\tau;