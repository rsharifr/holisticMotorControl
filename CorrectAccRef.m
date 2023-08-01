function a_tilde = CorrectAccRef(a_ref,q_SH,q_EL,qdot_SH,qdot_EL,sys,Fhand)
% here Fhand is the external force onto the end

q = [q_SH;q_EL];
qdot = [qdot_SH;qdot_EL];
X = [q;qdot];

M = sysEQ_massMatrix(X,sys);
J = sysEQ_J(X,sys);
C = sysEQ_CMatrix(X,sys);
Jdot = sysEQ_Jdot(X,sys);

% a_tilde = a_ref - Jdot*qdot + J*(M\C)*qdot; % this was the original which seemed to work 

qddot_ref = J\(a_ref - Jdot*qdot);

qddot_tilde = qddot_ref + M\(-J'*Fhand + C*qdot); 

a_tilde = 0*Jdot*qdot + J*qddot_tilde; % here Jdot*qdot isn't needed becuase the "nominal" condition do not have velocities
