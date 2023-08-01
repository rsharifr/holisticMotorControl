function [hand, elbow] = getHandPosition(q_SH,q_EL,sys)

shoulder = [0;0];

elbow = shoulder + [sys.a1.*cos(q_SH); sys.a1.*sin(q_SH)];
hand = elbow + [sys.a2.*cos(q_SH+q_EL); sys.a2.*sin(q_SH+q_EL)];