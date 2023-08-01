function [IC, y] = getPlanarArmIC(qEL, qdotEL, qSH, qdotSH, a_IC, parameters)
% * State variable(s):
% *    x[ 0] = `Main.PlanarArm.MuscleArm1.BiExtensor.activationDynamics1.a`(t)
% *    x[ 1] = `Main.PlanarArm.MuscleArm1.BiFlexor.activationDynamics1.a`(t)
% *    x[ 2] = `Main.PlanarArm.MuscleArm1.ElbowExtensor.activationDynamics1.a`(t)
% *    x[ 3] = `Main.PlanarArm.MuscleArm1.ElbowFlexor.activationDynamics1.a`(t)
% *    x[ 4] = `Main.PlanarArm.MuscleArm1.ShoulderExtensor.activationDynamics1.a`(t)
% *    x[ 5] = `Main.PlanarArm.MuscleArm1.ShoulderFlexor.activationDynamics1.a`(t)
% *    x[ 6] = `Main.PlanarArm.MuscleArm1.zElbow.theta`(t)
% *    x[ 7] = diff(`Main.PlanarArm.MuscleArm1.zElbow.theta`(t),t)
% *    x[ 8] = `Main.PlanarArm.MuscleArm1.zShoulder.theta`(t)
% *    x[ 9] = diff(`Main.PlanarArm.MuscleArm1.zShoulder.theta`(t),t)

IC = zeros(10,1);
IC(1:6) = a_IC;
IC(7) =  qEL; % elbow angle
IC(8) = qdotEL;
IC(9) =  qSH; % shoulder angle
IC(10) = qdotSH;
[~, IC, y] = PlanarArm(0,IC,zeros(8,1),parameters);