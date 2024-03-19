function [IC, y,xdot] = getPlanarArmAndRobotIC(qEL, qdotEL, qSH, qdotSH, a_IC, parameters, robotStates)
% * State variable(s):
% *    x[ 0] = `Main.PlanarArmAndRobot.MuscleArm1.BiExtensor.activationDynamics1.a`(t)
% *    x[ 1] = `Main.PlanarArmAndRobot.MuscleArm1.BiFlexor.activationDynamics1.a`(t)
% *    x[ 2] = `Main.PlanarArmAndRobot.MuscleArm1.ElbowExtensor.activationDynamics1.a`(t)
% *    x[ 3] = `Main.PlanarArmAndRobot.MuscleArm1.ElbowFlexor.activationDynamics1.a`(t)
% *    x[ 4] = `Main.PlanarArmAndRobot.MuscleArm1.ShoulderExtensor.activationDynamics1.a`(t)
% *    x[ 5] = `Main.PlanarArmAndRobot.MuscleArm1.ShoulderFlexor.activationDynamics1.a`(t)
% *    x[ 6] = `Main.PlanarArmAndRobot.MuscleArm1.zElbow.theta`(t)
% *    x[ 7] = diff(`Main.PlanarArmAndRobot.MuscleArm1.zElbow.theta`(t),t)
% *    x[ 8] = `Main.PlanarArmAndRobot.MuscleArm1.zShoulder.theta`(t)
% *    x[ 9] = diff(`Main.PlanarArmAndRobot.MuscleArm1.zShoulder.theta`(t),t)
% *    x[10] = `Main.PlanarArmAndRobot.R1.theta`(t)
% *    x[11] = diff(`Main.PlanarArmAndRobot.R1.theta`(t),t)
% *    x[12] = `Main.PlanarArmAndRobot.robot.R1.theta`(t)
% *    x[13] = diff(`Main.PlanarArmAndRobot.robot.R1.theta`(t),t)
% *    x[14] = `Main.PlanarArmAndRobot.robot.R2.flange_b_phiMB`(t)
% *    x[15] = diff(`Main.PlanarArmAndRobot.robot.R2.flange_b_phiMB`(t),t)

IC_guess = zeros(16,1);
IC_guess(1:6) = a_IC;
IC_guess(7) =  qEL; % elbow angle
IC_guess(8) = qdotEL;
IC_guess(9) =  qSH; % shoulder angle
IC_guess(10) = qdotSH;
if exist('robotStates','var')
    IC_guess(11) = robotStates.handRobotAngle;
    IC_guess(12) = robotStates.handRobotAngle_diff;
    IC_guess(13) = robotStates.q1;
    IC_guess(14) = robotStates.qdot1;
    IC_guess(15) = robotStates.q2;
    IC_guess(16) = robotStates.qdot2;
end
[xdot, IC, y] = PlanarArmAndRobot(0,IC_guess,zeros(8,1),parameters);
end