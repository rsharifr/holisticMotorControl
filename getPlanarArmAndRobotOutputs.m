function yout = getPlanarArmAndRobotOutputs(y)

% x is forward, z is to the right.

if iscolumn(y)
    yout.q = y([4,2]);
    yout.qdot = y([5,3]);
    yout.hand_a = y([8,6]);
    yout.hand_v = y([11,9]);
    yout.hand_p = y([14,12]);
    yout.muscleF = y(15:20);
    yout.ee_a = y([23,21]);
    yout.ee_v = y([29,27]);
    yout.ee_p = y([26,24]);
    yout.q_robot = y([30,32]);
    yout.qdot_robot = y([31,33]);
    
else
    yout.q = y(:,[4,2]);
    yout.qdot = y(:,[5,3]);
    yout.hand_a = y(:,[8,6]);
    yout.hand_v = y(:,[11,9]);
    yout.hand_p = y(:,[14,12]);
    yout.muscleF = y(:,15:20);
    yout.ee_a = y(:,[23,21]);
    yout.ee_v = y(:,[29,27]);
    yout.ee_p = y(:,[26,24]);
    yout.q_robot = y(:,[30,32]);
    yout.qdot_robot = y(:,[31,33]);    
end


% * Output variable(s):
% *    y[ 1] = `t`
% *    y[ 2] = `Main.PlanarArmAndRobot.MuscleArm1.zElbow.theta`(t)
% *    y[ 3] = diff(`Main.PlanarArmAndRobot.MuscleArm1.zElbow.theta`(t),t)
% *    y[ 4] = `Main.PlanarArmAndRobot.MuscleArm1.zShoulder.theta`(t)
% *    y[ 5] = diff(`Main.PlanarArmAndRobot.MuscleArm1.zShoulder.theta`(t),t)
% *    y[ 6] = `Main.PlanarArmAndRobot.axyz[1]`(t)
% *    y[ 7] = `Main.PlanarArmAndRobot.axyz[2]`(t)
% *    y[ 8] = `Main.PlanarArmAndRobot.axyz[3]`(t)
% *    y[ 9] = `Main.PlanarArmAndRobot.vxyz[1]`(t)
% *    y[10] = `Main.PlanarArmAndRobot.vxyz[2]`(t)
% *    y[11] = `Main.PlanarArmAndRobot.vxyz[3]`(t)
% *    y[12] = `Main.PlanarArmAndRobot.xyz[1]`(t)
% *    y[13] = `Main.PlanarArmAndRobot.xyz[2]`(t)
% *    y[14] = `Main.PlanarArmAndRobot.xyz[3]`(t)
% *    y[15] = `Main.PlanarArmAndRobot.FBiExt`(t)
% *    y[16] = `Main.PlanarArmAndRobot.FBiFlex`(t)
% *    y[17] = `Main.PlanarArmAndRobot.FElExt`(t)
% *    y[18] = `Main.PlanarArmAndRobot.FElFlex`(t)
% *    y[19] = `Main.PlanarArmAndRobot.FShExt`(t)
% *    y[20] = `Main.PlanarArmAndRobot.FShFlex`(t)
% *    y[21] = `Main.PlanarArmAndRobot.eeAcc[1]`(t)
% *    y[22] = `Main.PlanarArmAndRobot.eeAcc[2]`(t)
% *    y[23] = `Main.PlanarArmAndRobot.eeAcc[3]`(t)
% *    y[24] = `Main.PlanarArmAndRobot.eePos[1]`(t)
% *    y[25] = `Main.PlanarArmAndRobot.eePos[2]`(t)
% *    y[26] = `Main.PlanarArmAndRobot.eePos[3]`(t)
% *    y[27] = `Main.PlanarArmAndRobot.eeVel[1]`(t)
% *    y[28] = `Main.PlanarArmAndRobot.eeVel[2]`(t)
% *    y[29] = `Main.PlanarArmAndRobot.eeVel[3]`(t)
% *    y[30] = `Main.PlanarArmAndRobot.robot.R1.theta`(t)
% *    y[31] = diff(`Main.PlanarArmAndRobot.robot.R1.theta`(t),t)
% *    y[32] = `Main.PlanarArmAndRobot.robot.R2.flange_b_phiMB`(t)
% *    y[33] = diff(`Main.PlanarArmAndRobot.robot.R2.flange_b_phiMB`(t),t)