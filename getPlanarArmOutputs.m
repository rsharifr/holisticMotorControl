function yout = getPlanarArmOutputs(y)
% * Output variable(s):
% *    y[ 1] = `time`
% *    y[ 2] = `Main.PlanarArm.MuscleArm1.zElbow.theta`(t)
% *    y[ 3] = diff(`Main.PlanarArm.MuscleArm1.zElbow.theta`(t),t)
% *    y[ 4] = `Main.PlanarArm.MuscleArm1.zShoulder.theta`(t)
% *    y[ 5] = diff(`Main.PlanarArm.MuscleArm1.zShoulder.theta`(t),t)
% *    y[ 6] = `Main.PlanarArm.axyz[1]`(t)
% *    y[ 7] = `Main.PlanarArm.axyz[2]`(t)
% *    y[ 8] = `Main.PlanarArm.axyz[3]`(t)
% *    y[ 9] = `Main.PlanarArm.vxyz[1]`(t)
% *    y[10] = `Main.PlanarArm.vxyz[2]`(t)
% *    y[11] = `Main.PlanarArm.vxyz[3]`(t)
% *    y[12] = `Main.PlanarArm.xyz[1]`(t)
% *    y[13] = `Main.PlanarArm.xyz[2]`(t)
% *    y[14] = `Main.PlanarArm.xyz[3]`(t)
% *    y[15] = `Main.PlanarArm.FBiExt`(t)
% *    y[16] = `Main.PlanarArm.FBiFlex`(t)
% *    y[17] = `Main.PlanarArm.FElExt`(t)
% *    y[18] = `Main.PlanarArm.FElFlex`(t)
% *    y[19] = `Main.PlanarArm.FShExt`(t)
% *    y[20] = `Main.PlanarArm.FShFlex`(t)

% x is forward, z is to the right.

if iscolumn(y)
    yout.q = y([4,2]);
    yout.qdot = y([5,3]);
    yout.hand_a = y([8,6]);
    yout.hand_v = y([11,9]);
    yout.hand_p = y([14,12]);
    yout.muscleF = y(15:20);
else
    yout.q = y(:,[4,2]);
    yout.qdot = y(:,[5,3]);
    yout.hand_a = y(:,[8,6]);
    yout.hand_v = y(:,[11,9]);
    yout.hand_p = y(:,[14,12]);
    yout.muscleF = y(:,15:20);
end