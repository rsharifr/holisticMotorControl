function [ee, joint1] = getEEPosition(q1,q2,sys)

q1 = q1+pi; % the robot in maplesim is rotated 180 degrees. (robot's "forward" is -y, robot's "right" is -x)

robotBase = [0;0.8];

joint1 = robotBase + [sys.a1.*cos(q1); sys.a1.*sin(q1)];
ee = joint1 + [sys.a2.*cos(q1+q2); sys.a2.*sin(q1+q2)];