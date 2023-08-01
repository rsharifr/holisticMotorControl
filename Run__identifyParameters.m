thetaELset = 1:0.05:1.3;
thetaSHset = 0.6:0.05:1.2;

param = [1;1];

A = [];
F = [];

for thetaSH = thetaSHset
    for thetaEL = thetaELset
        IC = getPlanarArmIC(thetaEL, 0, thetaSH, 0, zeros(6,1), param);
        for k = 1:500
            Fpert = (rand([2,1])*2-1)*10;
            [~, ~, Y] = PlanarArm(0,IC,[Fpert(2);Fpert(1);zeros(6,1)],param);
            A = [A,getPlanarArmOutputs(Y).hand_a];
            F = [F,Fpert];
        end
    end
end
% hand_0 = getHandPosition(thetaSH,thetaEL,sys);

figure(1); clf
hold all
m = (F./A);
histogram(m(1,:),-5:0.03:5)
histogram(m(2,:),-5:0.03:5)
legend('x','y')
title(sprintf("medians:   m_x = %.3f kg  |   m_y = %.3f kg",median(m(1,:)),median(m(2,:))))