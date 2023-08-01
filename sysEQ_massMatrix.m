function M = sysEQ_massMatrix(x,sys)

M = zeros(2,2);

M(1,1) = sys.I1 + sys.m1*sys.l1^2 + sys.I2 + sys.m2*(sys.a1^2+sys.l2^2+2*sys.a1*sys.l2*cos(x(2)));
M(1,2) = sys.I2 + sys.m2*(sys.l2^2+sys.a1*sys.l2*cos(x(2)));
M(2,1) = sys.I2 + sys.m2*(sys.l2^2+sys.a1*sys.l2*cos(x(2)));
M(2,2) = sys.I2 + sys.m2*sys.l2^2;