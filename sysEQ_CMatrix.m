function C = sysEQ_CMatrix(x,sys)

h = -sys.m2*sys.a1*sys.l2*sin(x(2));

C = [h*x(4),   h*(x(3)+x(4));
    -h*x(3),   0];