function Jdot = sysEQ_Jdot(x,sys)

Jdot = [-sys.a1*cos(x(1))*x(3) - sys.a2*cos(x(1)+x(2))*(x(3)+x(4)),   -sys.a2*cos(x(1)+x(2))*(x(3)+x(4));
        -sys.a1*sin(x(1))*x(3) - sys.a2*sin(x(1)+x(2))*(x(3)+x(4)),   -sys.a2*sin(x(1)+x(2))*(x(3)+x(4))];