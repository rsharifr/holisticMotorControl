function J = sysEQ_J(x,sys)

J = [-sys.a1*sin(x(1))-sys.a2*sin(x(1)+x(2)),   -sys.a2*sin(x(1)+x(2));
     sys.a1*cos(x(1))+sys.a2*cos(x(1)+x(2)),    sys.a2*cos(x(1)+x(2))];
