function xd1 = xd1(t)
%XD1
%    XD1 = XD1(T)

%    This function was generated by the Symbolic Math Toolbox version 7.2.
%    13-Sep-2018 10:53:41

t2 = conj(t);
t3 = t2.*pi.*(1.0./5.0);
xd1 = [sin(t3).*(3.0./4.0e1);0.0;cos(t3).*(3.0./4.0e1)];
