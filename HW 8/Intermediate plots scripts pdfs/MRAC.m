function dxdt=MRAC(t,x,caseNum)
% x = [y ym kr ky]
if caseNum == 1
    r = 4;
elseif caseNum == 2
    r = 4*sin(3*t);
else
    fprintf('Error case number not recognized.\n');
end

y = x(1);
ym = x(2);
kr = x(3);
ky = x(4);

a = -1; b = 3; am = 4; bm = 4; gamma = 2;

u = kr * r + ky * y;
e = y - ym;

ydot = -a*y + b*u;
ymdot = -am*ym + bm*r;
krdot = -gamma * e * r;
kydot = -gamma * e * y;

dxdt = [ydot; ymdot; krdot; kydot];
end