% Adjoints equations related to our rocket.

function zdot = Zdyn(t,z)
           
global g;
global b;
global uMax;

v = z(2);
m = z(3);
py = z(4);
pv = z(5);
pm = z(6);

% Compute phi
phi = pv/m - pm*b;
% Use phi to compute control action
if phi > 0
    uStar = 0;
else
    uStar = uMax;
end
% Rocket dynamics and adjoint equations
yDot = v;
vDot = uStar/m - g;
mDot = -b*uStar;
pyDot = 0;
pvDot = -py;
pmDot = pv*uStar/(m^2);

zdot = [yDot; vDot; mDot; pyDot; pvDot; pmDot];