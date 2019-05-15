% Dynamics of the problem

function [yDyn,vDyn,mDyn] = fDyn(y,v,m,u)

global D;
global b;

g = gFunc(y);
rho = normRhoFunc(y);

% Put here the dynamics
yDyn = v;
vDyn = u/m - g - (D/m)*rho*v^2;
mDyn = -b*u;