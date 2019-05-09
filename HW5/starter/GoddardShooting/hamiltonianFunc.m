% Hamiltonian related to the Goddard's problem.

function H = hamiltonianFunc(y,v,m,py,pv,pm)

global g;
global b;
global uMax;

% Compute phi
phi = pv/m - pm*b;

% Use phi to compute control action
if phi > 0
    uStar = 0;
else
    uStar = uMax;
end

% Return Hamiltonian H(y,v,m,ph,pv,pm)
H = py*(v) + pv*(uStar/m - g) +  pm*(-b*uStar);