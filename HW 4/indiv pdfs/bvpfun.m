function dxdt = bvpfun(t,x,k)
dxdt = sqrt(k*x^2 -1);
end