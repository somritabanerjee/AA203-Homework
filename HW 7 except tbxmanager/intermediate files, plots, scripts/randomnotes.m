% Random notes
Qbar = blkdiag(Q,Q,P);
Sx = [eye(2); A; A^2];
Su = [zeros(2,1); B; A*B];
H = Su'*Qbar*Su + R;
F = Sx'*Qbar*Su;
Y = Sx'*Qbar*Sx;
Cost = u0'*H*u0 + 2*x0*F*u0 + x0'*Y*x0;

% Q4 online and offline trajectories same or close 
% Q4 use lyapunov final p or algebraic ricatti solution

% Question 4 -1 
% do we need to do Lyapunov analysis or just do it in code 
% what P do you choose? 
% lqr penalty