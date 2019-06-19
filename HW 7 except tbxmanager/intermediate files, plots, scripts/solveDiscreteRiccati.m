% Check Riccati solver
function Pinf = solveDiscreteRiccati(A, B, Q, R)
if nargin == 0
    A = [1 1; 
    0 1];
    B = [0;1];
    Q = [1 0; 
        0 1];
    R = 0.01;
end
% Own function
Pinf = [1 1; 1 1];
for i = 1:10
    Pinfnew = Q + A'*Pinf*A - A'*Pinf*B*inv(B'*Pinf*B + R)*B'*Pinf*A;
    Pinf = Pinfnew;
end

% MATLABfunctions
% [X,K,L] = idare(A,B,Q,R,S,E)
[X,L,G] = dare(A,B,Q,R,[],[]);
[K,S,E] = dlqr(A, B, Q, R);

if ~(norm(X-S)<1e-3 && norm(X-Pinf)<1e-3)
    fprintf('Not close to equal.\n')
end
end
