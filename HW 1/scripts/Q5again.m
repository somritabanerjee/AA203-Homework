%% AA 203 HW 1 Question 5 again
% Somrita Banerjee
clc
clear all
close all
Q = eye(2);
QT = 10 * eye(2);
R = eye(1);
A=[1 1; 0 1];
B=[0;1];
x0=[1;0];
T=20;
btilde = zeros(T,1);
Qtilde = zeros(T,T);
Qhat = blkdiag(kron(eye(20),Q),QT);
Atilde=eye(2);
for i = 1:T
    Atilde=[Atilde;A^i];
end
Btilde = zeros((T+1)*2,T);
for i=1:T
    for j=1:i
        Btilde(2*i+1: 2*i+2,j)=(A^(i-j)) *B;
    end
end
Rtilde = kron(eye(20),R);
Qtilde = Btilde'*Qhat*Btilde + Rtilde;
btilde = -(x0'*Atilde'*Qhat*Btilde)';
uStar = Qtilde\btilde

u= uStar;
x = zeros(2, T+1);
x(:,1) = x0;
sumJ = 0;
for t = 0:T-1
    x(:,t+2) = A*x(:,t+1) + B*u(t+1);
    sumJ = sumJ + x(:,t+1)'*Q*x(:,t+1) + u(t+1)'*R*u(t+1);
end
J = x(:,T+1)'*QT*x(:,T+1) + sumJ
