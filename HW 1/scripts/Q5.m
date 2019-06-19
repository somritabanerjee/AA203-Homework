%% AA 203 HW 1 Question 5
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
for j = 0: T-1 
    sumb = 0;
    sumQ = 0;
    for i = j: T-1
        sumb = sumb + (-x0' * (A^i)' * Q * (A^i) * B);
        sumQ = sumQ + B' * (A^i)' * Q * (A^i) *B;
    end
    btilde(j+1,1) =  sumb - x0'*((A^T)'*QT*(A^(T-1-j))*B);
    Qtilde(j+1,j+1) = sumQ + R + B' * (A^(T-1-j))' * QT * (A^(T-1-j)) * B;
end
uStar = inv(Qtilde)*btilde;
x = zeros(2, 21);
x(:,1) = [1; 0];
i = 1;
u = uStar;

sumJ = 0;
for t = 0:T-1
    x(:,t+2) = A*x(:,t+1) + B*u(t+1);
    sumJ = sumJ + x(:,t+1)'*Q*x(:,t+1) + u(t+1)'*R*u(t+1);
end
J = x(:,T+1)'*QT*x(:,T+1) + sumJ

QuadCost = 0.5*u'*Qtilde*u - btilde'*u
