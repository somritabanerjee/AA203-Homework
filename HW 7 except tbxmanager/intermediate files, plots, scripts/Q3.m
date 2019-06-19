%% HW 7 Question 3
clear all
close all
clc
A = [1 1 0;
    0 0.9 1;
    0 0.2 0];
B = [0; 1; 0];
% computes a control invariant set for LTI system x^+ = A*x+B*u
system = LTISystem('A', A, 'B', B);
system.x.min = [-5; -5; -5];
system.x.max = [5; 5; 5];
system.u.min = -0.5;
system.u.max = 0.5;
InvSet = system.invariantSet()
InvSet.plot()
xlabel('x1')
ylabel('x2')
zlabel('x3')
title('Invariant set')