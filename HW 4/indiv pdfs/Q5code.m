%% AA 203 Homework 4
% Somrita Banerjee
clc
clear all
close all
t = linspace(0,1,100);
uStar = (-8/(exp(4)-1)).*exp(2.*t);
a = -2/(exp(4)-1);
b = 2*exp(4)/(exp(4)-1);
xStar = a.*exp(2.*t) + b.*exp(-2.*t);
figure
plot(t, xStar, t, uStar)
legend('Optimal state response','Optimal control')
xlabel('Time')
ylabel('x* or u*')
title('Optimal control and state trajectory');


