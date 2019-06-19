%% AA 203 HW 8 Question 2
% Somrita Banerjee
clc
clear all 
close all
%% Part d
% x = [y ym kr ky]
tspan = [0 10];
a = -1; b = 3; am = 4; bm = 4;
krStar = bm/b;
kyStar = (a-am)/b;
xInit = [0 0 0 0];
options=odeset('RelTol',1e-3,'AbsTol',1e-6);

[t1,x1] = ode45(@(t,x) MRAC(t,x,1), tspan, xInit, options);
[t2,x2] = ode45(@(t,x) MRAC(t,x,2), tspan, xInit, options);

figure 
hold on
plot(t1, x1(:,1), 'r')
plot(t1, x1(:,2), 'b')
legend({'$$y(t)$$','$$y_m(t)$$'},'Interpreter','latex','FontSize',20)
xlabel('Time t','Interpreter','latex','FontSize',20)
title('Evolution of true and reference model with $$r(t) = 4$$','Interpreter','latex','FontSize',15)
grid on

figure 
hold on 
plot(t1, x1(:,3),'r')
plot(t1, krStar*ones(size(t1)),'--r')
plot(t1, x1(:,4),'b')
plot(t1, kyStar*ones(size(t1)),'--b')
legend({'$$k_r(t)$$','$$k_r^*$$','$$k_y(t)$$','$$k_y^*$$'},'Interpreter','latex','FontSize',20)
xlabel('Time t','Interpreter','latex','FontSize',20)
title('Evolution of feedback gains with $$r(t) = 4$$','Interpreter','latex','FontSize',15)
grid on

figure 
hold on
plot(t2, x2(:,1), 'r')
plot(t2, x2(:,2), 'b')
legend({'$$y(t)$$','$$y_m(t)$$'},'Interpreter','latex','FontSize',20)
xlabel('Time t','Interpreter','latex','FontSize',20)
title('Evolution of true and reference model with $$r(t) = 4\sin(3t)$$','Interpreter','latex','FontSize',15)
grid on

figure 
hold on 
plot(t2, x2(:,3),'r')
plot(t2, krStar*ones(size(t2)),'--r')
plot(t2, x2(:,4),'b')
plot(t2, kyStar*ones(size(t2)),'--b')
legend({'$$k_r(t)$$','$$k_r^*$$','$$k_y(t)$$','$$k_y^*$$'},'Interpreter','latex','FontSize',20)
xlabel('Time t','Interpreter','latex','FontSize',20)
title('Evolution of feedback gains with $$r(t) = 4\sin(3t)$$','Interpreter','latex','FontSize',15)
grid on