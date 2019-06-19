% HW 7 Question 2
clc 
% clear all
close all
dbstop if error
% x(t+1) = Ax + Bu 
A = [1 1; 
    0 1];
B = [0;1];
Q = [1 0; 
    0 1];
R = 0.01;
P = [1 0;
    0 1];
% cost = xN'PxN + x'Qx + u'Ru
ubar = 1;
xbar = 1;
umin = -ubar;
umax = ubar;
xmin = [-xbar; -xbar];
xmax = [xbar; xbar];
% Pinf = solution to Ricatti
Pinf = solveDiscreteRiccati(A, B, Q, R);

%% Part b
% xbar = 5;
% ubar = 0.5;
% umin = -ubar;
% umax = ubar;
% xmin = [-xbar; -xbar];
% xmax = [xbar; xbar];
% N = 3;
% P = [1 0;
%     0 1];
% R = 10;
% x0 = [-4.5; 2];
% Xf = []; % Represents no constraint on Xf
% [Xallmpc, Uallmpc, horvalmpc, ~] = solveMPC(N, x0, A, B, P, Q, R, xmax, xmin, umax, umin, Xf);
% tvec = 0:50;
% figure
% plot(Xallmpc(1,:),Xallmpc(2,:),'.-b')
% hold on
% x0 = [-4.5; 3];
% [Xallmpc, Uallmpc, horvalmpc, lastIter] = solveMPC(N, x0, A, B, P, Q, R, xmax, xmin, umax, umin, Xf);
% figure(1)
% plot(Xallmpc(1,1:lastIter),Xallmpc(2,1:lastIter),'-dr')
% title('Closed loop trajectories under receding horizon control MPC')
% legend('Initial x=[-4.5,2]','Initial x=[-4.5,3]')
% xlabel('x1')
% ylabel('x2')
% grid on

 
%% Part c 
% xbar = 10;
% ubar = 1;
% umin = -ubar;
% umax = ubar;
% xmin = [-xbar; -xbar];
% xmax = [xbar; xbar];
% N = 2;
% R = 0.01;
% P = Pinf;
% Xf = 0;
% numPoints = 10; % TODO change to 10 (or more)
% figure
% for x1=linspace(-xbar,xbar,numPoints)
%     for x2=linspace(-xbar,xbar,numPoints)
%         x0 = [x1;x2];
%         fprintf('Initial point (%.2f, %.2f)\n',x1,x2);
%         solvehorizon=50; % TODO change to 50
%         [Xallmpc, Uallmpc, horvalmpc, lastIter] = solveMPC(N, x0, A, B, P, Q, R, xmax, xmin, umax, umin, Xf, solvehorizon);
%         if lastIter>1
%             plot(Xallmpc(1,1:lastIter),Xallmpc(2,1:lastIter),'-d')% 
%             hold on
%         end
%     end
% end
% title('Q2c - Domain of attraction with horizon 2 and Xf = 0')
% xlabel('x1')
% ylabel('x2')
% grid on


%% Part d 
% xbar = 10;
% ubar = 1;
% umin = -ubar;
% umax = ubar;
% xmin = [-xbar; -xbar];
% xmax = [xbar; xbar];
% N = 6;
% R = 0.01;
% P = Pinf;
% Xf = 0;
% numPoints = 10; % TODO change to 10 (or more)
% figure
% for x1=linspace(-xbar,xbar,numPoints)
%     for x2=linspace(-xbar,xbar,numPoints)
%         x0 = [x1;x2];
%         fprintf('Initial point (%.2f, %.2f)\n',x1,x2);
%         solvehorizon=50; % TODO change to 50
%         [Xallmpc, Uallmpc, horvalmpc, lastIter] = solveMPC(N, x0, A, B, P, Q, R, xmax, xmin, umax, umin, Xf, solvehorizon);
%         if lastIter>1
%             plot(Xallmpc(1,1:lastIter),Xallmpc(2,1:lastIter),'-d')
%             hold on
%         end
%     end
% end
% title('Q2d - Domain of attraction with horizon 6 and Xf = 0')
% xlabel('x1')
% ylabel('x2')
% grid on


%% Part e 
xbar = 10;
ubar = 1;
umin = -ubar;
umax = ubar;
xmin = [-xbar; -xbar];
xmax = [xbar; xbar];
N = 2;
R = 0.01;
P = Pinf;
Xf = [];
numPoints = 10; % TODO change to 10 (or more)
figure
for x1=linspace(-xbar,xbar,numPoints)
    for x2=linspace(-xbar,xbar,numPoints)
        x0 = [x1;x2];
        fprintf('Initial point (%.2f, %.2f)\n',x1,x2);
        solvehorizon=50; % TODO change to 50
        [Xallmpc, Uallmpc, horvalmpc, lastIter] = solveMPC(N, x0, A, B, P, Q, R, xmax, xmin, umax, umin, Xf, solvehorizon);
        if lastIter>1
            plot(Xallmpc(1,1:lastIter),Xallmpc(2,1:lastIter),'-d')
            hold on
        end
    end
end
title('Q2e - Domain of attraction with horizon 2 and and Xf free in 2D')
xlabel('x1')
ylabel('x2')
grid on


%% Part f 
xbar = 10;
ubar = 1;
umin = -ubar;
umax = ubar;
xmin = [-xbar; -xbar];
xmax = [xbar; xbar];
N = 6;
R = 0.01;
P = Pinf;
Xf = [];
numPoints = 10; % TODO change to 10 (or more)
figure
for x1=linspace(-xbar,xbar,numPoints)
    for x2=linspace(-xbar,xbar,numPoints)
        x0 = [x1;x2];
        fprintf('Initial point (%.2f, %.2f)\n',x1,x2);
        solvehorizon=50; % TODO change to 50
        [Xallmpc, Uallmpc, horvalmpc, lastIter] = solveMPC(N, x0, A, B, P, Q, R, xmax, xmin, umax, umin, Xf, solvehorizon);
        if lastIter>1
            plot(Xallmpc(1,1:lastIter),Xallmpc(2,1:lastIter),'-d')
            hold on
        end
    end
end
title('Q2f - Domain of attraction with horizon 6 and Xf free in 2D')
xlabel('x1')
ylabel('x2')
grid on

%% Part h
xbar = 10;
ubar = 1;
umin = -ubar;
umax = ubar;
xmin = [-xbar; -xbar];
xmax = [xbar; xbar];
R = 0.01;
P = Pinf;
Xf = 0;
numPoints = 5; % TODO change to 10 (or more)
x0 = [-1;1]; % TODO change required?
% N_vals = [2;4;8];
N_vals = [2; 3; 4; 5; 6; 7; 8; 10; 20; 30];
mpccosts = zeros(length(N_vals),1);
figure
for i = 1: length(N_vals)
    N = N_vals(i);
    fprintf('N=%d\n',N);
    solvehorizon=50; % TODO change to 50
    [Xallmpc, Uallmpc, horvalmpc, lastIter] = solveMPC(N, x0, A, B, P, Q, R, xmax, xmin, umax, umin, Xf, solvehorizon);
    mpccosts(i) = horvalmpc;
    lblstr = sprintf('N=%d',N);
    if lastIter>1
        plot(Xallmpc(1,1:lastIter),Xallmpc(2,1:lastIter),'-d','DisplayName',lblstr)% TODO this produces different colors right?
%             plot(Xallmpc(1,1:lastIter),Xallmpc(2,1:lastIter),'-d','color',rand(1,3),'markersize',10,'DisplayName',lblstr)
        hold on
    end
end
title('Q2h - Domain of attraction with different horizons and Xf=0')
xlabel('x1')
ylabel('x2')
legend show
grid on
T = array2table([N_vals, mpccosts],...
    'VariableNames',{'N','Cost'})