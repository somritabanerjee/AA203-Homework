%% AA 203 HW 1 Question 4 part g
% Somrita Banerjee
clc
clear all
close all
k = 1;
eps = 0.01;
for initCond = 1:4
    switch initCond
        case 1
            x{1} = [5;1];
            y{1} = [5;1];
            gamma = 10;
        case 2
            x{1} = [1;5];
            y{1} = [1;5];
            gamma = 10;
        case 3
            x{1} = [5;1];
            y{1} = [5;1];
            gamma = 2;
        case 4
            x{1} = [1;5];
            y{1} = [1;5];
            gamma = 2;
    end
    k=1;
    while norm(gradF(x{k},gamma))>eps
        % Constant step size 
        eta = 0.05;
        x{k+1}= x{k} - eta.*gradF(x{k},gamma);
        k = k+1;
    end
    k = 1;
    while norm(gradF(y{k},gamma))>1
        % Exact line search
        d = -gradF(y{k},gamma);
        Q = [1 0; 0 gamma];
        eta = norm(d)/(d'*Q*d);
        y{k+1}= y{k} - eta.*gradF(y{k},gamma);
        k = k+1;
    end
    if initCond == 1
        xMatInit1 = cell2mat(x);
        yMatInit1 = cell2mat(y);
    elseif initCond == 2
        xMatInit2 = cell2mat(x);
        yMatInit2 = cell2mat(y);
    elseif initCond == 3
        xMatInit3 = cell2mat(x);
        yMatInit3 = cell2mat(y);
    elseif initCond == 4
        xMatInit4 = cell2mat(x);
        yMatInit4 = cell2mat(y);
    end
end
figure
hold on
plot(xMatInit1(1,:),xMatInit1(2,:),'--.r')
plot(xMatInit2(1,:),xMatInit2(2,:),'--.b')
title('Constant step size with \gamma = 10');
legend('Initial position (5,1)','Initial position (1,5)')
figure
hold on
plot(xMatInit3(1,:),xMatInit3(2,:),'--.r')
plot(xMatInit4(1,:),xMatInit4(2,:),'--.b')
title('Constant step size with \gamma = 2');
legend('Initial position (5,1)','Initial position (1,5)')
figure
hold on
plot(yMatInit1(1,:),yMatInit1(2,:),'--.r')
plot(yMatInit2(1,:),yMatInit2(2,:),'--.b')
title('Exact line search with \gamma = 10');
legend('Initial position (5,1)','Initial position (1,5)')
figure
hold on
plot(yMatInit3(1,:),yMatInit3(2,:),'--.r')
plot(yMatInit4(1,:),yMatInit4(2,:),'--.b')
title('Exact line search with \gamma = 2');
legend('Initial position (5,1)','Initial position (1,5)')
