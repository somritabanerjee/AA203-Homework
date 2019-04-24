% Goddard Space Launcher Model - AA203


%% Grid: generate a box-type grid of lower corner 'grid_min' and upper corner 'grid_max'

C = 100000;

grid_min = [0; 0; 250]; % Lower corner of computation domain
grid_max = [150500/C; 30/C; 500]; % Upper corner of computation domain
N = [20; 20; 50]; % Number of grid points per dimension
% pdDims = 3; % 3rd dimension is periodic
g = createGrid(grid_min, grid_max, N); % Generate the grid


%% Target set

toler = [500/C; 2.8/C; 125];
goal = [150000/C; 28/C; 375];
lower = goal - toler;
upper = goal + toler;
data0 = shapeRectangleByCorners(g, lower, upper);


%% Time vector

t0 = 0;
tMax = 6;
dt = 0.05;
tau = t0:dt:tMax;


%% Problem parameters

gValue = 9.81/C;
b = C*10^(-4);
uMax = 10000/C;
dMax = 1/C;
uMode = 'min'; % Minimize on controls
dMode = 'max'; % Maximize on disturbances


%% Pack problem parameters

% Define dynamic system
x0 = [0;0;500]; % Starting point
goddardLauncher = Goddard(x0, gValue, b, uMax, dMax);

% Put grid and dynamic systems into schemeData
schemeData.grid = g;
schemeData.dynSys = goddardLauncher;
schemeData.accuracy = 'veryHigh'; %set accuracy
schemeData.uMode = uMode;
schemeData.dMode = dMode;


%% Compute value function

HJIextraArgs.visualize = true; %show plot
HJIextraArgs.fig_num = 1; %set figure number
HJIextraArgs.deleteLastPlot = true; %delete previous plot as you update
data = HJIPDE_solve(data0, tau, schemeData, 'minVWithTarget', HJIextraArgs);
save('goddardAA203.mat', 'tau', 'g', 'data')


%% Visualize slices

load('goddardAA203.mat')

% Section for J(t=-4.5)
figure;
ind = find(tau==4.5);
visSetIm(g, data(:,:,:,ind));
title('Section for $$V(t=-4.5)$$','Interpreter','latex');

% Section for J(t=-5.5)
figure;
ind = find(tau==5.5);
visSetIm(g, data(:,:,:,ind));
title('Section for $$V(t=-5.5)$$','Interpreter','latex');

