%%% Advanced Econometrics: Problem Set 3
%%% Jieyang Wang, UNI: 4067
clear;
clc;

%% Question 1) - Defining functions
% h is hypothesis function
% g is sigmoid function
% cost is cost function

%% Question 1) - Gradient Descent
x = [0.5; 0.75; 1; 1.25; 1.5; 1.75; 2; 2.25; 2.5; 2.75; 3; 3.25; 3.5;...
    3.75; 4; 4.25; 4.5; 4.75; 5; 5.5];
y = [0; 0; 0; 0; 0; 1; 0; 1; 0; 1; 0; 1; 1; 1; 1; 1; 1; 1; 1; 1];
theta = [0,0]; % Initial values

% Parameters
Gamma = 0.1;    % step size (learning rate)
MAX_ITER = 20000;  % maximum number of iterations
FUNC_TOL = 0.00001;   % termination tolerance
fvals = [];       % store F(x) values across iterations

% gradient descent method
iter = 1;      % iterations counter
F = @(theta) cost(x,y,theta,@h); % objective function F
fvals(iter) = F(theta);

while iter < MAX_ITER  && abs(fvals(iter)) > FUNC_TOL
    iter = iter + 1;
    A = grad1(x,y,theta);
    theta = theta - Gamma * A;  % gradient descent method
    fvals(iter) = F(theta);     % evaluate objective function
end

% Plot
figure(1);
plot(1:iter, fvals, 'LineWidth',2); grid on;
title('Objective Function under Gradient Descent'); xlabel('Iteration'); ylabel('F(x)');

% Display of answers
disp(['Number of iterations for Gradient method is: ' num2str(iter(end))])
disp('Theta coefficients provided by formula are:')
disp(theta(1))
disp(theta(2))


%% Question 1) - Matlab Optimization Routine
theta1 = [0,0]; % set initial value
F = @(theta1) cost(x,y,theta1,@h);
thetafmin = fminsearch(F,theta1); 
disp(['Theta_0 = ' num2str(thetafmin(1)) ', Theta_1 = ' num2str(thetafmin(2))]);

% The answer we get is the same as using our own gradient method.
% The answer differs from what was shown in the slides due to differences
% in data points used. The shape of the graph though is similar.

%% Question 2.1) - Reading data and graphs
clear;
clc;
M = readmatrix('dataset_training for PS3.txt');
x = M(:,1); % horizontal line
y = M(:,2); % vertical line
z = M(:,3); % blue for 0 and yellow for 1
pointsize = 30;
figure(2);
scatter(x, y, pointsize, z,'filled');

%% Question 2.2) - Gradient Descent
% Given the graph, it looks like the boundary between z=0 and z=1 could be
% linear. Hence I will try the hypothesis that the boundary
% relationship is linear.
% h1 = g(theta_0 + theta_1*x + theta_2*y).

x2 = x.^2;
y2 = y.^2;
X1 = [x,y];
theta = [-25.16,0,0]; % set initial values

% Parameters
Gamma = 0.00001;    % step size (learning rate)
MAX_ITER = 20000;  % maximum number of iterations
FUNC_TOL = 0.001;   % termination tolerance
fvals = [];       % store F(x) values across iterations

% gradient descent method
iter = 1;      % iterations counter
F = @(theta) cost2(X1,z,theta,@h1); % objective function F
fvals(iter) = F(theta);
while iter < MAX_ITER  && abs(fvals(iter)) > FUNC_TOL
    iter = iter + 1;
    [A1, A2, A3] = grad2(X1,z,theta);
    theta = theta - Gamma * [A1, A2, A3];  % gradient descent method
    fvals(iter) = F(theta);     % evaluate objective function
end

% Plot
figure(3);
plot(1:iter, fvals, 'LineWidth',2); grid on;
title('Objective Function under Gradient Descent'); xlabel('Iteration'); ylabel('F(x)');

% Display of answers
disp(['Number of iterations for Gradient method is: ' num2str(iter(end))])
disp('Theta coefficients provided by formula are:')
disp(theta)

figure(4);
hold on
scatter(x, y, pointsize, z,'filled');
f = @(x,y) theta(1)+theta(2)*x+theta(3)*y;
fimplicit(f,[30 100 30 100]);
hold off;

%% Question 2.3) - Matlab Code
X = [x,y,x2,y2];
X2 = [x2,y2];
z1 = categorical(z);

% Testing linear hypothesis
B = mnrfit(X1,z1);
figure(5);
hold on
scatter(x, y, pointsize, z,'filled');
f = @(x,y) B(1)+B(2)*x+B(3)*y;
fimplicit(f,[30 100 30 100]);
hold off;

% The linear result is the same result we got from gradient descent

% Testing potential nonlinear hypothesis (1)
B2 = mnrfit(X,z1);
figure(6);
hold on
scatter(x, y, pointsize, z,'filled');
f = @(x,y) B2(1)+B2(2)*x+B2(3)*y+B2(4)*x.^2+B2(5)*y.^2;
fimplicit(f,[30 100 30 100]);
hold off;

% It appears that there is a better option for boundary using a nonlinear
% hypothesis function, as figure 5 shows. It is possible to model a
% boundary that captures all current data points correctly.