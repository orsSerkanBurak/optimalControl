clc;clear;close all;
global N; N = 49; % Scenario
global T; T = 12; % Time
global v; v = 10; % Velocity
global l; l = 50; % Final value of y
global M; M = 120; % Final value of x

% Imposing constraints on the control
global uMax; uMax = pi/6;
% Initialization on the control
uInit = pi/12*uMax*ones(N,1);
% Initialization on the state
xInit = zeros(N+1,1); yInit = ones(N+1,1); ksiInit = pi/2*ones(N+1,1);
varInit = [xInit; yInit; uInit; ksiInit];

% Set up the initial guess
varInitGuess = zeros(4*N+3, 1);
tic
% Solve the BVP using fmincon
options = optimoptions('fmincon','Display','iter','Algorithm','sqp','MaxFunctionEvaluations',1000000000);
[var, Fval, convergence] = fmincon(@cost, varInitGuess, [], [], [], [], [], [], @constraint, options);
toc
% Extract the solution from the variable vector
xSol = var(1:N+1);
ySol = var(N+2:2*N+2);
uSol = var(2*N+3:3*N+2);

% Plotting the trajectory
x_initial = 0;
y_initial = 50;
x_final = 120;
y_final = 50;
th = 0:pi/50:2*pi;
r2 = 15; x1 = 60; y1 = 50;
obtacle_1_x = r2 * cos(th) + x1;
obtacle_1_y = r2 * sin(th) + y1;

figure(1);
plot(x_initial, y_initial, '*b'); % Initial point
hold on;
plot(x_final, y_final, '*k'); % Terminal point
plot(obtacle_1_x, obtacle_1_y, 'r'); % Obstacle
plot(xSol, ySol, 'Color', [0, 0.4470, 0.7410], 'LineWidth', 2);
axis([0 120 0 100]);
title('Trajectory');
legend('Initial Point', 'Terminal Point', 'Obstacles', 'Trajectory');
xlabel('X(m)');
ylabel('Y(m)');
grid on;

% Plotting the control input
figure(2);
plot(uSol, 'Color', [0.6350, 0.0780, 0.1840], 'LineWidth', 2);
xlabel('Time (sec)');
ylabel('Bank angle, \psi (deg)');
title('Control Input Variation with Time');
title('Control Input');
grid on;

function [c, ceq] = constraint(var)
    % Function providing equality and inequality constraints
    % ceq(var) = 0 and c(var) nle 0
    global N;
    global T;
    global v;
    global l;
    global M;
    % Note: var = [x;y;u;phi]
    x = var(1:N+1);
    y = var(N+2:2*N+2);
    u = var(2*N+3:3*N+2);
    psi = var(3*N+3:4*N+3);
    
    c = zeros(N+1, 1);
    ceq = zeros(4*N+5, 1);
    
    for i = 1:N
        [xDyn, yDyn, ksiDyn] = fDyn(x(i), y(i), psi(i), u(i));
        
        % Imposing obstacles for every point
        c(i) = 15^2 - (x(i)-60)^2 - (y(i)-50)^2;
        
        % Imposing dynamical constraints
        ceq(i) = x(i+1) - x(i) - (1.0*T/(1.0*N))*xDyn;
        ceq(i+N) = y(i+1) - y(i) - (1.0*T/(1.0*N))*yDyn;
        ceq(i+2*N) = psi(i+1) - psi(i) - (1.0*T/(1.0*N))*ksiDyn;
    end
    
    % Imposing initial and final conditions
    ceq(1+3*N) = x(1);
    ceq(1+3*N+1) = y(1) - 50;
    ceq(1+3*N+2) = psi(1);
    ceq(1+3*N+3) = x(N+1) - M;
    ceq(1+3*N+4) = y(N+1) - l;
end

function c = cost(var)
    global N;
    c = 0.0;
    x = var(1:N+1);
    y = var(N+2:2*N+2);
    
    for i = 1:N
        c = c + sqrt((x(i+1)-x(i))^2 + (y(i+1)-y(i))^2);
    end
end

function [xDyn, yDyn, psiDyn] = fDyn(x, y, psi, u)
    % Original dynamics of the problem
    global v;
    xDyn = v * cos(psi);
    yDyn = v * sin(psi);
    psiDyn = u;
end
