clc;clear; close all;
delta_x = 2.5;
delta_y = 0.1;
x_grid = delta_x:delta_x:120-delta_x;
y_grid = 0:delta_y:100;
J_star = zeros(length(x_grid),length(y_grid));
tic
for h = 1:length(y_grid)
    J_star(end,h) = sqrt(delta_x^2 + ((h*delta_y-delta_y)-50)^2);
end

for k = length(x_grid)-1:-1:1
    for p = 1:length(y_grid)
        cost = [];
        for d = 1:length(y_grid)
            cost(d) = sqrt(delta_x^2 + (delta_y*(p-1)-delta_y*(d-1))^2) + J_star(k+1,d);
            if (delta_x*(k-1) - 60)^2 + (delta_y*(p-1) - 50)^2 < 15^2
                cost(d) = 10000000;
            end
        end
        [weight,index] = min(cost);
        J_star(k,p) = weight;
        path(k,p) = index;
    end
end

cost = [];
for d = 1:length(y_grid)
    cost(d) = sqrt(delta_x^2 + (50-delta_y*(d-1))^2) + J_star(1,d);
    if (delta_x*(k-1) - 60)^2 + (delta_y*(p-1) - 50)^2 < 15^2
        cost(d) = 10000000;
    end
end

[A,B] = min(cost);
J_star_initial = A;
path_initial = B;

traj(1) = 50/delta_y + 1;
traj(2) = B;
for m = 2:length(x_grid)-1
    traj(m+1) = path(m,traj(m));
end
traj = (traj-1)*delta_y;
traj(end+1) = 50;
traj(end+1) = 50;
toc

x_initial=0;y_initial=50;
x_final=120;y_final=50;

r1=15;x1=60;y1=50;
th = 0:pi/50:2*pi;
obtacle_1_x = r1 * cos(th) + x1;
obtacle_1_y = r1 * sin(th) + y1;

x_grid = 0:delta_x:120;

%% Control Input Calculation
for i = 1:length(x_grid)-1
    ksi(i) = atan((traj(i+1)-traj(i))/delta_x);
end
delta_t = sqrt((traj(2:end)-traj(1:end-1)).^2 + delta_x^2)/10;
for i = 3:length(ksi)-1
    u(i) = atand((ksi(i+1)-ksi(i-1))/(delta_t(i)*delta_t(i+1)) * 10/9.81);
end
time_u=[];
time_u(1) = 0;
for i = 2:(length(delta_t)-1)
    time_u(i) = time_u(i-1) + delta_t(i-1);
end

%% Plotting Section
% Trajectory Plot
figure(1)
plot(x_initial,y_initial,'*b'); % Initial point
hold on;
plot(x_final,y_final,'*k'); % Terminal point
plot(obtacle_1_x,obtacle_1_y,'r'); % Obstacle
plot(x_grid,traj,'Color',[0, 0.4470, 0.7410],'LineWidth',2); % Optimal trajectory
ylim([0 100])
title('Trajectory of Aircraft obtained by dynamic programming')
legend('Initial Point','Terminal Point','Obtacles','Trajectory')
xlabel('X(m)')
ylabel('Y(m)')
grid on;
% Control Input Plot
figure(2)
plot(time_u,u,'Color',[0.6350, 0.0780, 0.1840],'LineWidth',2);
xlabel('Time (sec)');
ylabel('Bank angle, \phi (deg)');
title('Control Input Variation with Time');
grid on;