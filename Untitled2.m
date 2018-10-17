%% Part I - Transferring a robot
% Going from x_init to x_final

%% Constant definition
% 
clear all;
close all;
%A and b matrix
A = [1 0 0.1 0; 0 1 0 0.1; 0 0 0.9 0; 0 0 0 0.9];
b = [0 0; 0 0; 0.1 0; 0 0.1];

%position
E = [1 0 0 0;0 1 0 0];
% Regularizition Parameter
lambda = [0.001 0.01 0.1 1 10 100 1000];

% Elapsed Time
T = 81;

%Position x = [p v]
p_init = [0 5];
p_final = [15 -15];
x_init = [0; 5; 0; 0];
x_final = [15; -15; 0; 0];

% Waypoints Position 
K = 6;
w=[10 20 30 30 20 10; 10 10 10 0 0 -10];

% Desired Time on Waypoint
tau = [10 25 30 40 50 60];

% Maximum force
U_max = 100;

%% CVX algorithm
% 
cvx_begin quiet
    variable x(4,T);
    variable u(2,T); 
    
    p1_cost=0;
    p2_cost=0;
    for i=1:K
       p1_cost = p1_cost + pow_pos(norm(E*x(:,tau(:,i)) - w(:,i)),2);  
    end
    for j=2:T-1
        p2_cost = p2_cost +  pow_pos(norm(u(:,j) - u(:,j-1)),2);
    end
    cost = p1_cost + lambda(3)*p2_cost;
    minimize(cost)
    subject to
        x(:,1) == x_init;
        x(:,T) == x_final;
        for t=1:T-1
            norm(u(:,t)) <= U_max;
        end
        for t=1:T-1
           x(:,t+1) == A*x(:,t) + b*u(:,t); 
        end
cvx_end

%% a) - Optimal Positions
%

figure
% plot optimal positions
plot(x(1,:), x(2,:),'o','MarkerSize', 3,'LineWidth', 2)
hold on

% plot waypoints
plot(w(1,:), w(2,:), 's','MarkerSize',10, 'LineWidth', 2)

% plot the time tau closest to waypoint
for i=1:6
    aux(:,i) = [x(1,tau(i)) x(2,tau(i))]';
end
plot(aux(1,:), aux(2,:), 'mo','MarkerSize', 10)

%% b) Control Signal
%

figure 
plot(linspace(0, 79,80), u(1,:))
hold on
plot(linspace(0, 79,80), u(2,:))
%% c) - Count how many times u(t) has changed
%
count = 0;
for i=2:T-1
    change=norm(u(:,i) - u(:,i-1));
    if change > 10^-6
        count = count +1;
    end
end
fprintf('u(t) has changed: %d times \n', count)
