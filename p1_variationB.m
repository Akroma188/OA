%% Variation B function
% approximate intermediate regions and simple control signals


function [count, dev] = p1_variationB()

%% Constant Definition
% approximate intermediate regions and simple control signals

 %A and b matrix
    A = [1 0 0.1 0; 0 1 0 0.1; 0 0 0.9 0; 0 0 0 0.9];
    b = [0 0; 0 0; 0.1 0; 0 0.1];

    %position
    E = [1 0 0 0;0 1 0 0];
    % Regularizition Parameter
    %lambda = [0.001 0.01 0.1 1 10 100 1000];
    % Elapsed Time
    T = 81;
    lambda=0.1;
    %Position x = [p v]
    p_init = [0 5];
    p_final = [15 -15];
    x_init = [0; 5; 0; 0];
    x_final = [15; -15; 0; 0];

    % Waypoints Position and radius
    K = 6;
    c = [10 20 30 30 20 10; 10 10 10 0 0 -10];
    r = 2;
    
    % Desired Time on Waypoint
    tau = [10 25 30 40 50 60] + 1;

    % Maximum force
    U_max = 100;

    
%% CVX algorithm
% 
    cvx_begin quiet
        variable x(4,T);
        variable u(2,T); 

        p1_cost=0;
        p2_cost=0;
        % do first part of the sum
        for i=1:K
           x_aux = E*x(:,tau(i));
           norm_aux(1) = x_aux(1)-c(1,i);
           norm_aux(2) = x_aux(2)-c(2,i);
           d_aux = norm(norm_aux);

           d=  d_aux - r;
           p1_cost = p1_cost + pow_pos(d, 2);
        end
        % do second part of the sum
        for j=2:T-1
            p2_cost = p2_cost +  norm(u(:,j) - u(:,j-1));
        end
        cost = p1_cost + lambda*p2_cost;
        minimize(cost)
        % constraints
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
    plot(c(1,:), c(2,:), 'o','MarkerSize',10, 'LineWidth', 2)

    % plot the time tau closest to waypoint
    for i=1:6
        aux(:,i) = [x(1,tau(i)) x(2,tau(i))]';
    end
    plot(aux(1,:), aux(2,:), 'mo','MarkerSize', 10)
    legend('p(t)','D(c_k, r)','p(\tau_k)','Interpreter', 'Latex')

    tle = strcat('\begin{tabular}{c} Optimal Position \\','Variation B','\end{tabular}');

    title(tle, 'Interpreter', 'Latex')
    xlabel('$ p_1 $ ','Interpreter', 'Latex')
    ylabel('$ p_2 $','Interpreter', 'Latex')
    
%% b) Control Signal
%

    % plot util point T-1 
    figure 
    plot(linspace(0, 78,79), u(1, 1:79))
    hold on
    plot(linspace(0, 78,79), u(2, 1:79))
    legend(' u_{1}(t) ', ' u_{2}(t) ','Interpreter', 'Latex')
    tle = strcat('\begin{tabular}{c} Control Signal \\','Variation B', '\end{tabular}');
    title(tle, 'Interpreter', 'Latex')
    xlabel('t','Interpreter', 'Latex')
    ylabel('$ u_{1,2} $ ','Interpreter', 'Latex')

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
%% d) - Mean deviation
%
    dev = 0;
    for k=1:K
        x_aux = E*x(:,tau(k));
        norm_aux(1) = x_aux(1)-c(1,k);
        norm_aux(2) = x_aux(2)-c(2,k);
        d_aux = norm(norm_aux);

        d=  d_aux - r;
        dev = dev + d;
    end
    fprintf('deviation = %f for lamda = %f \n', dev, lambda)

end