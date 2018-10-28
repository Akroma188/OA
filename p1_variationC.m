%% Variation C interesting functions
% exact intermediate points



function [x, u, count] = p1_variationC(n_power)

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

    %Position x = [p v]
    p_init = [0 5];
    p_final = [15 -15];
    x_init = [0; 5; 0; 0];
    x_final = [15; -15; 0; 0];

    % Waypoints Position 
    K = 6;
    w=[10 20 30 30 20 10; 10 10 10 0 0 -10];

    % Desired Time on Waypoint
    tau = [10 25 30 40 50 60] + 1;

    % Maximum force
    U_max = 15;

%% 4.2
% 


%% CVX algorithm
% 

cvx_begin quiet
        variable x(4,T);
        variable u(2,T); 

        p1_cost=0;
        % do first part of the sum
        
        for i=1:K
            if n_power > 1
                p1_cost = p1_cost + pow_pos(norm(E*x(:,tau(i)) - w(:,i), 2), n_power);
            else
                p1_cost = p1_cost + norm(E*x(:,tau(i)) - w(:,i), 2);
            end
        end

        cost = p1_cost;
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
    plot(w(1,:), w(2,:), 's','MarkerSize',10, 'LineWidth', 2)

    % plot the time tau closest to waypoint
    for i=1:6
        aux(:,i) = [x(1,tau(i)) x(2,tau(i))]';
    end
    plot(aux(1,:), aux(2,:), 'mo','MarkerSize', 10)
    legend('p(t)','\omega_k','p(\tau_k)','Interpreter', 'Latex')
    if n_power == 2
        tle = strcat('\begin{tabular}{c} Optimal Position \\', '$ {l}_', num2str(2),'^',num2str(n_power),' $',' Regularizer', '\end{tabular}');
    else
        tle = strcat('\begin{tabular}{c} Optimal Position \\', '$ {l}_', num2str(2),' $',' Regularizer', '\end{tabular}');
    end

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
    if n_power == 2
        tle = strcat('\begin{tabular}{c} Control Signal \\', '$ {l}_', num2str(2),'^',num2str(n_power),' $',' Regularizer', '\end{tabular}');
    else
        tle = strcat('\begin{tabular}{c} Control Signal \\', '$ {l}_', num2str(2),' $',' Regularizer', '\end{tabular}');
    end
    title(tle, 'Interpreter', 'Latex')
    xlabel('t','Interpreter', 'Latex')
    ylabel('$ u_{1,2} $ ','Interpreter', 'Latex')
    
%% c) - Count how many times the robot captures the waypoints
%
    count = 0;
    for k=1:K
       if norm(x(1:2,tau(k))-w(:,k)) <= 10^-6
           count = count +1;
       end
    end
    
    fprintf('The robot has captured %d waypoints of of 6 \n', count)
    
    
    
end