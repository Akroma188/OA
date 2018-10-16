%% Variotion A function
% Going from x_init to x_final

% n = norm to the power n // m = norm value
% Plots Optimal Position and Control Signal
% Returns number of times u(t) has changed and the deviation for each lambda
function [count, dev] = variationA(lambda, n_power, m)
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
           p1_cost = p1_cost + pow_pos(norm(E*x(:,tau(:,i)) - w(:,i)), 2);  
        end
        for j=2:T-1
            if n_power>1
                p2_cost = p2_cost +  pow_pos(norm(u(:,j) - u(:,j-1), m), n_power);
            else
                p2_cost = p2_cost +  norm(u(:,j) - u(:,j-1), m);
            end
        end
        cost = p1_cost + lambda*p2_cost;
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
    legend('p(t)','\omega_k','p(\tau_k)','Interpreter', 'Latex')
    if n_power == 2
        tle = strcat('\begin{tabular}{c} Optimal Position \\', '$ {l}_', num2str(m),'^',num2str(n_power),' $',' Regularizer',' // ',strcat('$ \lambda $ = ', num2str(lambda)), '\end{tabular}');
    else
        tle = strcat('\begin{tabular}{c} Optimal Position \\', '$ {l}_', num2str(m),' $',' Regularizer',' // ',strcat('$ \lambda $ = ', num2str(lambda)), '\end{tabular}');
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
    legend(' u_{1} ', ' u_{2} ','Interpreter', 'Latex')
    if n_power == 2
        tle = strcat('\begin{tabular}{c} Control Signal \\', '$ {l}_', num2str(m),'^',num2str(n_power),' $',' Regularizer',' // ',strcat('$ \lambda $ = ', num2str(lambda)), '\end{tabular}');
    else
        tle = strcat('\begin{tabular}{c} Control Signal \\', '$ {l}_', num2str(m),' $',' Regularizer',' // ',strcat('$ \lambda $ = ', num2str(lambda)), '\end{tabular}');
    end
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

%% d) - Deviation
%
    dev = 0
    for k=1:K
        dev = dev + norm(E*x(:,tau(:,k)) - w(:,k));
    end
    dev = (1/K) * dev;
    %fprintf('deviation = %f for lamda = %f \n', dev, lambda)


end