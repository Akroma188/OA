%% Part I - Transferring a robot
% Going from x_init to x_final

function main
    close all
    clear all
    lambda = [0.001 0.01 0.1 1 10 100 1000];
%% Variation A
%    

%% 2.2
%
    for i=1:7
       [c, d] = p1_variationA(lambda(i), 2,2);
       count(i)=c;
       dev(i)=d;
    end
    % plot number of times u(t) changes for all lambdas
    figure
    bar(count)
    title('Times u(t) has changed', 'Interpreter', 'Latex')
    xlabel('$ \lambda $','Interpreter', 'Latex')
    ylabel('number ','Interpreter', 'Latex')
    
    % plot deviation for all lambdas
    figure
    bar(dev)
    title('Mean Deviation', 'Interpreter', 'Latex')
    xlabel('$ \lambda $','Interpreter', 'Latex')
    ylabel('deviation ','Interpreter', 'Latex')
%% 2.3
%

    for i=1:7
       [c, d] = p1_variationA(lambda(i), 1,2);
       count(i)=c;
       dev(i)=d;
    end
    %plot number of times u(t) changes for all lambdas
    figure
    bar(count)
    title('Times u(t) has changed', 'Interpreter', 'Latex')
    xlabel('$ \lambda $','Interpreter', 'Latex')
    ylabel('number ','Interpreter', 'Latex')
    
    %plot deviation for all lambdas
    figure
    bar(dev)
    title('Mean Deviation', 'Interpreter', 'Latex')
    xlabel('$ \lambda $','Interpreter', 'Latex')
    ylabel('deviation ','Interpreter', 'Latex')
    
%% 2.4
%    
    for i=1:7
       [c, d] = p1_variationA(lambda(i), 1,1);
       count(i)=c;
       dev(i)=d;
    end
    %plot number of times u(t) changes for all lambdas
    figure
    bar(count)
    title('Times u(t) has changed', 'Interpreter', 'Latex')
    xlabel('$ \lambda $','Interpreter', 'Latex')
    ylabel('number ','Interpreter', 'Latex')
    
    %plot deviation for all lambdas
    figure
    bar(dev)
    title('Mean Deviation', 'Interpreter', 'Latex')
    xlabel('$ \lambda $','Interpreter', 'Latex')
    ylabel('deviation ','Interpreter', 'Latex')
%% Variation B
%    

p1_variationB()

%% Variation C
%    

%% 4.2 and 4.3
%

    n_power = [2 1];
    
    [x_old, u_old, count] = p1_variationC(n_power(1));
    [x_old, u_old, count] = p1_variationC(n_power(2));
    

%% 4.4
% 
    M = 10
    
    for m=1:M
        [x_old, u_old] = p1_variationC_weights(x_old, u_old, m);
    end


end