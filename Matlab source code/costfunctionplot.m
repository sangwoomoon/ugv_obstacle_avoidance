clear all;
close all;

x = 0:0.2:0.2*(31-1);
y = 0:0.2:0.2*(100-1);

% goal point

% threat obstacle

for iter = 1 : 31
    for iter_j = 1 : 99
        cost(iter,iter_j) = (0.5*cos(x(iter))+0.6)*exp(-(y(iter_j)/y(iter_j+1)));
    end
end

figure(2)
surf(y(1:99),x,cost);
% shading interp;
xlabel('distance (meter)')
ylabel('\theta (rad)')
zlabel('cost')

% agent

for iter = 1 : 31
    for iter_j = 1 : 99
        cost(iter,iter_j) = 0.3*(0.5*cos(x(iter))+0.6)*exp(-(y(iter_j)));
    end
end

figure(3)
surf(y(1:99),x,cost);
% shading interp;
xlabel('distance (meter)')
ylabel('\theta (rad)')
zlabel('cost')

% swarm


for iter = 1 : 31
    for iter_j = 1 : 99
        cost_goal(iter,iter_j) = 1-exp(-(y(iter_j))/y(iter_j+1));
    end
end

x = 0:0.2:0.2*(31-1);
y = 0:0.2:0.2*(100-1);

figure(4)
surf(y(1:99),x,cost_goal);
% shading interp;
xlabel('distance (meter)')
ylabel('\theta (radian)')
zlabel('cost')
