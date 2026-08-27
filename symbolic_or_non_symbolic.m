% Show an example of solving two simultaneous equations using 
% MATLAB (symbolic method or non-symbolic method)

syms x y

eq1 = 2*x + 3*y == 6;
eq2 = 4*x - y == 5;

solution = solve([eq1, eq2], [x, y]);

x_sol = solution.x;
y_sol = solution.y;

fprintf('X = %d, Y = %d\n', x_sol, y_sol)