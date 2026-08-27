% Create a program where you will use a for-loop to generate 
% 10 random numbers and generate a plot (with x axis being 1 to 10 
% and y axis being the random number generated). 
% Include axis labels for the generated plot.

% array of 10 values
numbers = zeros(1,10);

for i = 1:10
    numbers(i) = rand();
end

% x-axis
x = 1:10; 

% plot
plot(x, numbers, '-o', 'LineWidth', 1.5);
xlabel('X-axis from 1-10');
ylabel('Random Number Value');
title('Plot of 10 Random Numbers');
grid on;