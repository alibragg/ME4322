% Obtain two numbers from the user. Write a function that get two 
% variables and if both are odd, add them together, if both are even 
% subtract the smaller from the larger, if one is odd and the other even, 
% multiply them together. Display the result in the command window.

valOne = input('Input the first value: '); 
valTwo = input('Input the second value: '); 

% if both odd, add them
if mod(valOne, 2) ~= 0 && mod(valTwo, 2) ~= 0
    result = valOne + valTwo;
    % display results
    fprintf('%d + %d = %d\n', valOne, valTwo, result)
% if both even
elseif mod(valOne, 2) == 0 && mod(valTwo, 2) == 0
% if valOne < valTwo then valTwo - valOne, else valOne - valTwo
    if valOne < valTwo
        result = valTwo - valOne;
        % display results
        fprintf('%d - %d = %d\n', valTwo, valOne, result)
    else
        result = valOne - valTwo;
        % display results
        fprintf('%d - %d = %d\n', valOne, valTwo, result)
    end
% if else
else
% multiply valOne * valTwo
    result = valOne * valTwo;
    % display results
    fprintf('%d * %d = %d\n', valOne, valTwo, result)
end