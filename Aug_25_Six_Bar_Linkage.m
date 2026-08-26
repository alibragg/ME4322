% six bar linkage
% Static Equilibrium

clc;
clear;

% define the joints
A = [7 4 0];
B = [5 16 0];
C = [25 25 0];
D = [23 10 0];
E = [18 35 0];
F = [43 32 0];
G = [45 17 0];

% Define the lengths of the links
lAB = norm(B - A);
lBC = norm(C - B);
lCD = norm(D - C);
lBE = norm(E - B);
lEF = norm(F - E);
lFG = norm(G - F);

% Weight of Each Link
WAB = [0 -1 0];
WBEC = [0 -1 0];
WCD = [0 -1 0];
WEF = [0 -1 0];
WFG = [0 -1 0];

% Center of mass of each link
S1 = (A+B)/2;
S2 = (B+C+E)/3; % not accurate
S3 = (C+D)/2;
S4 = (E+F)/2;
S5 = (F+G)/2;

syms FAx FAy FBx FBy FCx FCy FDx FDy FEx FEy FFx FFy FGx FGy Tin

ForceA = [FAx FAy 0];
ForceB = [FBx FBy 0];
ForceC = [FCx FCy 0];
ForceD = [FDx FDy 0];
ForceE = [FEx FEy 0];
ForceF = [FFx FFy 0];
ForceG = [FGx FGy 0];
InputTorque = [0 0 Tin];

% Applied Force
AppliedForce = [50 0 0];

% Static Equilibrium Conditions for Link AB

% Sum of forces = 0
% Fa + Fb + WeightofAB = 0
eqn1 = ForceA + ForceB + WAB == 0;

% Sum of moments = 0 with respect to Center of Mass of Link AB
% S1A x FA (force vector A) + S1B x FB + InputTorque = 0

eqn2 = cross(A-S1, ForceA) + cross(B-S1, ForceB) + InputTorque == 0;

% Equations for Link BEC
% Sum of Forces = 0
% -Fb + Fc + Fe + WBEC = 0

eqn3 = -ForceB + ForceC + ForceE + WBEC == 0;

% Sum of Moments = 0
% Sum of Moments = 0 with respect to Center of Mass of Link BEC
% S2B x -FB + S2C x FC + S2E x FE = 0
eqn4 = cross(S2-B, -ForceB) + cross(S2-C, ForceC) + cross(S2-E, ForceE) == 0;

% Equations for Link CD
% Sum of Forces = 0
% -Fc + Fd + WCD = 0
eqn5 = -ForceC + ForceD + WCD == 0;

% Sum of Moments = 0
% Sum of Moments = 0 with respect to Center of Mass of Link CD
% S3C x -FC + S3D x FD = 0
eqn6 = cross(C-S3, -ForceC) + cross(D-S3, ForceD) == 0;

% Equations for Link EF
% Sum of Forces = 0
% -Fe + Ff + WEF = 0
eqn7 = -ForceE + ForceF + WEF == 0;

% Sum of Moments = 0
% Sum of Moments = 0 with respect to Center of Mass of Link EF
% S4E x -FE + S4F x FF = 0
eqn8 = cross(E-S4, -ForceE) + cross(F-S4, ForceF) == 0;

% Equations for Link FG
% Sum of Forces = 0
eqn9 = -ForceF + ForceG + WFG  + AppliedForce == 0;

% Sum of Moments = 0
% Sum of Moments = 0 with respect to Center of Mass of Link FG
% S5F x -FF + S5G x FG = 0
eqn10 = cross(F-S5, -ForceF) + cross(G-S5, ForceG) == 0;

% Solving the 10 equations

eqnMatrix = [eqn1, eqn2, eqn3, eqn4, eqn5, eqn6, eqn7, eqn8, eqn9, eqn10];

StaticSolution=solve(eqnMatrix, [FAx FAy FBx FBy FCx FCy FDx FDy FEx FEy FFx FFy FGx FGy Tin]);

Force_Ax = double(StaticSolution.FAx);
Force_Ay = double(StaticSolution.FAy);
Force_Bx = double(StaticSolution.FBx);
Force_By = double(StaticSolution.FBy);
Force_Cx = double(StaticSolution.FCx);
Force_Cy = double(StaticSolution.FCy);
Force_Dx = double(StaticSolution.FDx);
Force_Dy = double(StaticSolution.FDy);
Force_Ex = double(StaticSolution.FEx);
Force_Ey = double(StaticSolution.FEy);
Force_Fx = double(StaticSolution.FFx);
Force_Fy = double(StaticSolution.FFy);
Force_Gx = double(StaticSolution.FGx);
Force_Gy = double(StaticSolution.FGy);
Input_Torque = double(StaticSolution.Tin);

% Display the results for all forces and input torque
disp('Force A:');
disp([Force_Ax, Force_Ay]);
disp('Force B:');
disp([Force_Bx, Force_By]);
disp('Force C:');
disp([Force_Cx, Force_Cy]);
disp('Force D:');
disp([Force_Dx, Force_Dy]);
disp('Force E:');
disp([Force_Ex, Force_Ey]);
disp('Force F:');
disp([Force_Fx, Force_Fy]);
disp('Force G:');
disp([Force_Gx, Force_Gy]);
disp('Input Torque:');
disp(Input_Torque);