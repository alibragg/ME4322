% Alexis Bragg
% ME 4322
% Homework 1

clc;
clear;

% define the joints
A = [1.4 0.485 0];
B = [1.67 0.99 0];
C = [0.255 1.035 0];
D = [0.285 0.055 0];
E = [0.195 2.54 0];
F = [-0.98 2.57 0];
G = [0.05 0.2 0];

% Define the lengths of the links
lAB = 0.5726;
lBC = 1.4157;
lDE = 1.4866;
lEF = 1.1754;
lGF = 2.5841;

% Weight of Each Link
% WAB = ...
% WBC = 
% WDE = 
% WEF = 
% WGF = 

% Center of mass of each link
S1 = (A+B)/2;
S2 = (B+C)/2; % not accurate
S3 = (D+E)/2;
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
AppliedForce = [0 200 0];


% Static Equilibrium Conditions for Link AB
% Sum of forces = 0
% Sum of moments = 0 with respect to Center of Mass of Link AB
% S1A x FA (force vector A) + S1B x FB + InputTorque = 0


% Equations for Link BC