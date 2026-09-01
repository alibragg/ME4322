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

% joint values
% store the positions for plotting 
new_B_x(1) = B(1);
new_B_y(1) = B(2);
new_C_x(1) = C(1);
new_C_y(1) = C(2);
new_E_x(1) = E(1);
new_E_y(1) = E(2);
new_F_x(1) = F(1);
new_F_y(1) = F(2);

% Define the lengths of the links
lAB = norm(B - A);
lBC = norm(C - B);
lCD = norm(D - C);
lBE = norm(E - B);
lEF = norm(F - E);
lFG = norm(G - F);
lCE = norm(E - C);

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

% Loop ABCDA

syms wBEC wCD
omega_AB = [0 0 1];
omega_BEC = [0 0 wBEC];
omega_CD = [0 0 wCD];

eqn11 = cross(omega_AB, B-A) + cross(omega_BEC, C-B) + cross(omega_CD, D-C) == 0;

loop1Solution = solve(eqn11, [wBEC wCD]);

% Extract angular velocities from the loop solution
angularVelocity_BEC = double(loop1Solution.wBEC);
angularVelocity_CD = double(loop1Solution.wCD);

disp('angularVelocity_BEC')
disp(angularVelocity_BEC)
disp('angularVelocity_CD')
disp(angularVelocity_CD)

omegaBEC = [0 0 angularVelocity_BEC];
omegaCD = [0 0 angularVelocity_CD];

% Second Loop
% DCEFGD

syms wEF wFG
omega_EF = [0 0 wEF];
omega_FG = [0 0 wFG];

eqn12 = cross(omegaCD, C-D) + cross(omegaBEC, E-C) + cross(omega_EF, F-E) + cross(omega_FG, G-F) == 0;

% Solve the second loop equations
loop2Solution = solve(eqn12, [wEF wFG]);

% Extract angular velocities from the second loop solution
angularVelocity_EF = double(loop2Solution.wEF);
angularVelocity_FG = double(loop2Solution.wFG);

% Angular Acceleration
% Loop 1 ABCDA
% Angular Acceleration for Loop 1
syms aBEC aCD
alpha_AB = [0 0 0]; % Assuming no angular acceleration for AB
alpha_BEC = [0 0 aBEC];
alpha_CD = [0 0 aCD];

a_B_A = cross(alpha_AB, B-A) + cross(omega_AB, cross(omega_AB, B-A)); % accel of B w.r.t A
a_C_B = cross(alpha_BEC, C-B) + cross(omegaBEC, cross(omegaBEC, C-B));
a_D_C = cross(alpha_CD, D-C) + cross(omegaCD, cross(omegaCD, D-C));

eqn13 = a_B_A + a_C_B + a_D_C == 0;

loop1AccSolution = solve(eqn13, [aBEC aCD]);

alphaBEC = double(loop1AccSolution.aBEC);
alphaCD = double(loop1AccSolution.aCD);

alphaBEC_vector = [0 0 alphaBEC];
alphaCD_vector = [0 0 alphaCD];

disp('alphaBEC')
disp(alphaBEC)
disp('alphaCD')
disp(alphaCD)

% Loop 2 DCEFGD

syms aEF aFG
alpha_EF = [0 0 aEF];
alpha_FG = [0 0 aFG];

% a_C_D + a_E_C + a_F_E + a_G_F = 0
a_C_D = cross(alphaCD_vector, C-D) + cross(omegaCD, cross(omegaCD, C-D));
a_E_C = cross(alphaBEC_vector, E-C) + cross(omegaBEC, cross(omegaBEC, E-C));

angVel_EF = [0 0 angularVelocity_EF];
angVel_FG = [0 0 angularVelocity_FG];

a_F_E = cross(alpha_EF, F-E) + cross(angVel_EF, cross(angVel_EF, F-E));
a_G_F = cross(alpha_FG, G-F) + cross(angVel_FG, cross(angVel_FG, G-F));

eqn14 = a_C_D + a_E_C + a_F_E + a_G_F == 0;

loop2AccSolution = solve(eqn14, [aEF aFG]);

% Extract angular accelerations from the loop solution
alphaEF = double(loop2AccSolution.aEF)
alphaFG = double(loop2AccSolution.aFG)

disp('alphaEF')
disp(alphaEF)
disp('alphaFG')
disp(alphaFG)

% Extract angular accelerations for the second loop
alphaEF = double(loop2AccSolution.aEF);
alphaFG = double(loop2AccSolution.aFG);


% Velocity at Joint

vB_A = cross(omega_AB, B-A);

% VE_A = V_E_B + V_B_A;

v_E_B = cross(omegaBEC, E-B);

vE_A = v_E_B + vB_A;

% V_S4/G = V_S4_F + V_F_G (center of mass)
V_S4_F = cross(angVel_EF, S4-F);

V_F_G = cross(angVel_FG, F-G);

vS4_G = V_S4_F + V_F_G;



% Class Participation:

% % Velocity at Joint E
% vE_F = cross(omega_EF, F-E);
% 
% 
% % Velocity at Joint A
% vA_B = cross(omega_AB, A-B);
% vG_A = vE_A + vA_B; % Velocity of G with respect to A
% disp('Velocity at Joint A:');
% disp(vG_A);
% 
% 
% % Velocity at Joint F
% vF_G = cross(omega_FG, G-F);
% vF_E = vE_F + vF_G; % Velocity of F with respect to E
% 
% % Center of mass velocity at Joint F
% disp('Velocity at Joint F:');
% disp(vF_E);
% 
% % Velocity at Joint G
% vG_A = vF_E + vF_G; % Velocity of G with respect to A
% 
% % Center of mass velocity at Joint G
% vG_E = vF_E + vF_G; % Velocity of G with respect to E
% disp('Velocity at Joint G:');
% disp(vG_E);


% Accelerations at com of every link

aS1_A = cross(alpha_AB, S1-A) + cross(omega_AB, cross(omega_AB, S1-A));

aS2_A = cross(alphaBEC_vector, S2-B) + cross(omegaBEC, cross(omegaBEC, S2-B));

aS3_D = cross(alphaCD_vector, S3-D) + cross(omegaCD, cross(omegaCD, S3-D));

aS4_G = cross([0 0 alphaEF], S4-F) + cross(omega_EF, cross(angVel_EF, S4-F));

aS5_G = cross([0 0 alphaFG], S5-G) + cross(angVel_FG, cross(angVel_FG, S5-G));

% Newton's Second Law Implementation

MassAB = 1;
MassBEC = 1;
MassCD = 1;
MassEF = 1;
MassFG = 1;

% Mass Moment of Inertia (w.r.t. center of mass)
J_AB = 1;
J_BEC = 1;
J_CD = 1;
J_EF = 1;
J_FG = 1;

syms NFAx NFAy NFBx NFBy NFCx NFCy NFDx NFDy NFEx NFEy NFFx NFFy NFGx NFGy NTin

% Define Forces

NForceA = [NFAx NFAy 0];
NForceB = [NFBx, NFBy, 0];
NForceC = [NFCx, NFCy, 0];
NForceD = [NFDx, NFDy, 0];
NForceE = [NFEx, NFEy, 0];
NForceF = [NFFx, NFFy, 0];
NForceG = [NFGx, NFGy, 0];
NInputTorque = [0, 0, NTin];

% Equations for Link AB
% sum of forces
eqn15 = NForceA + NForceB + WAB == MassAB * aS1_A;

% Sum of moments = 0
eqn16 = cross(A-S1, NForceA) + cross(B-S1, NForceB) + NInputTorque == J_AB * alpha_AB;

% Equations for Link BEC
% Sum of Forces
eqn17 = -NForceB + NForceC + NForceE + WBEC == MassBEC * aS2_A;

% sum of moments 
eqn18 = cross(B-S2, -ForceB) + cross(C-S2, NForceC) + cross(E-S2, NForceE) == J_BEC * alphaBEC_vector;

% Equations for Link CD
% sum of Forces
eqn19 = -NForceC + NForceD + WCD == MassCD * aS3_D;

% sum of moments
eqn20 = cross(C-S3, -NForceC) + cross(D-S3, NForceD) == J_CD * alphaCD_vector;

% Equations for Link EF
% Sum of Forces
eqn21 = -NForceE + NForceF + WEF == MassEF * aS4_G;

% sum of moments
eqn22 = cross(E-S4, -NForceE) + cross(F-S4, NForceF) == J_EF * [0 0 alphaEF];

% Equations for Link FG
% sum of forces
eqn23 = -NForceF + NForceG + WFG + AppliedForce == MassFG * aS5_G;

% sum of moments
eqn24 = cross(F-S5, -NForceF) + cross(G-S5, NForceG) == J_FG * [0 0 alphaFG];

% solving equations
NeqnMatrix = [eqn15, eqn16, eqn17, eqn18, eqn19, eqn20, eqn21, eqn22, eqn23, eqn24];
DynamicSolution = solve(NeqnMatrix, [NFAx, NFAy, NFBx, NFBy, NFCx, NFCy, NFDx, NFDy, NFEx, NFEy, NFFx, NFFy, NFGx, NFGy, NTin]);

% Extract forces from the dynamic solution
NForce_Ax = double(DynamicSolution.NFAx);
NForce_Ay = double(DynamicSolution.NFAy);
NForce_Bx = double(DynamicSolution.NFBx);
NForce_By = double(DynamicSolution.NFBy);
NForce_Cx = double(DynamicSolution.NFCx);
NForce_Cy = double(DynamicSolution.NFCy);
NForce_Dx = double(DynamicSolution.NFDx);
NForce_Dy = double(DynamicSolution.NFDy);
NForce_Ex = double(DynamicSolution.NFEx);
NForce_Ey = double(DynamicSolution.NFEy);
NForce_Fx = double(DynamicSolution.NFFx);
NForce_Fy = double(DynamicSolution.NFFy);
NForce_Gx = double(DynamicSolution.NFGx);
NForce_Gy = double(DynamicSolution.NFGy);
NTorque_in = double(DynamicSolution.NTin);



% Circle Intersection Technique (8/28 Class)

% joint coordinates have been defined
% length of links also defined 

% compute initial angle of input link AB

initial_theta = atan2(B(2)-A(2), B(1)-A(1));

if (initial_theta < 0)
    inputAngle = 2*pi + initial_theta; % in radians
else
    inputAngle = initial_theta;
end

for theta = 1:1:360 % increment the input angle by 1 degree until 360 degrees
    % new position of joint B

    B_new = A + [lAB*cos(inputAngle+deg2rad(theta)) lAB*sin(inputAngle+deg2rad(theta)) 0];

    % new position of joint C
    % with B_new as center, BC as radius
    % with D as center and DC as radius

    [Cx, Cy] = circcirc(B+new(1), B_new(2), lBC, D(1), D(2), lCD);

    % Monday 8/31 Class

    % check if there is a NaN (not a number)

    circIntersect_x = any(isnan(vpa(Cx)));
    circIntersect_y = any(isnan(vpa(Cy)));

    if circIntersect_x == 0 && circIntersect_y == 0
        C_1 = [Cx(1) Cy(1) 0];
        C_2 = [Cx(2) Cy(2) 0 ];

        % distance to determine whether C_1 or C_2 is correct

        dist1 = norm(C_1-C);
        dist2 = norm(C_2-C);

        if(dist1 < dist2)
            C_new = vpa(C_1);
        else
            C_new = vpa(C_2);
        end

        % new position of Joint E using B_new and C_new

        [Ex, Ey] = circcirc(B_new(1), B_new(2), lBE, C_new(1), C_new(2), lCE);

        % check if there is NaN

        circIntersect_x_E = any(isnan(vpa(Ex)));
        circIntersect_y_E = any(isnan(vpa(Ey)));

        if circIntersect_x==0 && circIntersect_y==0
            E_1 = [Ex(1) Ey(1) 0];
            E_2 = [Ex(2) Ey(2) 0];

            % distance to determine whether E_1 or E_2 is correct
            dist1 = norm(E_1-E);
            dist2 = norm(E_2-E);

            if(dist1<dist2)
                E_new = vpa(E_1);
            else
                E_new = vpa(E_2);
            end
        end

        % New position of Joing F using E_new and G
        [Fx, Fy] = circcirc(E_new(1), E_new(2), lEF, G(1), G(2), lFG);

        % checking if NaN
        circIntersect_x_F = any(isnan(vpa(Fx)));
        circIntersect_y_F = any(isnan(vpa(Fy)));

        if circIntersect_x_F == 0 && circIntersect_y_F==0
            F_1 = [Fx(1) Fy(2) 0];
            F_2 = [Fx(2) Fy(2) 0];

            % distance to determine whether F_1 or F_2 is correct
            distF1 = norm(F_1 - F);
            distF2 = norm(F_2 - F);

            if(distF1 < distF2)
                F_new = vpa(F_1);
            else
                F_new = vpa(F_2);
            end
        

        % Store values for plotting

        new_B_x(theta+1) = B_new(1);
        new_B_y(theta+1) = B_new(2);
        new_C_x(theta+1) = C_new(1);
        new_C_y(theta+1) = C_new(2);
        new_E_x(theta+1) = E_new(1);
        new_E_y(theta+1) = E_new(2);
        new_F_x(theta+1) = F_new(1);
        new_F_y(theta+1) = F_new(2);

        B=B_new;
        C=C_new;
        E=E_new;
        F=F_new;

        % static equilibrium code
             % add center of mass from above all the way to all forces and input torque
             % Force_Ax(theta+1) ...
             % save all results into a single matrix. do a plot

        % velocity and acceleration

        % newton's second law

        else
            fprintf('New position of F cannot be determined at angle: %d degree', theta);
        end

    else
        fprintf('New position of E cannot be determined at angle" %d degree', theta);
    end

    else
        fprintf('New position of C cannot be determined at angle: %d degree', theta);
    end

end