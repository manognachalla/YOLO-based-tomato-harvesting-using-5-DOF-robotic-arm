% ik_solver.m

clc; clear;

% Define symbolic variables
syms theta1 theta2 theta3 theta4 theta5 real
syms L1 L2 L3 L4 L5 real

% Store in global variables
global theta_syms length_syms
theta_syms = [theta1; theta2; theta3; theta4; theta5];
length_syms = [L1; L2; L3; L4; L5];

% Input link lengths
L = [25.929, 81.379, 83.009, 77, 106];

% Initial joint angles guess (in radians)
theta_init = [0, 0, 0, -pi/2, pi/2];

% Desired end-effector position (x, y, z)
T_goal = [eye(3), [183.0000; 0; 190.3170]; 0 0 0 1];

% Run inverse kinematics
theta_solution = inverse_kinematics(T_goal, theta_init, L);

disp("Final joint angles (in degrees):");
disp(rad2deg(theta_solution));

disp("Final end-effector position:");
T_final = forward_kinematics(theta_solution, L);
disp(T_final(1:3,4));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FUNCTION DEFINITIONS BELOW %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function T = forward_kinematics(theta_vals, L)
    global theta_syms length_syms

    % Assign symbolic vars
    theta1 = theta_syms(1); theta2 = theta_syms(2); theta3 = theta_syms(3);
    theta4 = theta_syms(4); theta5 = theta_syms(5);
    L1 = length_syms(1); L2 = length_syms(2); L3 = length_syms(3);
    L4 = length_syms(4); L5 = length_syms(5);

    A1 = DH2A(L1, theta1, 0, pi/2); A1(1,2)=0; A1(2,2)=0; A1(3,3)=0;
    A2 = DH2A(0, theta2 + pi/2, L2, 0);
    A3 = DH2A(0, theta3, L3, 0);
    A4a = DH2A(0, theta4 + pi/2, 0, pi/2); A4a(1,2)=0; A4a(2,2)=0; A4a(3,3)=0;
    A4b = DH2A(L4, 0, 0, 0);
    A5 = DH2A(L5, theta5, 0, 0);
    A4 = A4a * A4b;

    T_sym = simplify(A1 * A2 * A3 * A4 * A5);

    vars = [theta_syms; length_syms];
    vals = [theta_vals(:); L(:)];

    T_sub = subs(T_sym, vars, vals);

    if ~isempty(symvar(T_sub))
        error("Substitution incomplete. Remaining symbols: %s", strjoin(string(symvar(T_sub)), ', '));
    end

    T = double(T_sub);
end

function J = calculate_jacobian(theta_vals, L)
    delta = 1e-5;  % small perturbation
    T0 = forward_kinematics(theta_vals, L);
    p0 = T0(1:3,4);

    J = zeros(3, length(theta_vals));

    for i = 1:length(theta_vals)
        theta_temp = theta_vals;
        theta_temp(i) = theta_temp(i) + delta;

        T_new = forward_kinematics(theta_temp, L);
        p_new = T_new(1:3, 4);

        J(:,i) = (p_new - p0) / delta;
    end
end

function theta_vals = inverse_kinematics(T_goal, theta_init, L)
    theta_vals = theta_init;
    max_iter = 100;
    tol = 1e-4;

    for i = 1:max_iter
        T_curr = forward_kinematics(theta_vals, L);
        p_curr = T_curr(1:3, 4);
        p_goal = T_goal(1:3, 4);
        error = p_goal - p_curr;

        if norm(error) < tol
            fprintf("Converged in %d iterations.\n", i);
            return;
        end

        J = calculate_jacobian(theta_vals, L);
        delta_theta = pinv(J) * error;

        theta_vals = theta_vals + delta_theta';
    end

    warning("IK did not converge within max iterations.");
end

function A = DH2A(b, theta, a, alpha)
    ps = [0 0 0 1];
    Tb = [eye(3) [0;0;b]; ps];
    Ttht = [Rz(theta) [0;0;0]; ps];
    Ta = [eye(3) [a;0;0]; ps];
    Talp = [Rx(alpha) [0;0;0]; ps];
    A = Tb*Ttht*Ta*Talp;
end

function R = Rx(phi)
    R = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
end

function R = Rz(phi)
    R = [cos(phi) -sin(phi) 0; sin(phi) cos(phi) 0; 0 0 1];
end
