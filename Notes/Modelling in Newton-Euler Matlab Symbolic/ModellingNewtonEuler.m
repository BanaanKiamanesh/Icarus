clear
close all
clc

%% Code

syms m d g              % Mass, Arm Length, Gravitational Const
syms Ixx Iyy Izz        % Inertia Matrix Diag

syms   X   Y   Z real   % Translational Motion Syms
syms   u   v   w real

syms Phi Theta Psi real   % Rotational Motion Syms
syms   p     q   r real

% Input Forces
syms M [3 1] real         % Moments
syms T       real         % Sigma Thrust
U = [T
    M];

%% Create Properties



% Velocity
Vel = [u; v; w];

% Acceleration
RM = RotationMatrix([Phi, Theta, Psi]);
Acc = [0; 0; -g] + RM * [0; 0; T/m];

% Angular Velocity
Omega = [p; q; r];
AngVel = [1,         0, -sin(Theta)
          0,  cos(Phi), cos(Theta) * sin(Phi)
          0, -sin(Phi), cos(Theta) * cos(Phi)] \ Omega;

% Angular Acceleration
AngAcc = I \ (M - cross(Omega, I * Omega));

%% Create State Equations
Xdot = [Vel
        Acc
        AngVel
        AngAcc];

Xdot = simplify(Xdot);
% pretty(Xdot);

%% Linearization Over Equlibrium Point

% State Variables
x =  [X, Y, Z, u, v, w, Phi, Theta, Psi, p, q, r]';

% Create the Equilibrium Point
syms PsiE real
Xe = [0, 0, 0, 0, 0, 0, 0, 0, PsiE, 0, 0, 0];
Ue = [m*g, 0, 0, 0];

% Create A Matrix
A = subs(jacobian(Xdot, x), [x; U]', [Xe, Ue]);

% Create B Matrix
B = subs(jacobian(Xdot, U), [x; U]', [Xe, Ue]);

% Create C Matrix
C = ones(size(x))';

%% Model Dynamical Parameter Declaration
m = 0.468;
l = 0.17;
Ixx = 0.0023;
Iyy = 0.0023;
Izz = 0.0046;
k = 0.016;
g = 9.81;
dt = 0.01;

PsiE = 0;

A = double(vpa(subs(A), 3));
B = double(vpa(subs(B), 3));
C = C;
D = zeros(1, 4);

%% Create Tranfer Functions and Tune PID Controller
Tf.num = [];
Tf.den = [];
Tf.Transfunc = [];
Tf.PID.Kp = [];
Tf.PID.Ki = [];
Tf.PID.Kd = [];
Tf.dt = dt;

Tf = repmat(Tf, size(U));

for i = 1:size(U)
    % Get the Transfer Function for ith Input
    [Tf(i).num, Tf(i).den] = ss2tf(A, B, C, D, i);
    Tf(i).Transfunc = tf(Tf(i).num, Tf(i).den, Tf(i).dt);

    % Design a PID Controller
    tmpPID = pidtune(Tf(i).Transfunc, 'PID');
    Tf(i).PID.Kp = tmpPID.Kp;
    Tf(i).PID.Ki = tmpPID.Ki;
    Tf(i).PID.Kd = tmpPID.Kd;
    
    % Disp Properties
    disp(['PID Controller for Tf(', num2str(i), ') Gains = '])
    disp(Tf(i).PID)
end
