clear
close all
clc

format short
format compact
rng(123, "twister")

%% Param Declaration

% Dynamical Properties
DynPar.Mass = 0.468;
DynPar.ArmLength = 0.17;
DynPar.Ixx = 0.0023;
DynPar.Iyy = 0.0023;
DynPar.Izz = 0.0046;
DynPar.Thrust2Drag = 0.016;


% InitCondial Conditions
InitCond.Position    = [0, 0, 2];
InitCond.Velocity    = [0, 0, 0];
InitCond.Orientation = [0, 0, 0];
InitCond.Omega       = [0, 0, 0];


% Controller Gains and Params
% X, Y, Z respectively
Gains.Linear.Kp = [0, 0, 10];
Gains.Linear.Ki = [0, 0, 0];
Gains.Linear.Kd = [0, 0, 0];

% Phi, Theta, Psi respectively
Gains.Angular.Kp = [0.1, 0.1, 0.2];
Gains.Angular.Ki = [0, 0, 0];
Gains.Angular.Kd = [0.06, 0.11, 0.2];

% Simulation Properties
dt = 0.01;
SimTime = 5;

%% InitCond a QuadCopter Obj
Q = QuadCopter(DynPar, Gains,InitCond, dt, SimTime);
Motion = Q.Simulate();

%% Plot Results
Plt = MotionPlotter(Motion.t, Motion.Y, DynPar.ArmLength);
Plt.PlotLinearMotion()
Plt.PlotAngularMotion()
Plt.Plot3D();
