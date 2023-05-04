clear
close all
clc
warning off

%% Load and Declare Parameters
load OptData\OptimizationSol4.mat

Sol = Sol.SIV;

% Dynamical Properties
DynPar.Mass = 0.468;
DynPar.ArmLength = 0.17;
DynPar.Ixx = 0.0023;
DynPar.Iyy = 0.0023;
DynPar.Izz = 0.0046;
DynPar.Thrust2Drag = 0.016;


% InitCondial Conditions
InitCond.Position    = [1, 0, 2.5];
InitCond.Velocity    = [0, 0, 0];
InitCond.Orientation = [0, 0, 0];
InitCond.Omega       = [0, 0, 0];

% Simulation Properties
dt = 0.01;
SimTime = 45;

Trajectory = @Traj;

% Controller Gains and Params
% X, Y, Z respectively
Gains.Linear.Kp  = [    2,   0.8,  1.5] + Sol(1 : 3);
Gains.Linear.Ki  = [    0,   0.1,    0] + Sol(4 : 6);
Gains.Linear.Kd  = [    6,   7.5,    5] + Sol(7 : 9);
Gains.Linear.Tau = [    1,     1, 0.01] + Sol(10:12);

Gains.Linear.Sat  = [Inf, Inf, Inf];

% Phi, Theta, Psi respectively
Gains.Angular.Kp  = [ 0.1, 0.05,  0.2] + Sol(13:15);
Gains.Angular.Ki  = [ 0.1, 0.05,    0] + Sol(16:18);
Gains.Angular.Kd  = [ 0.1,  0.1,  0.2] + Sol(19:21);
Gains.Angular.Tau = [0.01, 0.01, 0.01] + Sol(22:24);

Gains.Angular.Sat  = [Inf, Inf, Inf];

Quad = QuadCopter(DynPar, Gains, InitCond, dt, SimTime, Trajectory);

[Motion, InSig] = Quad.Simulate();

%% Plot Results
Plt = MotionPlotter(Motion.t, Motion.Y, InSig.U, InSig.Thrusts, InSig.Err, InSig.RefSig, Quad.ArmLength);
Plt.PlotAngularMotion()
Plt.PlotLinearMotion()
Plt.PlotTrackingErr()
Plt.PlotThrusts()
Plt.PlotControlSignals()

Plt.Plot3D();