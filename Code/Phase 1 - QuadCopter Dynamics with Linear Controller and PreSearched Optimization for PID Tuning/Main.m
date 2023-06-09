clear
close all
clc
warning off

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
InitCond.Position    = [1, 0, 2.5];
InitCond.Velocity    = [0, 0, 0];
InitCond.Orientation = [0, 0, 0];
InitCond.Omega       = [0, 0, 0];


% Controller Gains and Params
% X, Y, Z respectively
Gains.Linear.Kp  = [    2,   0.8,  1.5];
Gains.Linear.Ki  = [    0,   0.1,    0];
Gains.Linear.Kd  = [    6,   7.5,    5];
Gains.Linear.Tau = [    1,     1, 0.01];

% Gains.Linear.Kp  = [    2,   0.8,  1.5];
% Gains.Linear.Ki  = [0.001, 0.001,    0];
% Gains.Linear.Kd  = [    6,   7.5,    5];
% Gains.Linear.Tau = [    1,     1, 0.01];

Gains.Linear.Sat  = [Inf, Inf, 3];

% Phi, Theta, Psi respectively
Gains.Angular.Kp  = [ 0.1, 0.05,  0.2];
Gains.Angular.Ki  = [ 0.1, 0.05,    0];
Gains.Angular.Kd  = [ 0.1,  0.1,  0.2];
Gains.Angular.Tau = [0.01, 0.01, 0.01];

% Gains.Angular.Kp  = [ 0.1, 0.05,  0.2];
% Gains.Angular.Ki  = [   0,    0,    0];
% Gains.Angular.Kd  = [ 0.1,  0.1,  0.2];
% Gains.Angular.Tau = [0.01, 0.01, 0.01];

Gains.Angular.Sat  = [Inf, Inf, Inf];

% Simulation Properties
dt = 0.01;
SimTime = 15;

%% InitCond a QuadCopter Obj
Q = QuadCopter(DynPar, Gains,InitCond, dt, SimTime, @Traj);
[Motion, InSig] = Q.Simulate();

%% Plot Results
Plt = MotionPlotter(Motion.t, Motion.Y, InSig.U, InSig.Thrusts, InSig.Err, InSig.RefSig, DynPar.ArmLength);
Plt.PlotAngularMotion()
Plt.PlotLinearMotion()
Plt.PlotTrackingErr()
Plt.PlotThrusts()
Plt.PlotControlSignals()

Plt.Plot3D();