clear;
close all;
clc;

%% QuadCopter Dynamical Paramters Declaration
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
SimTime = 15;

Trajectory = @Traj;

save QuadData.mat DynPar InitCond dt SimTime Trajectory

clear