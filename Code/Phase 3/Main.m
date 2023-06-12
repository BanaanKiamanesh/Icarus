clear
close all
clc

format short
format compact
rng(123, 'twister')

%% Parameter Declaration

% Dynamic Parameters
DynPar.g = 9.81;
DynPar.m = 0.650;
DynPar.l = 0.23;
DynPar.Ir = 6e-5;
DynPar.Ix = 7.5e-3;
DynPar.Iy = 7.5e-3;
DynPar.Iz = 1.3e-2;
DynPar.k = 3.13e-5;
DynPar.d = 7.5e-7;

% Simulation Properties
dt = 0.005;
SimTime = 40;

% Controller Parameters
CtrlGains.Zeta = [0.35804, 0.50581, 0.34049, 0.61075, 0.61075, 0.61075];
CtrlGains.Kappa = [8.0568, 13.6547, 1.8914, 1.358775, 5.2608, 5.0176];

% Disturbance Properties
DisturbProperties.Dl = -0.65;
DisturbProperties.Du = 0.65;

DisturbProperties.DF = @Disturbance;


%% Born of a QuadCopter
Q = QuadCopter(DynPar, CtrlGains, DisturbProperties, dt, SimTime);
[t, X, Uout, S, Xd, Omega, Err] = Q.Simulate();


%% Plot Results
Plt = MotionPlotter(t, X, Uout(1:4, :), Omega(1:4, :), Err([4, 5, 6, 3], :), Xd, DynPar.l, S);
Plt.PlotLinearMotion()
Plt.PlotAngularMotion()
Plt.PlotSlidingSurfaces()
Plt.PlotTrackingErr()
Plt.PlotThrusts()
Plt.PlotControlSignals()
% Plt.Plot3D()
