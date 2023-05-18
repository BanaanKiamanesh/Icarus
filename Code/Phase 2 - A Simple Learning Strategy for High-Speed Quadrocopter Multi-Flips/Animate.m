clear
close all
clc
warning off
format compact
format short
rng(123, "twister")

%% Code

load OptData/OptimizationData.mat

iter = [20, 50, 100, 1000, 2000, 10000];

for i = iter
    Par = BestCost(i).Position;

    gen = MultiFlipParams;
    quad = Quadcopter(true);

    Sect = gen.GetSection(Par);

    [State, Time] = quad.Update(Sect);
    % State(:, 3) = State(:, 3) + 1;

    Plotter = MotionPlotter(Time, State, 0.17);
    Plotter.Plot3D(i);

    pause(1)
    if i ~= iter(end)
        close all
    end
    pause(2)
end

%% Motion Plots of Optimal Solutions
Plotter.PlotLinearMotion();
Plotter.PlotAngularMotion();
