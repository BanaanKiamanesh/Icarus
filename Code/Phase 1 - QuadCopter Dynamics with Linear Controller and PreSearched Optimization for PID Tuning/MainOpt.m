clear;
close all;
clc;

%% Optimization Parameter Declaration
InitSolGen = @InitSolGen;               % Initial Solution Generation Function
Cost = @(Sol) CostFunction(Sol);        % Cost Function

% Optimizer Params
HabitatNum = 100;                       % Number of Habitats
MaxIt = 100;                            % Maximum Number of Iterations
VarNum = 24;                            % Number of Variables
Range = [-1, 1] * 20;                   % Variablee Range of Change

%% BBO Optimizer Object Creation and Run
% Optimizer Init
BBOAlgo = BBO(Cost, InitSolGen, HabitatNum, MaxIt, VarNum, Range);
% Optimizer Solve
Sol = BBOAlgo.Run();

clearvars -except Sol
% Save the Solution to File
save OptData\OptimizationSol4.mat Sol