clear
close all
clc
warning off
rng(123, 'twister')

%% Optimization Parameter Declaration

TURNS = 3;                                                        % Number of Turns
IdealState = [0, 0, 0, 0, 0, 0, 2 * pi * TURNS, 0, 0];            % Desired State

CostFunction = @(params) CostFunction(params, IdealState);        % Cost Function
InitSolGen = @InitSolGen;                                         % Initial Solution Generation Function

% Optimizer Params
HabitatNum = 60;                        % Number of Habitats
MaxIt      = 10000;                     % Maximum Number of Iterations
VarNum     = 5;                         % Number of Variables
Range      = [-1, 1] * 3;               % Variablee Range of Change

%% BBO Optimizer Object Creation and Run
% Optimizer Init
BBOAlgo = BBO(CostFunction, InitSolGen, HabitatNum, MaxIt, VarNum, Range);
% Optimizer Solve
[Sol, BestCost] = BBOAlgo.Run();

% clearvars -except Sol
% Save the Solution to File
save OptData\OptimizationData.mat BestCost
