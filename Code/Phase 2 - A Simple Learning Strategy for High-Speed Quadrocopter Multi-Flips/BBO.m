classdef BBO
    properties
        CostFunction
        InitSolGen
        MaxIt
        HabitatNum
        VarNum
        Range
        Range2
    end

    properties
        Lambda
        Mu
        Alpha
        MutationRate

        Sigma

        hbt
        Counter
        BestSol
        BestHSI
    end

    methods
        function obj = BBO(CostFunction, InitSolGen, HabitatNum, MaxIt, VarNum, Range)
            obj.CostFunction = CostFunction;
            obj.InitSolGen   = InitSolGen;
            obj.MaxIt        = MaxIt;
            obj.HabitatNum   = HabitatNum;
            obj.VarNum       = VarNum;
            obj.Range        = Range;

            obj.Range  = sort(obj.Range);
            obj.Range2 = obj.Range(2);

            obj.Sigma = 0.02 * (obj.Range(2) - obj.Range(1));

            % Migration Rates
            obj.Mu = linspace(1, 0, obj.HabitatNum);           % Emmigration Rates
            obj.Lambda = 1 - obj.Mu;                             % Immigration Rates

            obj.Alpha = 0.9;                               % Transferation Rate
            obj.MutationRate = 0.2;                       % Mutation Rate

            % BBO Init
            % Sample Habitat
            Habitat.SIV  = [];
            Habitat.HSI  = [];

            % Create Habitats Array
            obj.hbt = repmat(Habitat, obj.HabitatNum, 1);

            % Starting Parallel Pool
            parpool('local', 4);

            parfevalOnAll(@warning,0,'off','all')
            parfor i = 1:5
                warning(['i = ' num2str(i)]);
            end

            % Initialize Habitats
            tmphbt = obj.hbt;
            parfor i = 1:obj.HabitatNum
                tmphbt(i).SIV = obj.InitSolGen();
                tmphbt(i).HSI = obj.CostFunction(tmphbt(i).SIV);
            end

            obj.hbt = tmphbt;

            % Sort Population
            [~,  SortOrder] = sort([obj.hbt.HSI]);
            obj.hbt = obj.hbt(SortOrder);

            % Best Solution Ever Found
            obj.BestSol = obj.hbt(1);

            % Array to Hold Best Costs
            obj.BestHSI = repmat(Habitat, obj.MaxIt, 1);
        end

        function [Sol, BestofIt] = Run(obj)

            % BBO Main Loop
            for it = 1:obj.MaxIt
                
                newhbt = obj.hbt;
                parfor i = 1:obj.HabitatNum
                    for k = 1:obj.VarNum
                        % Migration
                        if rand <= obj.Lambda(i)
                            % Emmigration Probabilities
                            EP = obj.Mu;
                            EP(i) = 0;
                            EP = EP / sum(EP);

                            % Select Source Habitat (Roulette Wheel Selection)
                            j = find(rand <= cumsum(EP), 1, 'first');

                            % Migration
                            newhbt(i).SIV(k) = obj.hbt(i).SIV(k) + obj.Alpha*(obj.hbt(j).SIV(k) - obj.hbt(i).SIV(k));

                        end

                        % Mutation
                        if rand <= obj.MutationRate
                            newhbt(i).SIV(k) = newhbt(i).SIV(k) + obj.Sigma * randn;
                        end
                    end

                    % Evaluation
                    newhbt(i).HSI = obj.CostFunction(newhbt(i).SIV);
                end

                % Sort New Population
                [~,  SortOrder] = sort([newhbt.HSI]);
                newhbt = newhbt(SortOrder);

                % Merge New and Old Populations
                obj.hbt = [obj.hbt; newhbt];

                % Sort Population
                [~,  SortOrder] = sort([obj.hbt.HSI]);
                obj.hbt = obj.hbt(SortOrder);

                % Select Next Iteration Population(Truncate)
                obj.hbt = obj.hbt(1: obj.HabitatNum);

                % Update Best Solution Ever Found
                obj.BestSol = obj.hbt(1);

                % Store Best HSI Ever Found
                obj.BestHSI(it) = obj.BestSol;

                disp(['Iteration ', num2str(it), ': Best Cost = ', num2str(obj.BestHSI(it).HSI)]);
            end

            % Return Best Solution Ever Found
            Sol = obj.BestSol;

            BestofIt = obj.BestHSI;

            delete(gcp('nocreate'))             % ShutDown the Current Pool
        end
    end
end