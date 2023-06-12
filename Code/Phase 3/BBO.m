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

            % Migration Rates
            obj.Mu = linspace(1, 0, obj.HabitatNum);           % Emmigration Rates
            obj.Lambda = 1 - obj.Mu;                             % Immigration Rates

            obj.Alpha = 0.9;                               % Transferation Rate
            obj.MutationRate = 0.05;                       % Mutation Rate

            % BBO Init
            % Sample Habitat
            Habitat.SIV  = [];
            Habitat.HSI  = [];
            Habitat.Quad = [];
            obj.Counter  = 0;

            % Create Habitats Array
            obj.hbt = repmat(Habitat, obj.HabitatNum, 1);

            % Starting Parallel Pool
            p = parpool('local', 4);

            parfevalOnAll(@warning,0,'off','all')
            parfor i = 1:5
                warning(['i = ' num2str(i)])
            end

            % Initialize Habitats
            tmphbt = obj.hbt;
            parfor i = 1:obj.HabitatNum
                tmphbt(i).SIV = obj.InitSolGen();
                [tmphbt(i).HSI, tmphbt(i).Quad] = obj.CostFunction(tmphbt(i).SIV);
            end

            obj.hbt = tmphbt;

            % Sort Population
            [~,  SortOrder] = sort([obj.hbt.HSI]);
            obj.hbt = obj.hbt(SortOrder);

            % Best Solution Ever Found
            obj.BestSol = obj.hbt(1);

            % Array to Hold Best Costs
            obj.BestHSI = zeros(obj.MaxIt, 1);
        end

        function Sol = Run(obj)
            % BBO Main Loop
            for it = 1:obj.MaxIt
                disp(it)

                newhbt = obj.hbt;
                parfor i = 1:obj.HabitatNum
                    for k = 1:obj.VarNum
                        % Migration
                        if rand <= obj.Lambda(i)
                            % Emmigration Probabilities
                            EP = obj.Mu;
                            EP(i) = 0;
                            EP = EP / sum(EP);

                            % Select Source Habitat
                            j = RouletteWheelSelection(EP);

                            % Migration
                            newhbt(i).SIV(k) = ceil(obj.hbt(i).SIV(k) + obj.Alpha*(obj.hbt(j).SIV(k) - obj.hbt(i).SIV(k)));

                            if newhbt(i).SIV(k) > obj.Range2
                                newhbt(i).SIV(k) = obj.Range2;
                            end
                        end

                        % Mutation
                        if rand <= obj.MutationRate
                            newhbt(i).SIV(k) = Mutate(newhbt(i).SIV(k), obj.Range);
                        end
                    end

                    % Evaluation
                    [newhbt(i).HSI, newhbt(i).Quad] = obj.CostFunction(newhbt(i).SIV);
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
                obj.BestHSI(it) = obj.BestSol.HSI;
            end

            % Return Best Solution Ever Found
            Sol = obj.BestSol;

            delete(gcp('nocreate'))             % ShutDown the Current Pool
        end
    end
end


function z = Mutate(x, Range)

    %MUTATE Function to Mutate the Population With Discrete Amounts In Given Range

    % Create a Vector For the Existence Range
    Range = (Range(1): Range(2));
    % Remove the Current Value
    Range(Range == x) = [];
    % Take a Random number in the Range
    z = randsample(Range, 1);

end

function j = RouletteWheelSelection(P)

    %ROULETTEWHEELSELECTION
    % Function for Probablistic RouletteWheel Selection
    r = rand;
    C = cumsum(P);
    j = find(r <= C, 1, 'first');

end