function [Cost, CostVec] = CostFunction(Par, IdealState)

    gen = MultiFlipParams;
    quad = Quadcopter(false);

    if Par(2) > 2.5 || Par(3) > 2.5 || Par(5) > 2.5

        CostVec = ones(1, 9) * 1e7;
        Cost = sum(CostVec);
        return

    else

        Sect = gen.GetSection(Par);

        for i = 1:size(Sect, 1)

            if cell2mat(Sect(i, 3)) < 0
                CostVec = ones(1, 9) * 1e6;
                Cost = sum(CostVec);
                return
            end
        end

        quad.Update(Sect);
        FinalState = [quad.Pos, quad.Vel, quad.Orient];

        CostVec = abs(IdealState - FinalState);
        Cost = sum(CostVec);
    end
end
