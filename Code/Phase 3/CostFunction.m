function [J, Q] = CostFunction(Sol)

    RoomDims.X = [-4, 4];
    RoomDims.Y = [-4, 4];
    RoomDims.Z = [ 0, 5];

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
    CtrlGains.Zeta = [0.35804, 0.50581, 0.34049, 0.61075, 0.61075, 0.61075] + Sol(1:6);
    CtrlGains.Kappa = [8.0568, 13.6547, 1.8914, 1.358775, 5.2608, 5.0176] + Sol(1:6);

    % Disturbance Properties
    DisturbProperties.Dl = -0.65;
    DisturbProperties.Du = 0.65;

    DisturbProperties.DF = @Disturbance;


    % Born of a QuadCopter
    Q = QuadCopter(DynPar, CtrlGains, DisturbProperties, dt, SimTime);
    [~, Motion, Uout, ~, ~, ~, Err] = Q.Simulate();

    % Calculate Cost
    X = Motion(7, :);
    Y = Motion(9, :);
    Z = Motion(11, :);

    if any(isnan(X(:)), "all") || any(isinf(X(:)), "all")
        J = Inf;
        return;
    end

    if any(isnan(Uout(:)), "all") || any(isinf(Uout(:)), "all")
        J = Inf;
        return;
    end

    if any(X >= RoomDims.X(2)) || any(X <= RoomDims.X(1))
        J = Inf;
        return;
    end

    if any(Y >= RoomDims.Y(2)) || any(Y <= RoomDims.Y(1))
        J = Inf;
        return;
    end

    if any(Z >= RoomDims.Z(2)) || any(Z <= RoomDims.Z(1))
        J = Inf;
        return;
    end

    % Control and Err Signal Extraction
    U = Uout(1:4, :);                 % 4-by-N Control Sig Matrix

    % Tracking Err Cost
    Weights = [1, 1, 1, 1, 1, 1];
    TE = dot(sum(abs(Err).^2, 2), Weights);


    % Control Effort Cost
    Weights = [1, 1, 1, 1];
    CE = dot(sum(abs(U).^2, 2), Weights);

    % Create Final Cost as a MultiObjective Optimization Problem with Weights
    W1 = 10;
    W2 = 0;
    J = TE + W2/W1 * CE;

    if isnan(J) || isinf(J)
        J = Inf;
        return;
    end
end