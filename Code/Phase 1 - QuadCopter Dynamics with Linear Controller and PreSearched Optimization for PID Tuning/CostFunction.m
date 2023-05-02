function [J, Quad] = CostFunction(Sol, RoomDims)

    RoomDims.X = [-4, 4];
    RoomDims.Y = [-4, 4];
    RoomDims.Z = [ 0, 5];


    load("Data/QuadData.mat"); %#ok<LOAD>

    % % Dynamical Properties
    % DynPar.Mass = 0.468;
    % DynPar.ArmLength = 0.17;
    % DynPar.Ixx = 0.0023;
    % DynPar.Iyy = 0.0023;
    % DynPar.Izz = 0.0046;
    % DynPar.Thrust2Drag = 0.016;
    %
    %
    % % InitCondial Conditions
    % InitCond.Position    = [1, 0, 2.5];
    % InitCond.Velocity    = [0, 0, 0];
    % InitCond.Orientation = [0, 0, 0];
    % InitCond.Omega       = [0, 0, 0];
    %
    % % Simulation Properties
    % dt = 0.01;
    % SimTime = 30;
    %
    % Trajectory = @Traj;
    %
    % % Controller Gains and Params
    % % X, Y, Z respectively
    % Gains.Linear.Kp  = [    2,   0.8,  1.5] + Sol(1 : 3);
    % Gains.Linear.Ki  = [    0,   0.1,    0] + Sol(4 : 6);
    % Gains.Linear.Kd  = [    6,   7.5,    5] + Sol(7 : 9);
    % Gains.Linear.Tau = [    1,     1, 0.01] + Sol(10:12);
    %
    % Gains.Linear.Sat  = [Inf, Inf, Inf];
    %
    % % Phi, Theta, Psi respectively
    % Gains.Angular.Kp  = [ 0.1, 0.05,  0.2] + Sol(13:15);
    % Gains.Angular.Ki  = [ 0.1, 0.05,    0] + Sol(16:18);
    % Gains.Angular.Kd  = [ 0.1,  0.1,  0.2] + Sol(19:21);
    % Gains.Angular.Tau = [0.01, 0.01, 0.01] + Sol(22:24);
    %
    % Gains.Angular.Sat  = [Inf, Inf, Inf];

    Quad = QuadCopter(DynPar, Gains, InitCond, dt, SimTime, Trajectory);

    [Motion, InSig] = Quad.Simulate();

    % Calculate Cost
    X = Motion.Y(:, 1);
    Y = Motion.Y(:, 2);
    Z = Motion.Y(:, 3);

    if any(isnan(Motion.Y(:)), "all") || any(isinf(Motion.Y(:)), "all")
        J = Inf;
        return;
    end

    if any(isnan(InSig.U(:)), "all") || any(isinf(InSig.U(:)), "all")
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
    U = InSig.U';                 % N-by-4 Control Sig Matrix
    Err = InSig.Err';             % 4-by-N Err     Sig Matrix

    % Tracking Err Cost
    Weights = [1, 1, 1, 1];
    TE = dot(sum(Err.^2), Weights);


    % Control Effort Cost
    Weights = [1, 1, 1, 1];
    CE = dot(sum(U.^2), Weights);

    % Create Final Cost as a MultiObjective Optimization Problem with Weights
    W1 = 10;
    W2 = 0;
    J = TE + W2/W1 * CE;

    if isnan(J) || isinf(J)
        J = Inf;
        return;
    end
end