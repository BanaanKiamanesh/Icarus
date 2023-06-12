classdef QuadCopter < handle

    % Dynamical Properties
    properties (GetAccess = private)
        g
        m
        l
        Ir
        Ix
        Iy
        Iz
        k
        d

        % Dynamical Simplification Parameters
        a
        b
    end

    properties (GetAccess = private)
        Kappa
        Zeta

        % Disturbance Properties
        DF                      % Disturbance Function
        Dl                      % Disturbance Lower Bound
        Du                      % Disturbance Upper Bound
        D1
        D2
    end

    % Simulation Properties
    properties (GetAccess = private)
        dt
        SimTime
    end

    methods
        function obj = QuadCopter(DynPar, CtrlGains, DisturbProperties, dt, SimTime)

            % Dynamical Properties
            obj.m = DynPar.m;
            obj.g = DynPar.g;
            obj.l = DynPar.l;
            obj.Ir = DynPar.Ir;
            obj.Ix = DynPar.Ix;
            obj.Iy = DynPar.Iy;
            obj.Iz = DynPar.Iz;
            obj.k = DynPar.k;
            obj.d = DynPar.d;

            % Dynamic Simplification Parameters
            obj.a(1) = (obj.Iy - obj.Iz) / obj.Ix;
            obj.a(2) = -obj.Ir / obj.Ix;
            obj.a(3) = (obj.Iz - obj.Ix) / obj.Iy;
            obj.a(4) = -obj.Ir / obj.Iy;
            obj.a(5) = (obj.Ix - obj.Iy) / obj.Iz;
            obj.b(1) = obj.l / obj.Ix;
            obj.b(2) = obj.l / obj.Iy;
            obj.b(3) = obj.l / obj.Iz;


            % Simulation Properties
            obj.dt = dt;
            obj.SimTime = SimTime;

            % Controller Parameters
            obj.Kappa = CtrlGains.Kappa;
            obj.Zeta  = CtrlGains.Zeta;

            obj.Dl = DisturbProperties.Dl;
            obj.Du = DisturbProperties.Du;
            obj.DF = DisturbProperties.DF;
        end

        function [t, X, Uout, S, Xd, Omega, Err] = Simulate(obj)

            t = 0:obj.dt:obj.SimTime;           % Time Vector
            StepNum = length(t);                % Number of Steps to Run the Simulation Loop

            % Memory Allocation
            X = zeros(12, StepNum);             % State Transition

            U = zeros(4, StepNum);              % Control Inputs
            Ux = zeros(1, StepNum);             % Virtual Input for X
            Uy = zeros(1, StepNum);             % Virtual Input for Y

            X(11, 1) = 3;                       % Initial Height
            X(9,  1) = 1;
            X(7,  1) = 0;
            
            U(:, 1) = 4*ones(4, 1);             % Initial Inputs

            % Sliding Surfaces
            SPhi   = zeros(1, StepNum);
            STheta = zeros(1, StepNum);
            SPsi   = zeros(1, StepNum);
            SZ     = zeros(1, StepNum);
            SY     = zeros(1, StepNum);
            SX     = zeros(1, StepNum);

            Omega  = zeros(5, StepNum);         % Motor Angular Vel

            % Desired Virtual Phi and Theta Angles
            Phid = zeros(1, StepNum);
            Thetad = zeros(1, StepNum);

            % Trajectory
            Xd = zeros(4, StepNum);

            % Error Signals
            Err = zeros(6, StepNum);

            % Disturbance Terms
            obj.D1 = (obj.Du - obj.Dl) / 2;
            obj.D2 = (obj.Du + obj.Dl) / 2;


            for i = 1:StepNum - 1

                % Trajectory Gen
                [Xd(:, i), dXd, ddXd] = Trajectory(t(i));

                % Disturbance Term
                Disturb = Disturbance(t(i));

                Omega(1, i) = sqrt(U(1, i) / (4*obj.k) - (U(3, i) / (2*obj.k*obj.l)) - (U(4, i) / (4*obj.d)));
                Omega(2, i) = sqrt(U(1, i) / (4*obj.k) - (U(2, i) / (2*obj.k*obj.l)) + (U(4, i) / (4*obj.d)));
                Omega(3, i) = sqrt(U(1, i) / (4*obj.k) - (U(3, i) / (2*obj.k*obj.l)) - (U(4, i) / (4*obj.d)));
                Omega(4, i) = sqrt(U(1, i) / (4*obj.k) + (U(2, i) / (2*obj.k*obj.l)) + (U(4, i) / (4*obj.d)));
                Omega(5, i) = -Omega(1, i) + Omega(2, i) - Omega(3, i) + Omega(4, i);

                Phid(i)   = -asin(Uy(i));
                Thetad(i) = asin(Ux(i) / cos(Phid(i)));

                Err(1, i) = (Phid(i) - X(1, i));
                Err(2, i) = (Thetad(i) - X(3, i));
                Err(3, i) = (Xd(4, i) - X(5, i));
                Err(4, i) = (Xd(1, i) - X(7, i));
                Err(5, i) = (Xd(2, i) - X(9, i));
                Err(6, i) = (Xd(3, i) - X(11, i));

                SPhi(i + 1)   =         -X(2, i)  + obj.Zeta(1) * Err(1, i);
                STheta(i + 1) =         -X(4, i)  + obj.Zeta(2) * Err(2, i);
                SPsi(i + 1)   = dXd(4) - X(6, i)  + obj.Zeta(3) * Err(3, i);
                SX(i + 1)     = dXd(1) - X(8, i)  + obj.Zeta(4) * Err(4, i);
                SY(i + 1)     = dXd(2) - X(10, i) + obj.Zeta(5) * Err(5, i);
                SZ(i + 1)     = dXd(3) - X(12, i) + obj.Zeta(6) * Err(6, i);

                U(1, i + 1) = obj.m / (cos(X(1, i)) * cos(X(3, i))) * (obj.Kappa(4)*signum(SZ(i)) + obj.g + obj.Zeta(4)*(dXd(3) - X(12, i)) + ddXd(3) + (obj.D2 - obj.D1 * signum(SZ(i))));
                U(2, i + 1) = (1/obj.b(1)) * (obj.Kappa(1) * signum(SPhi(i)) - obj.a(1)*X(4, i)*X(6, i) - X(4, i)*obj.a(2)*Omega(5, i) - obj.Zeta(1)*(X(2, i)) + (obj.D2 - obj.D1 * signum(SPhi(i))));
                U(3, i + 1) = (1/obj.b(2)) * (obj.Kappa(2) * signum(STheta(i))  - obj.a(3)*X(2, i)*X(6, i) - X(2, i)*obj.a(4)*Omega(5, i) - obj.Zeta(2)*(X(4, i)) + (obj.D2 - obj.D1 * signum(STheta(i))));
                U(4, i + 1) = (1/obj.b(3)) * (obj.Kappa(3) * signum(SPsi(i))    - obj.a(5)*X(4, i)*X(2, i) + obj.Zeta(3)*(dXd(4) - X(6, i)) + ddXd(4) + (obj.D2 - obj.D1 * signum(SPsi(i))));
                Ux(i + 1)   = (obj.m / U(1, i)) * (obj.Kappa(5)*signum(SX(i)) + obj.Zeta(5)*(dXd(1) - X(8, i)) + ddXd(1) + (obj.D2 - obj.D1 * signum(SX(i))));
                Uy(i + 1)   = (obj.m / U(1, i)) * (obj.Kappa(6)*signum(SY(i)) + obj.Zeta(6)*(dXd(2) - X(10, i))+ ddXd(2) + (obj.D2 - obj.D1 * signum(SY(i))));

                X(1, i + 1)  = X(1, i)  + obj.dt * (X(2, i));
                X(2, i + 1)  = X(2, i)  + obj.dt * (X(4, i)*X(6, i)*obj.a(1) + X(4, i)*obj.a(2)*Omega(5, i) + obj.b(1)*U(2, i) + Disturb);
                X(3, i + 1)  = X(3, i)  + obj.dt * (X(4, i));
                X(4, i + 1)  = X(4, i)  + obj.dt * (X(2, i)*X(6, i)*obj.a(3) + X(2, i)*obj.a(4)*Omega(5, i) + obj.b(2)*U(3, i) + Disturb);
                X(5, i + 1)  = X(5, i)  + obj.dt * (X(6, i));
                X(6, i + 1)  = X(6, i)  + obj.dt * (X(4, i)*X(6, i)*obj.a(5) + obj.b(3)*U(4, i) + Disturb);
                X(7, i + 1)  = X(7, i)  + obj.dt * (X(8, i));
                X(8, i + 1)  = X(8, i)  + obj.dt * (Ux(i + 1)*(1/obj.m)*U(1, i) + Disturb);
                X(9, i + 1)  = X(9, i)  + obj.dt * (X(10, i));
                X(10, i + 1) = X(10, i) + obj.dt * (Uy(i + 1)*(1/obj.m)*U(1, i) + Disturb);
                X(11, i + 1) = X(11, i) + obj.dt * (X(12, i));
                X(12, i + 1) = X(12, i) + obj.dt * (-obj.g + (((cos(X(1, i))*cos(X(3, i))*U(1, i))/obj.m)) + Disturb);
            end

            S = [SPhi
                STheta
                SPsi
                SX
                SY
                SZ];

            Uout = [U
                Ux
                Uy];

            % Remove Samples from Start and the End for Simulation Imperfections
            t     = t(10:end - 10);
            X     = X(:, 10:end - 10);
            Uout  = Uout(:, 10:end - 10);
            S     = S(:, 10:end - 10);  
            Xd    = Xd(:, 10:end - 10);
            Omega = Omega(:, 10:end - 10);
            Err   = Err(:, 10:end - 10);
        end
    end
end