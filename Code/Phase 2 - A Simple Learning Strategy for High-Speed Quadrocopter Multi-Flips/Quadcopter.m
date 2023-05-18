classdef Quadcopter < handle

    properties (Constant = true)
        g  = 9.81
        dt = 0.001
    end


    properties (GetAccess = private, SetAccess = private)
        Mass        = 0.468
        ArmLength   = 0.17
        inertia     = [1, 1, 2] * 0.0023
        Thrust2Drag = 0.016

        I
        InvI
    end


    properties (GetAccess = public, SetAccess = private)
        SaveState

        Pos
        Vel
        Orient
        Omega
    end

    methods
        function obj = Quadcopter(SaveState, DynPar)

            if nargin < 2

                if nargin < 1
                    obj.SaveState = true;
                else
                    obj.SaveState = SaveState;
                end

                obj.Mass        = 0.468;
                obj.ArmLength   = 0.17;
                obj.inertia     = [1, 1, 2] * 0.0023;
                obj.Thrust2Drag = 0.016;

            else
                obj.SaveState = SaveState;

                obj.Mass        = DynPar.Mass;
                obj.ArmLength   = DynPar.ArmLength;
                obj.inertia     = DynPar.inertia;
                obj.Thrust2Drag = DynPar.Thrust2Drag;
            end

            obj.I = diag(obj.inertia);
            obj.InvI = inv(obj.I);

            obj.InitState();
        end


        function obj = InitState(obj)
            obj.Pos    = [0, 0, 1];
            obj.Vel    = zeros(1, 3);
            obj.Orient = zeros(1, 3);
            obj.Omega  = zeros(1, 3);
        end


        function Thrust = MotorThrust(obj, Moments, CollThrust)

            Mp = Moments(1);
            Mq = Moments(2);
            Mr = Moments(3);

            Thrust = [(CollThrust + Mr / obj.Thrust2Drag) - (2 * Mq / obj.ArmLength), ...
                      (CollThrust - Mr / obj.Thrust2Drag) + (2 * Mp / obj.ArmLength), ...
                      (CollThrust + Mr / obj.Thrust2Drag) + (2 * Mq / obj.ArmLength), ...
                      (CollThrust - Mr / obj.Thrust2Drag) - (2 * Mp / obj.ArmLength)] / 4;
        end


        function Acc = Acceleration(obj, Thrust, Euler)

            % Rotation Matrix
            RM = RotationMatrix(Euler);

            Acc = RM * [0; 0; sum(Thrust, 'all') / obj.Mass] - [0; 0; obj.g];
        end


        function AngAcc = AngularAcceleration(obj, Omega, Thrust)

            ThrustMatrix = [obj.ArmLength * (Thrust(2) - Thrust(4)), ...
                            obj.ArmLength * (Thrust(3) - Thrust(1)), ...
                            obj.Thrust2Drag * (Thrust(1) - Thrust(2) + Thrust(3) - Thrust(4))];

            AngAcc = obj.InvI * ThrustMatrix' - cross(obj.InvI * Omega, obj.I * Omega);
        end


        function Mom = Moments(obj, DesAcc, AngVel)
            Mom = obj.I * (DesAcc' + cross(obj.InvI * AngVel, obj.I * AngVel));
        end


        function [FinalState, Time] = Update(obj, Sect)

            CollT     = cell2mat(Sect(:, 1));
            DesAngAcc = cell2mat(Sect(:, 2));
            T         = cell2mat(Sect(:, 3));

            if obj.SaveState

                OverallLength = 0;

                for i = 1: numel(CollT)
                    OverallLength = OverallLength + ceil(T(i) / obj.dt);
                end

                OverallLength = OverallLength - (numel(CollT) - 1);

                FinalState = zeros(OverallLength, 12);

            else
                FinalState = [];
            end


            Idx  = 1;
            Time = [];


            for i = 1:numel(CollT)

                CollThrust    = CollT(i);
                DesAngularAcc = DesAngAcc(i, :);
                time          = T(i);

                if time < (2 * obj.dt)
                    continue
                end

                tSpan = (0:obj.dt:time);
                State = [obj.Pos, obj.Vel, obj.Orient, obj.Omega];

                [tmpT, Out] = RungeKutta4(@(t, y)obj.Integrator(y, t, CollThrust, DesAngularAcc), ...
                    tSpan, obj.dt, State);

                obj.Pos    = Out(end, 1:3);
                obj.Vel    = Out(end, 4:6);
                obj.Orient = Out(end, 7:9);
                obj.Omega  = Out(end, 10:12);

                OutLen = size(Out, 1);

                if obj.SaveState
                    FinalState(Idx: Idx + OutLen - 1, :) = Out;
                    Idx = Idx + OutLen - 1;
                end
            end

            Time = linspace(0, tmpT(end), size(FinalState, 1));
            % 
            % Time = 0:obj.dt:sum(T, 'all');
        end


        function SysState = Integrator(obj, State, ~, CollThrust, DesAngAcc)

            obj.Pos = State(1:3);
            obj.Vel = State(4:6);
            obj.Orient = State(7:9);
            obj.Omega  = State(10:12);

            Moments = obj.Moments(DesAngAcc, obj.Omega);
            Thrusts = obj.MotorThrust(Moments, CollThrust);
            Acc     = obj.Acceleration(Thrusts, obj.Orient);

            AngAcc  = obj.AngularAcceleration(obj.Omega, Thrusts);
            obj.Omega = AngVel2dtEuler(obj.Omega, obj.Orient);

            SysState = [obj.Vel(:); Acc(:); obj.Omega(:); AngAcc(:)];
        end
    end
end


function dtAngVel = dtEuler2AngVel(dtEuler, Euler)                                    %#ok<DEFNU>
    Euler    = reshape(Euler, [1, 3]);
    dtAngVel = (AgularRotationMatrix(Euler) * dtEuler)';
end


function dtEuler = AngVel2dtEuler(Omega, Euler)
    dtEuler = AgularRotationMatrix(Euler) \ Omega;
end


function R = AgularRotationMatrix(Euler)

    Phi   = Euler(1);
    Theta = Euler(2);
    Psi   = Euler(3);

    R = [1,        0,           -sin(Theta)
         0, cos(Phi), cos(Theta) * sin(Phi)
         0, sin(Psi), cos(Theta) * cos(Phi)];
end
