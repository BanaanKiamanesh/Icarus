classdef QuadCopter < handle

    properties
        % Dynamical Properties
        Mass
        ArmLength
        Thrust2Drag

        Ixx
        Iyy
        Izz
        I
        InvI

        % Controller Gains
        LinCtrl
        AngCtrl

        % Simulation Properties
        CurrentTime
        dt
        SimTime

        % States
        Pos
        Vel
        Orient
        Omega

        % Input Control Signals
        T
        Tau

        % IMU Noise Variance
        NoiseVar
    end

    % Controllers
    properties
        PhiPID
        ThetaPID
        PsiPID
        ZdotPID
    end

    properties (Constant = true)
        g = 9.81                    % Gravity Const
    end

    methods
        function obj = QuadCopter(DynPar, Controller, InitCond, dt, SimTime)

            % Dynamical Properties
            obj.Mass = DynPar.Mass;
            obj.ArmLength = DynPar.ArmLength;
            obj.Ixx = DynPar.Ixx;
            obj.Iyy = DynPar.Iyy;
            obj.Izz = DynPar.Izz;
            obj.Thrust2Drag = DynPar.Thrust2Drag;

            obj.I = [obj.Ixx, 0, 0
                0, obj.Iyy, 0
                0, 0, obj.Izz];

            obj.InvI = inv(obj.I);

            % Controller Gains and Params
            obj.LinCtrl.Kp  = Controller.Linear.Kp(:);          % Linear Motion Controller
            obj.LinCtrl.Ki  = Controller.Linear.Ki(:);
            obj.LinCtrl.Kd  = Controller.Linear.Kd(:);
            obj.LinCtrl.Tau = Controller.Linear.Tau(:);
            obj.LinCtrl.Sat  = Controller.Linear.Sat(:);

            obj.AngCtrl.Kp  = Controller.Angular.Kp(:);          % Angular Motion Controller
            obj.AngCtrl.Ki  = Controller.Angular.Ki(:);
            obj.AngCtrl.Kd  = Controller.Angular.Kd(:);
            obj.AngCtrl.Tau = Controller.Angular.Tau(:);
            obj.AngCtrl.Sat  = Controller.Angular.Sat(:);

            % Initial Conditions
            obj.Pos = InitCond.Position(:);
            obj.Vel = InitCond.Velocity(:);
            obj.Orient = InitCond.Orientation(:);
            obj.Omega = InitCond.Omega(:);

            % Simulation Properties
            obj.dt = dt;
            obj.SimTime = SimTime;

            % Input Forces
            obj.T = obj.Mass * obj.g;
            % obj.T = 0;
            obj.Tau = [0; 0; 0];
            
            
            obj.NoiseVar = 0.0;

            % Initialize Controller
            obj.ControllerInit();

        end

        function [Xdot, U] = ODE(obj, t, State)

            obj.CurrentTime = t;

            obj.AttitudeControl();
            U = [obj.T
                obj.Tau];

            obj.Pos = State(1:3);
            obj.Vel = State(4:6);
            obj.Orient = State(7:9);
            obj.Omega = State(10:12);

            % State Sanity Assurance
            obj.Pos = obj.Pos(:);
            obj.Vel = obj.Vel(:);
            obj.Orient = obj.Orient(:);
            obj.Omega = obj.Omega(:);

            % Velocity
            Xdot = zeros(12, 1);
            Xdot(1:3) = obj.Vel;

            % Acceleration
            R = RotationMatrix(obj.Orient);
            Xdot(4:6) = [0; 0; -obj.g] + R * [0; 0; obj.T/obj.Mass];

            % Angular Velocity
            Phi   = obj.Orient(1);
            Theta = obj.Orient(2);
            % Psi   = obj.Orient(3);

            Xdot(7:9) = [1,         0, -sin(Theta)
                         0,  cos(Phi), cos(Theta) * sin(Phi)
                         0, -sin(Phi), cos(Theta) * cos(Phi)] \ obj.Omega;

            % Angular Acceleration
            Xdot(10:12) = obj.InvI * (obj.Tau - cross(obj.Omega, obj.I * obj.Omega));

        end

        function [Motion, InSig] = Simulate(obj)
            odeFun = @(t, y) obj.ODE(t, y);
            tSpan = [0, obj.SimTime];

            Y0 = [obj.Pos
                  obj.Vel
                  obj.Orient
                  obj.Omega];

            % Solve ODE for Given Params
            [Motion.t, Motion.Y, InSig.CtrlSig] = RungeKutta4(odeFun, tSpan, obj.dt, Y0);

            % Remove Some Elements from Beginning and the end, Due to
            % Imperfect ODE Solve...
            Motion.t = Motion.t(5:end - 5);
            Motion.Y = Motion.Y(5:end - 5, :);
            InSig.CtrlSig = InSig.CtrlSig(:, 5:end - 5);


            % Extract Motor Thrust Signals from Control Signals
            %   Total Thrust = T1 + T2 + T3 + T4
            %        MomentX = ArmLength * T2 - ArmLength * T4 
            %        MomentY = ArmLength * T1 - ArmLength * T3
            %        MomentZ = Thrust2Drag * T1 - Thrust2Drag * T2
            %                + Thrust2Drag * T3 - Thrust2Drag * T4

            TM = [             1,                1,               1,                1
                               0,    obj.ArmLength,               0,   -obj.ArmLength
                   obj.ArmLength,                0,  -obj.ArmLength,                0
                 obj.Thrust2Drag, -obj.Thrust2Drag, obj.Thrust2Drag, -obj.Thrust2Drag];

            InSig.Thrusts = TM \ InSig.CtrlSig;
        end


        function Val = WrapAngle(obj, Angle)
            Val = rem(Angle - pi, 2*pi) + pi + rand * obj.NoiseVar;
        end
    end

    % Control Methods
    methods

        function ControllerInit(obj)

            % Initialize Angular Controllers
            obj.PhiPID   = PID(obj.AngCtrl.Kp(1), obj.AngCtrl.Ki(1), obj.AngCtrl.Kd(1), obj.AngCtrl.Tau(1), obj.dt, obj.AngCtrl.Sat(1));
            obj.ThetaPID = PID(obj.AngCtrl.Kp(2), obj.AngCtrl.Ki(2), obj.AngCtrl.Kd(2), obj.AngCtrl.Tau(2), obj.dt, obj.AngCtrl.Sat(2));
            obj.PsiPID   = PID(obj.AngCtrl.Kp(3), obj.AngCtrl.Ki(3), obj.AngCtrl.Kd(3), obj.AngCtrl.Tau(3), obj.dt, obj.AngCtrl.Sat(3));

            % Initialize Linear Controllers
            obj.ZdotPID = PID(obj.LinCtrl.Kp(3), obj.LinCtrl.Ki(3), obj.LinCtrl.Kd(3), obj.LinCtrl.Tau(3), obj.dt, obj.LinCtrl.Sat(3));
        end


        function AttitudeControl(obj)
            refSig = [deg2rad(10)
                      deg2rad(10)
                      deg2rad(10)
                      0];

            DesPhi   = refSig(1);
            DesTheta = refSig(2);
            DesPsi   = refSig(3);
            DesZ     = refSig(4);

            obj.Tau(1)     = obj.PhiPID.Update(DesPhi, obj.WrapAngle(obj.Orient(1)));
            obj.Tau(2) = obj.ThetaPID.Update(DesTheta, obj.WrapAngle(obj.Orient(2)));
            obj.Tau(3)     = obj.PsiPID.Update(DesPsi, obj.WrapAngle(obj.Orient(3)));

            % obj.Tau = zeros(3, 1);

%             disp(['CtrlSig: ', num2str(obj.T), mat2str(obj.Tau)])

            obj.T = obj.ZdotPID.Update(DesZ, obj.Vel(3)) + obj.Mass * obj.g;
            % obj.T = obj.Mass * obj.g;
        end
    end
end