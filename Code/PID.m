classdef PID < handle

    properties
        Kp
        Ki
        Kd
        dt

        PrevErr
        ErrIntegral
    end

    methods
        function obj = PID(Kp, Ki, Kd, dt)
            obj.Kp = Kp;
            obj.Ki = Ki;
            obj.Kd = Kd;

            obj.dt = dt;

            % Init Values
            obj.ErrIntegral = 0;
            obj.PrevErr = 0;
        end

        function Val = Update(obj, Des, Meas)

            % Calculate Error
            Err = Des - Meas;

            % Proportional Term
            P = obj.Kp * Err;

            % Integral Term
            obj.ErrIntegral = obj.ErrIntegral + Err * obj.dt;
            I = obj.Ki * obj.ErrIntegral;

            % Derivative Term
            D = obj.Kd * (Err - obj.PrevErr) / obj.dt;

            obj.PrevErr = Err;

            Val = P + I + D;
        end
    end
end