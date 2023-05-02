classdef PID < handle

    properties
        Kp
        Ki
        Kd
        Tau
        T
        SigLim

        PrevErr
        PrevInt
        PrevDeriv
    end

    methods
        function obj = PID(Kp, Ki, Kd, Tau, dt, SigLim)
            obj.Kp  = Kp;
            obj.Ki  = Ki;
            obj.Kd  = Kd;
            obj.Tau = Tau;
            obj.SigLim = abs(SigLim);

            obj.T = dt;

            % Init Values
            obj.PrevInt = 0;
            obj.PrevErr = 0;
            obj.PrevDeriv = 0;
        end

        function Val = Update(obj, Des, Meas)

            % Calculate Error
            Err = Des - Meas;

            % Proportional Term
            P = obj.Kp * Err;

            % Integral Term
            I = (obj.Ki * obj.T / 2) * (Err + obj.PrevErr) + obj.PrevInt;

            % Derivative Term
            D = (2 * obj.Kd / (2 * obj.Tau + obj.T)) * (Err - obj.PrevErr) ...
              + (2 * obj.Tau - obj.T) / (2 * obj.Tau + obj.T) * obj.PrevDeriv;

            obj.PrevErr = Err;
            obj.PrevInt = I;
            obj.PrevDeriv = D;

            Val = P + I + D;

            Val = Constrain(Val, -obj.SigLim, obj.SigLim);
        end
    end
end


function Val = Constrain(X, LB, UB)

    % Swap Places if LB and UP are not Set Properly
    if LB > UB
        tmp = LB;
        LB = UB;
        UB = tmp;
    end

    Val = max(LB, min(UB, X));
end