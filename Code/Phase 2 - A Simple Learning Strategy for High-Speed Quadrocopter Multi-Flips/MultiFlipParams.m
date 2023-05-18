classdef MultiFlipParams

    properties(GetAccess = private, SetAccess = private)
        mass;
        Ixx;
        length;
        Bup;
        Bdown;
        Cpmax;
        Cn;
        gravity;
    end


    methods

        function obj = MultiFlipParams(TURNS)

            if nargin ~= 1
                TURNS = 3;
            end

            obj.mass    = 0.468;
            obj.Ixx     = 0.0023;
            obj.length  = 0.17;
            obj.Bup     = 21.58;
            obj.Bdown   = 3.92;
            obj.Cpmax   = pi * 1800/180;
            obj.Cn      = TURNS;
            obj.gravity = 9.806;
        end


        function ap = GetAcc(obj, p0, p3)

            ap.acc     = -obj.mass * obj.length * (obj.Bup - p0) / (4 * obj.Ixx);
            ap.start   = obj.mass * obj.length * (obj.Bup - obj.Bdown) / (4 * obj.Ixx);
            ap.coast   = 0;
            ap.stop    = -obj.mass * obj.length * (obj.Bup - obj.Bdown) / (4 * obj.Ixx);
            ap.recover = obj.mass * obj.length * (obj.Bup - p3) / (4 * obj.Ixx);
        end


        function InitParams = GetInitParams(obj)

            p0 = 0.9 * obj.Bup;
            p1 = 0.1;
            p3 = p0;
            p4 = p1;

            acc = obj.GetAcc(p0, p3);
            acc_start= acc.start;

            p2 = (2 * pi * obj.Cn / obj.Cpmax) - (obj.Cpmax / acc_start);

            InitParams = [p0, p1, p2, p3, p4];
        end


        function Sects = GetSection(obj, Pars)

            p0 = Pars(1);
            p1 = Pars(2);
            p2 = Pars(3);
            p3 = Pars(4);
            p4 = Pars(5);

            ap = obj.GetAcc(p0, p3);

            T2 = (obj.Cpmax - p1 * ap.acc) / ap.start;
            T4 = -(obj.Cpmax + p4 * ap.recover) / ap.stop;

            aq = 0;
            ar = 0;

            Sects = cell(5, 3);
            Sects(1, :) = {obj.mass * p0, [ap.acc, aq, ar], p1};

            tmp = obj.mass * obj.Bup - 2 *abs(ap.start) * obj.Ixx / obj.length;
            Sects(2, :) = {tmp, [ap.start, aq, ar], T2};

            Sects(3, :) = {obj.mass * obj.Bdown, [ap.coast, aq, ar], p2};

            tmp = obj.mass * obj.Bup - 2 * abs(ap.stop) * obj.Ixx / obj.length;
            Sects(4, :) = {tmp, [ap.stop, aq, ar], T4};

            Sects(5, :) = {obj.mass * p3, [ap.recover, aq, ar], p4};
        end
    end
end
