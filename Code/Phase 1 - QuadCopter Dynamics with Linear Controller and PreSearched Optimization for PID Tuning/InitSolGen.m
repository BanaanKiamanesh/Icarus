function Sol = InitSolGen()
    %INITSOLGEN Initial Solution Generator

    % 6 PID Controller with 4 Parameters Each ==> (Kp, Ki, Kd, Tau)
    % Linear Motion PID
    % Random Numbers Varying in Range [-0.005 ~ 0.005]
    Sol(1 : 3) = rand(3, 1) * 0.01 - 0.005;
    Sol(4 : 6) = rand(3, 1) * 0.01 - 0.005;
    Sol(7 : 9) = rand(3, 1) * 0.01 - 0.005;
    Sol(10:12) = rand(3, 1) * 0.01 - 0.005;

    % Angular Motion PID
    % Random Numbers Varying in Range [-0.005 ~ 0.005]
    Sol(13:15) = rand(3, 1) * 0.01 - 0.005;
    Sol(16:18) = rand(3, 1) * 0.01 - 0.005;
    Sol(19:21) = rand(3, 1) * 0.01 - 0.005;
    Sol(22:24) = rand(3, 1) * 0.01 - 0.005;
end