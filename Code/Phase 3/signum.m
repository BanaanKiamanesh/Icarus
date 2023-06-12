function y = signum(x)

    Mode = 2;
    % Mode 1 ==> Sign Function
    if Mode == 1
        y = sign(x);

        % Mode 2 ==> Tanh
    elseif Mode == 2
        epsilon = 0.2;
        y = tanh(x / epsilon);

        % Mode 3 ==> Regularized
    elseif Mode == 3
        epsilon = 0.01;
        y = x / (abs(x) + epsilon);
    end
end