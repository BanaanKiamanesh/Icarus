function Trajectory = Traj(t)

    % Function to Generate the QuadCopter Reference Trajectory
    % The OutPut Must be a Vector Containing X, Y, Z, Psi Respectively!

    X = cos(t);
    Y = sin(2 * t);
    Z = 2.5 + sin(t);
    Psi = sin(t);

    Trajectory = [X
                  Y
                  Z
                  Psi];
end