function [Xd, dXd, ddXd] = Trajectory(t)
    Xd   = [ cos(t);  sin(t);    sin(2*t) + 2; sin(t)];
    dXd  = [-sin(t);  cos(t);  2*cos(2*t); cos(t)];
    ddXd = [-cos(t); -sin(t); -4*sin(2*t); -sin(t)];
end