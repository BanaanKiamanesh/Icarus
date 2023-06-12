function y = Disturbance(t)
    y = 0.5*sin(t) + 0.1*cos(t*4) + 0.02*sin(t*10);
    % y = y*0.2;
end