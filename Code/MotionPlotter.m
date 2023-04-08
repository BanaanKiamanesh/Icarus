classdef MotionPlotter < handle

    properties
        t
        Pos
        Vel
        Orient
        Omega

        ArmLen
    end

    properties
        LinearMotionPlot
        AngularMotionPlot
        PlotTraj
    end

    methods
        function obj = MotionPlotter(t, Motion, ArmLength)
            obj.Pos = Motion(:, 1:3);
            obj.Vel = Motion(:, 4:6);
            obj.Orient = Motion(:, 7:9);
            obj.Omega = Motion(:, 10:12);

            obj.t = t;

            obj.ArmLen = ArmLength;
        end

        function PlotLinearMotion(obj)
            obj.LinearMotionPlot = figure('Name', 'Position and Velocity Motion Plots', ...
                'Units', 'normalized', 'OuterPosition', [0, 0, 1, 1]);
            obj.LinearMotionPlot.GraphicsSmoothing = 'on';

            title("Position and Velocity Motion")

            subplot(2, 3, 1)
            plot(obj.t, obj.Pos(:, 1), "LineWidth", 3)
            xlabel("X (m)")
            ylabel("t (s)")
            grid minor

            subplot(2, 3, 2)
            plot(obj.t, obj.Pos(:, 2), "LineWidth", 3)
            xlabel("Y (m)")
            ylabel("t (s)")
            grid minor

            subplot(2, 3, 3)
            plot(obj.t, obj.Pos(:, 3), "LineWidth", 3)
            xlabel("Z (m)")
            ylabel("t (s)")
            grid minor

            subplot(2, 3, 4)
            plot(obj.t, obj.Vel(:, 1), "LineWidth", 3)
            xlabel("dX (m/s)")
            ylabel("t (s)")
            grid minor

            subplot(2, 3, 5)
            plot(obj.t, obj.Vel(:, 2), "LineWidth", 3)
            xlabel("dY (m/s)")
            ylabel("t (s)")
            grid minor

            subplot(2, 3, 6)
            plot(obj.t, obj.Vel(:, 3), "LineWidth", 3)
            xlabel("dZ (m/s)")
            ylabel("t (s)")
            grid minor
        end

        function PlotAngularMotion(obj)
            obj.AngularMotionPlot = figure('Name', 'Orientation and Angular Velocity Motion Plots', ...
                'Units', 'normalized', 'OuterPosition', [0, 0, 1, 1]);
            obj.AngularMotionPlot.GraphicsSmoothing = 'on';

            title("Orientation and Angular Velocity Motion")

            subplot(2, 3, 1)
            plot(obj.t, obj.Orient(:, 1), "LineWidth", 3)
            xlabel("\phi (radians)")
            ylabel("t (s)")
            grid minor

            subplot(2, 3, 2)
            plot(obj.t, obj.Orient(:, 2), "LineWidth", 3)
            xlabel("\theta (radians)")
            ylabel("t (s)")
            grid minor

            subplot(2, 3, 3)
            plot(obj.t, obj.Orient(:, 3), "LineWidth", 3)
            xlabel("\psi (radians)")
            ylabel("t (s)")
            grid minor

            subplot(2, 3, 4)
            plot(obj.t, obj.Omega(:, 1), "LineWidth", 3)
            xlabel("d\phi (radians/s)")
            ylabel("t (s)")
            grid minor

            subplot(2, 3, 5)
            plot(obj.t, obj.Omega(:, 2), "LineWidth", 3)
            xlabel("d\theta (radians/s)")
            ylabel("t (s)")
            grid minor

            subplot(2, 3, 6)
            plot(obj.t, obj.Omega(:, 3), "LineWidth", 3)
            xlabel("d\psi (radians/s)")
            ylabel("t (s)")
            grid minor
        end

        function Plot3D(obj)

            obj.PlotTraj = figure('Name', '3D Trajectory Plot', 'Units', 'normalized', 'OuterPosition', [0, 0, 1, 1]);
            obj.PlotTraj.GraphicsSmoothing = 'on';
            grid on
            axis equal;
            xlabel("X")
            ylabel("Y")
            zlabel("Z")
            % axis off
%             view(z3)
%             view(90, 0)
            view(0, 0)
%             view(0, 90)

            % Setting Limits to the Plot
            RoomDims.X = [-5, 5];
            RoomDims.Y = [-5, 5];
            RoomDims.Z = [0, 5];

            xlim(RoomDims.X)
            ylim(RoomDims.Y)
            zlim(RoomDims.Z)


            % Initial MotorCoordinates
            MotorCoord = [obj.ArmLen,           0, 0
                                   0,  obj.ArmLen, 0
                         -obj.ArmLen,           0, 0
                                   0, -obj.ArmLen, 0];

            % Create Primary Plot Objects
            hold(gca, 'on');
            ArmX = plot3(gca, MotorCoord([1, 3], 1) + obj.Pos(1, 1), ...
                              MotorCoord([1, 3], 2) + obj.Pos(1, 2), ...
                              MotorCoord([1, 3], 3) + obj.Pos(1, 3), ...
                              '-ro' ,'MarkerSize', 5);

            ArmY = plot3(gca, MotorCoord([2, 4], 1) + obj.Pos(1, 1), ...
                              MotorCoord([2, 4], 2) + obj.Pos(1, 2), ...
                              MotorCoord([2, 4], 3) + obj.Pos(1, 3), ...
                              '-bo','MarkerSize', 5);

            PayLoad = plot3(gca, obj.Pos(1, 1), ...
                                 obj.Pos(1, 2), ...
                                 obj.Pos(1, 3), ...
                                 'ok', 'Linewidth', 3);

            Shadow = plot3(gca, obj.Pos(1, 1), ...
                                obj.Pos(1, 2), ...
                                0, ...
                                'xk', 'LineWidth', 5);

            hold(gca, 'off');

            % Main Simulation Loop
            for i = 1:length(obj.Pos)

                % Extract Current Orientation and Position
                Orientation = obj.Orient(i, 1:3);
                Center = obj.Pos(i, 1:3);

                % Create a Temp Mat to Hold Rotated MotorCoords
                tmp = zeros(size(MotorCoord));

                for j = 1:size(MotorCoord, 1)       % Rotate Points
                    tmp(j, :) = RotatePoint(Orientation, MotorCoord(j, :), Center)';
                end

                set(ArmX, ...
                    'XData', tmp([1, 3], 1), ...
                    'YData', tmp([1, 3], 2), ...
                    'ZData', tmp([1, 3], 3));

                set(ArmY, ...
                    'XData', tmp([2, 4], 1), ...
                    'YData', tmp([2, 4], 2), ...
                    'ZData', tmp([2, 4], 3));

                set(PayLoad, ...
                    'XData', Center(1), ...
                    'YData', Center(2), ...
                    'ZData', Center(3));

                set(Shadow, ...
                    'XData', Center(1), ...
                    'YData', Center(2), ...
                    'ZData', 0);

                drawnow %limitrate

                if    Center(3) < RoomDims.Z(1) || Center(3) > RoomDims.Z(2) ...
                   || Center(2) < RoomDims.Y(1) || Center(2) > RoomDims.Y(2) ...
                   || Center(1) < RoomDims.X(1) || Center(1) > RoomDims.X(2)

                    msgbox("Crashed..!", 'Error', 'error');
                    break;
                end

            end
        end
    end
end


function NewPoint = RotatePoint(Orient, Point, OffSet)

    if nargin == 2
        OffSet = [0; 0; 0];
    end

    % Sanity Check
    OffSet = reshape(OffSet, [3, 1]);
    Point = reshape(Point, [3, 1]);


    % Create the Rotation Matrix
    Phi = Orient(1);
    Theta = Orient(2);
    Psi = Orient(3);

    cPhi = cos(Phi);
    sPhi = sin(Phi);
    cThe = cos(Theta);
    sThe = sin(Theta);
    cPsi = cos(Psi);
    sPsi = sin(Psi);

    RotMat = [cThe * cPsi , sPhi * sThe * cPsi - cPhi * sPsi, cPhi * sThe * cPsi + sPhi * sPsi
              cThe * sPsi , sPhi * sThe * sPsi + cPhi * cPsi, cPhi * sThe * sPsi - sPhi * cPsi
              -sThe       , cThe * sPhi                     , cThe * cPhi];

    % Rotate The Point
    NewPoint = RotMat * Point + OffSet;
end
