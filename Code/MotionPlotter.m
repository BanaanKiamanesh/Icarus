classdef MotionPlotter < handle

    properties
        t
        Pos
        Vel
        Orient
        Omega

        CtrlSig
        Thrusts

        ArmLen
    end

    properties
        ThrustPlot
        CtrlSigPlot
        LinearMotionPlot
        AngularMotionPlot
        Viz3D
    end

    methods
        function obj = MotionPlotter(t, Motion, CtrlSig, Thrusts, ArmLength)
            obj.Pos = Motion(:, 1:3);
            obj.Vel = Motion(:, 4:6);
            obj.Orient = Motion(:, 7:9);
            obj.Omega = Motion(:, 10:12);

            obj.Thrusts = Thrusts;
            obj.CtrlSig = CtrlSig;

            obj.t = t;

            obj.ArmLen = ArmLength;
        end

        function PlotControlSignals(obj)
            obj.CtrlSigPlot = figure('Name', 'Control Signals Plots', ...
                'Units', 'normalized', 'OuterPosition', [0, 0, 1, 1]);
            obj.CtrlSigPlot.GraphicsSmoothing = 'on';
            obj.CtrlSigPlot.Color = [1, 1, 1];
            obj.CtrlSigPlot = gca;
            obj.CtrlSigPlot.FontSize = 10;
            obj.CtrlSigPlot.FontWeight = 'B';

            subplot(4, 1, 1)
            plot(obj.t, obj.CtrlSig(1, :), "LineWidth", 2.5)
            ylabel("\SigmaThrust", 'FontSize', 10,'FontWeight', 'Bold')
            xlabel("t (s)", 'FontSize', 10,'FontWeight', 'Bold')
            grid minor
            title("Total Thrust", 'FontSize', 14,'FontWeight', 'Bold')

            subplot(4, 1, 2)
            plot(obj.t, obj.CtrlSig(2, :), "LineWidth", 2.5)
            ylabel("M_x", 'FontSize', 10,'FontWeight', 'Bold')
            xlabel("t (s)", 'FontSize', 10,'FontWeight', 'Bold')
            grid minor
            title("Moment Along X Axis", 'FontSize', 14,'FontWeight', 'Bold')

            subplot(4, 1, 3)
            plot(obj.t, obj.CtrlSig(3, :), "LineWidth", 2.5)
            ylabel("M_y", 'FontSize', 10,'FontWeight', 'Bold')
            xlabel("t (s)", 'FontSize', 10,'FontWeight', 'Bold')
            grid minor
            title("Moment Along Y Axis", 'FontSize', 14,'FontWeight', 'Bold')

            subplot(4, 1, 4)
            plot(obj.t, obj.CtrlSig(4, :), "LineWidth", 2.5)
            ylabel("M_z", 'FontSize', 10,'FontWeight', 'Bold')
            xlabel("t (s)", 'FontSize', 10,'FontWeight', 'Bold')
            grid minor
            title("Moment Along Z Axis", 'FontSize', 14,'FontWeight', 'Bold')
        end


        function PlotThrusts(obj)
            obj.ThrustPlot = figure('Name', 'Motor Thrusts Plots', ...
                'Units', 'normalized', 'OuterPosition', [0, 0, 1, 1]);
            obj.ThrustPlot.GraphicsSmoothing = 'on';
            obj.ThrustPlot.Color = [1, 1, 1];
            obj.ThrustPlot = gca;
            obj.ThrustPlot.FontSize = 10;
            obj.ThrustPlot.FontWeight = 'B';

            subplot(4, 1, 1)
            plot(obj.t, obj.Thrusts(1, :), "LineWidth", 2.5)
            ylabel("T_1 (kg)", 'FontSize', 10,'FontWeight', 'Bold')
            xlabel("t (s)", 'FontSize', 10,'FontWeight', 'Bold')
            grid minor
            title("Motor 1", 'FontSize', 14,'FontWeight', 'Bold')

            subplot(4, 1, 2)
            plot(obj.t, obj.Thrusts(2, :), "LineWidth", 2.5)
            ylabel("T_2 (kg)", 'FontSize', 10,'FontWeight', 'Bold')
            xlabel("t (s)", 'FontSize', 10,'FontWeight', 'Bold')
            grid minor
            title("Motor 2", 'FontSize', 14,'FontWeight', 'Bold')

            subplot(4, 1, 3)
            plot(obj.t, obj.Thrusts(3, :), "LineWidth", 2.5)
            ylabel("T_3 (kg)", 'FontSize', 10,'FontWeight', 'Bold')
            xlabel("t (s)", 'FontSize', 10,'FontWeight', 'Bold')
            grid minor
            title("Motor 3", 'FontSize', 14,'FontWeight', 'Bold')

            subplot(4, 1, 4)
            plot(obj.t, obj.Thrusts(4, :), "LineWidth", 2.5)
            ylabel("T_2 (kg)", 'FontSize', 10,'FontWeight', 'Bold')
            xlabel("t (s)", 'FontSize', 10,'FontWeight', 'Bold')
            grid minor
            title("Motor 4", 'FontSize', 14,'FontWeight', 'Bold')
        end


        function PlotLinearMotion(obj)
            obj.LinearMotionPlot = figure('Name', 'Position and Velocity Motion Plots', ...
                'Units', 'normalized', 'OuterPosition', [0, 0, 1, 1]);
            obj.LinearMotionPlot.GraphicsSmoothing = 'on';
            obj.LinearMotionPlot.Color = [1, 1, 1];
            obj.LinearMotionPlot = gca;
            obj.LinearMotionPlot.FontSize = 10;
            obj.LinearMotionPlot.FontWeight = 'B';

            title("Position and Velocity Motion")

            subplot(2, 3, 1)
            plot(obj.t, obj.Pos(:, 1), "LineWidth", 2.5)
            ylabel("X (m)", 'FontSize', 10,'FontWeight', 'Bold')
            xlabel("t (s)", 'FontSize', 10,'FontWeight', 'Bold')
            grid minor
            title("X", 'FontSize', 14,'FontWeight', 'Bold')

            subplot(2, 3, 2)
            plot(obj.t, obj.Pos(:, 2), "LineWidth", 2.5)
            ylabel("Y (m)", 'FontSize', 10,'FontWeight', 'Bold')
            xlabel("t (s)", 'FontSize', 10,'FontWeight', 'Bold')
            grid minor
            title("Y", 'FontSize', 14,'FontWeight', 'Bold')

            subplot(2, 3, 3)
            plot(obj.t, obj.Pos(:, 3), "LineWidth", 2.5)
            ylabel("Z (m)", 'FontSize', 10,'FontWeight', 'Bold')
            xlabel("t (s)", 'FontSize', 10,'FontWeight', 'Bold')
            grid minor
            title("Z", 'FontSize', 14,'FontWeight', 'Bold')

            subplot(2, 3, 4)
            plot(obj.t, obj.Vel(:, 1), "LineWidth", 2.5)
            ylabel("$$ \bf{\dot{X} (m/s)} $$", 'FontSize', 10,'FontWeight', 'Bold', 'Interpreter', 'Latex')
            xlabel("t (s)", 'FontSize', 10,'FontWeight', 'Bold')
            grid minor
            title("Velocity Along X", 'FontSize', 14,'FontWeight', 'Bold')

            subplot(2, 3, 5)
            plot(obj.t, obj.Vel(:, 2), "LineWidth", 2.5)
            ylabel("$$ \bf{\dot{Y} (m/s)} $$", 'FontSize', 10,'FontWeight', 'Bold', 'Interpreter', 'Latex')
            xlabel("t (s)", 'FontSize', 10,'FontWeight', 'Bold')
            grid minor
            title("Velocity Along Y", 'FontSize', 14,'FontWeight', 'Bold')

            subplot(2, 3, 6)
            plot(obj.t, obj.Vel(:, 3), "LineWidth", 2.5)
            ylabel("$$ \bf{\dot{Z} (m/s)} $$", 'FontSize', 10,'FontWeight', 'Bold', 'Interpreter', 'Latex')
            xlabel("t (s)", 'FontSize', 10,'FontWeight', 'Bold')
            grid minor
            title("Velocity Along Z", 'FontSize', 14,'FontWeight', 'Bold')
        end


        function PlotAngularMotion(obj)
            obj.AngularMotionPlot = figure('Name', 'Orientation and Angular Velocity Motion Plots', ...
                'Units', 'normalized', 'OuterPosition', [0, 0, 1, 1]);
            obj.AngularMotionPlot.GraphicsSmoothing = 'on';
            obj.AngularMotionPlot.Color = [1, 1, 1];
            obj.AngularMotionPlot = gca;
            obj.AngularMotionPlot.FontSize = 10;
            obj.AngularMotionPlot.FontWeight = 'B';

            subplot(2, 3, 1)
            plot(obj.t, obj.Orient(:, 1), "LineWidth", 2.5)
            ylabel("\phi (radians)", 'FontSize', 10,'FontWeight', 'Bold')
            xlabel("t (s)", 'FontSize', 10,'FontWeight', 'Bold')
            grid minor
            title("Pitch", 'FontSize', 14,'FontWeight', 'Bold')

            subplot(2, 3, 2)
            plot(obj.t, obj.Orient(:, 2), "LineWidth", 2.5)
            ylabel("\theta (radians)", 'FontSize', 10,'FontWeight', 'Bold')
            xlabel("t (s)", 'FontSize', 10,'FontWeight', 'Bold')
            grid minor
            title("Roll", 'FontSize', 14,'FontWeight', 'Bold')

            subplot(2, 3, 3)
            plot(obj.t, obj.Orient(:, 3), "LineWidth", 2.5)
            ylabel("\psi (radians)", 'FontSize', 10,'FontWeight', 'Bold')
            xlabel("t (s)", 'FontSize', 10,'FontWeight', 'Bold')
            grid minor
            title("Yaw", 'FontSize', 14,'FontWeight', 'Bold')

            subplot(2, 3, 4)
            plot(obj.t, obj.Omega(:, 1), "LineWidth", 2.5)
            ylabel("$$ \bf{\dot{\phi} (rad/s)} $$", 'FontSize', 10,'FontWeight', 'Bold', 'Interpreter', 'Latex')
            xlabel("t (s)", 'FontSize', 10,'FontWeight', 'Bold')
            grid minor
            title("Angular Velocity Along X", 'FontSize', 14,'FontWeight', 'Bold')

            subplot(2, 3, 5)
            plot(obj.t, obj.Omega(:, 2), "LineWidth", 2.5)
            ylabel("$$ \bf{\dot{\theta} (rad/s)} $$", 'FontSize', 10,'FontWeight', 'Bold', 'Interpreter', 'Latex')
            xlabel("t (s)", 'FontSize', 10,'FontWeight', 'Bold')
            grid minor
            title("Angular Velocity Along Y", 'FontSize', 14,'FontWeight', 'Bold')

            subplot(2, 3, 6)
            plot(obj.t, obj.Omega(:, 3), "LineWidth", 2.5)
            ylabel("$$ \bf{\dot{\psi} (rad/s)} $$", 'FontSize', 10,'FontWeight', 'Bold', 'Interpreter', 'Latex')
            xlabel("t (s)", 'FontSize', 10,'FontWeight', 'Bold')
            grid minor
            title("Angular Velocity Along Z", 'FontSize', 14,'FontWeight', 'Bold')
        end


        function Plot3D(obj)

            obj.Viz3D = figure('Name', '3D Trajectory Plot', 'Units', 'normalized', 'OuterPosition', [0, 0, 1, 1]);
            obj.Viz3D.GraphicsSmoothing = 'on';
            obj.Viz3D.Color = [1, 1, 1];
            obj.Viz3D = gca;
            obj.Viz3D.FontSize = 10;
            obj.Viz3D.FontWeight = 'B';
            grid minor
            axis equal;
            title("3D Visualization of the Motion", 'FontSize', 16, 'FontWeight', 'Bold')
            xlabel("X", 'FontSize', 14,'FontWeight', 'Bold')
            ylabel("Y", 'FontSize', 14,'FontWeight', 'Bold')
            zlabel("Z", 'FontSize', 14,'FontWeight', 'Bold')
            % axis off
            view(3)
            %             view(90, 0)
            %             view(0, 0)
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
                '-ro','MarkerSize', 5);

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

                drawnow expose %limitrate

                % Crash Detection
                if    Center(3) < RoomDims.Z(1) || Center(3) > RoomDims.Z(2) ...
                        || Center(2) < RoomDims.Y(1) || Center(2) > RoomDims.Y(2) ...
                        || Center(1) < RoomDims.X(1) || Center(1) > RoomDims.X(2)

                    %                     msgbox("Crashed..!", 'Error', 'error');
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

    RotMat = [cThe * cPsi, sPhi * sThe * cPsi - cPhi * sPsi, cPhi * sThe * cPsi + sPhi * sPsi
              cThe * sPsi, sPhi * sThe * sPsi + cPhi * cPsi, cPhi * sThe * sPsi - sPhi * cPsi
                    -sThe,                      cThe * sPhi,                      cThe * cPhi];

    % Rotate The Point
    NewPoint = RotMat * Point + OffSet;
end
