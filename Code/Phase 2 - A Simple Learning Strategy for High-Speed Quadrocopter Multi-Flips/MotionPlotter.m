classdef MotionPlotter < handle

    % Internal Data Properties
    properties (GetAccess = private)
        t
        Pos
        Vel
        Orient
        Omega

        ArmLen
    end

    % Plot Objects
    properties (GetAccess = private, SetAccess = private)
        LinearMotionPlot
        AngularMotionPlot
        Viz3D

        Path
    end

    methods
        function obj = MotionPlotter(t, Motion, ArmLength)
            % Unpack Motion to Pos, Vel, Orient and Ang Orient
            obj.Pos    = Motion(:, 1:3);
            obj.Vel    = Motion(:, 4:6);
            obj.Orient = Motion(:, 7:9);
            obj.Omega  = Motion(:, 10:12);

            obj.t = t;                          % Time Vector

            obj.ArmLen = ArmLength;             % Quad Arm Length
        end

        function PlotLinearMotion(obj, Save)

            if nargin == 1          % Set Default Arg
                Save = false;
            end

            % Create Plot Object
            obj.LinearMotionPlot = figure('Name', 'Position and Velocity Motion Plots', ...
                'Units', 'normalized', 'OuterPosition', [0, 0, 1, 1]);
            obj.LinearMotionPlot.GraphicsSmoothing = 'on';
            obj.LinearMotionPlot.Color = [1, 1, 1];

            % Set Super Title
            sgtitle('Linear Motion', 'FontSize', 24,'FontWeight', 'Bold')

            % Position and Velocity
            subplot(2, 3, 1)
            hold(gca, 'on');
            plot(obj.t, obj.Pos(:, 1), 'LineWidth', 2.5)
            hold(gca, 'off');
            ylabel('X (m)', 'FontSize', 10,'FontWeight', 'Bold')
            xlabel('t (s)', 'FontSize', 10,'FontWeight', 'Bold')
            title('X', 'FontSize', 14,'FontWeight', 'Bold')
            grid minor

            subplot(2, 3, 2)
            hold(gca, 'on');
            plot(obj.t, obj.Pos(:, 2), 'LineWidth', 2.5)
            hold(gca, 'off');
            ylabel('Y (m)', 'FontSize', 10,'FontWeight', 'Bold')
            xlabel('t (s)', 'FontSize', 10,'FontWeight', 'Bold')
            title('Y', 'FontSize', 14,'FontWeight', 'Bold')
            grid minor

            subplot(2, 3, 3)
            hold(gca, 'on');
            plot(obj.t, obj.Pos(:, 3), 'LineWidth', 2.5)
            hold(gca, 'off');
            ylabel('Z (m)', 'FontSize', 10,'FontWeight', 'Bold')
            xlabel('t (s)', 'FontSize', 10,'FontWeight', 'Bold')
            title('Z', 'FontSize', 14,'FontWeight', 'Bold')
            grid minor

            subplot(2, 3, 4)
            plot(obj.t, obj.Vel(:, 1), 'LineWidth', 2.5)
            ylabel('$$ \bf{\dot{X} (m/s)} $$', 'FontSize', 10,'FontWeight', 'Bold', 'Interpreter', 'Latex')
            xlabel('t (s)', 'FontSize', 10,'FontWeight', 'Bold')
            title('Velocity Along X', 'FontSize', 14,'FontWeight', 'Bold')
            grid minor

            subplot(2, 3, 5)
            plot(obj.t, obj.Vel(:, 2), 'LineWidth', 2.5)
            ylabel('$$ \bf{\dot{Y} (m/s)} $$', 'FontSize', 10,'FontWeight', 'Bold', 'Interpreter', 'Latex')
            xlabel('t (s)', 'FontSize', 10,'FontWeight', 'Bold')
            title('Velocity Along Y', 'FontSize', 14,'FontWeight', 'Bold')
            grid minor

            subplot(2, 3, 6)
            plot(obj.t, obj.Vel(:, 3), 'LineWidth', 2.5)
            ylabel('$$ \bf{\dot{Z} (m/s)} $$', 'FontSize', 10,'FontWeight', 'Bold', 'Interpreter', 'Latex')
            xlabel('t (s)', 'FontSize', 10,'FontWeight', 'Bold')
            title('Velocity Along Z', 'FontSize', 14,'FontWeight', 'Bold')
            grid minor

            % Make Directory and Save Plot if Save is Set
            if Save
                mkdir Plots

                try
                    rmdir Plots\LinearMotion
                catch
                end

                mkdir Plots\LinearMotion
                exportgraphics(obj.LinearMotionPlot, 'Plots/LinearMotion/LinearMotion.jpg');
                exportgraphics(obj.LinearMotionPlot, 'Plots/LinearMotion/LinearMotion.pdf');
            end
        end


        function PlotAngularMotion(obj, Save)

            if nargin == 1          % Set Default Arg
                Save = false;
            end

            % Create Plot Object
            obj.AngularMotionPlot = figure('Name', 'Orientation and Angular Velocity Motion Plots', ...
                'Units', 'normalized', 'OuterPosition', [0, 0, 1, 1]);
            obj.AngularMotionPlot.GraphicsSmoothing = 'on';
            obj.AngularMotionPlot.Color = [1, 1, 1];

            % Set Super Title
            sgtitle('Angular Motion', 'FontSize', 24,'FontWeight', 'Bold')

            % Roation Angle and Angular Velocities Plots
            subplot(2, 3, 1)
            plot(obj.t, obj.Orient(:, 1), 'LineWidth', 2.5)
            ylabel('\phi (radians)', 'FontSize', 10,'FontWeight', 'Bold')
            xlabel('t (s)', 'FontSize', 10,'FontWeight', 'Bold')
            title('Pitch', 'FontSize', 14,'FontWeight', 'Bold')
            grid minor

            subplot(2, 3, 2)
            plot(obj.t, obj.Orient(:, 2), 'LineWidth', 2.5)
            ylabel('\theta (radians)', 'FontSize', 10,'FontWeight', 'Bold')
            xlabel('t (s)', 'FontSize', 10,'FontWeight', 'Bold')
            title('Roll', 'FontSize', 14,'FontWeight', 'Bold')
            grid minor

            subplot(2, 3, 3)
            hold(gca, 'on');
            plot(obj.t, obj.Orient(:, 3), 'LineWidth', 2.5)
            hold(gca, 'off');
            ylabel('\psi (radians)', 'FontSize', 10,'FontWeight', 'Bold')
            xlabel('t (s)', 'FontSize', 10,'FontWeight', 'Bold')
            title('Yaw', 'FontSize', 14,'FontWeight', 'Bold')
            grid minor

            subplot(2, 3, 4)
            plot(obj.t, obj.Omega(:, 1), 'LineWidth', 2.5)
            ylabel('$$ \bf{\dot{\phi} (rad/s)} $$', 'FontSize', 10,'FontWeight', 'Bold', 'Interpreter', 'Latex')
            xlabel('t (s)', 'FontSize', 10,'FontWeight', 'Bold')
            title('Angular Velocity Along X', 'FontSize', 14,'FontWeight', 'Bold')
            grid minor

            subplot(2, 3, 5)
            plot(obj.t, obj.Omega(:, 2), 'LineWidth', 2.5)
            ylabel('$$ \bf{\dot{\theta} (rad/s)} $$', 'FontSize', 10,'FontWeight', 'Bold', 'Interpreter', 'Latex')
            xlabel('t (s)', 'FontSize', 10,'FontWeight', 'Bold')
            title('Angular Velocity Along Y', 'FontSize', 14,'FontWeight', 'Bold')
            grid minor

            subplot(2, 3, 6)
            plot(obj.t, obj.Omega(:, 3), 'LineWidth', 2.5)
            ylabel('$$ \bf{\dot{\psi} (rad/s)} $$', 'FontSize', 10,'FontWeight', 'Bold', 'Interpreter', 'Latex')
            xlabel('t (s)', 'FontSize', 10,'FontWeight', 'Bold')
            title('Angular Velocity Along Z', 'FontSize', 14,'FontWeight', 'Bold')
            grid minor

            % Make Directory and Save Plot if Save is Set
            if Save
                mkdir Plots

                try
                    rmdir Plots\AngularMotion
                catch
                end

                mkdir Plots\AngularMotion
                exportgraphics(obj.AngularMotionPlot, 'Plots/AngularMotion/AngularMotion.jpg');
                exportgraphics(obj.AngularMotionPlot, 'Plots/AngularMotion/AngularMotion.pdf');
            end
        end

        
        function Plot3D(obj, Iter, Save)

            if nargin == 2          % Set Default Arg
                Save = false;
            end

            % Create Plot Object
            obj.Viz3D = figure('Name', '3D Trajectory Plot', 'Units', 'normalized', 'OuterPosition', [0, 0, 1, 1]);
            obj.Viz3D.GraphicsSmoothing = 'on';
            obj.Viz3D.Color = [1, 1, 1];

            grid minor
            axis equal;
            title('3D Visualization of the Motion', 'FontSize', 16, 'FontWeight', 'Bold')
            xlabel('X', 'FontSize', 14, 'FontWeight', 'Bold')
            ylabel('Y', 'FontSize', 14, 'FontWeight', 'Bold')
            zlabel('Z', 'FontSize', 14, 'FontWeight', 'Bold')

            view(3)
            %             view(90, 0)
            %             view(0, 0)
            %             view(0, 90)
            view(-50, 10)

            % Setting Limits to the Plot
            RoomDims.X = [-2, 2];
            RoomDims.Y = [-2, 2];
            RoomDims.Z = [0, 3];

            % Set Room Dimension Limitations
            xlim(RoomDims.X)
            ylim(RoomDims.Y)
            zlim(RoomDims.Z)

            % Initial MotorCoordinates
            MotorCoord = [obj.ArmLen,           0, 0
                                   0,  obj.ArmLen, 0
                         -obj.ArmLen,           0, 0
                                   0, -obj.ArmLen, 0]';

            % Create Primary Plot Objects
            hold(gca, 'on');
            ArmX = plot3(gca, MotorCoord(1, [1, 3]) + obj.Pos(1, 1), ...
                         MotorCoord(2, [1, 3]) + obj.Pos(1, 2), ...
                         MotorCoord(3, [1, 3]) + obj.Pos(1, 3), ...
                         '-ro', 'MarkerSize', 10, 'MarkerEdgeColor', [0, 0, 0], 'MarkerFaceColor', [0.5, 0.5, 0.5], 'LineWidth', 1.1);

            ArmY = plot3(gca, MotorCoord(1, [2, 4]) + obj.Pos(1, 1), ...
                         MotorCoord(2, [2, 4]) + obj.Pos(1, 2), ...
                         MotorCoord(3, [2, 4]) + obj.Pos(1, 3), ...
                         '-bo', 'MarkerSize', 10, 'MarkerEdgeColor', [0, 0, 0], 'MarkerFaceColor', [0.5, 0.5, 0.5], 'LineWidth', 1.1);

            PayLoad = plot3(gca, obj.Pos(1, 1), ...
                            obj.Pos(1, 2), ...
                            obj.Pos(1, 3), ...
                            'sk', 'Linewidth', 2, 'MarkerSize', 4);

            Shadow = plot3(gca, obj.Pos(1, 1), ...
                           obj.Pos(1, 2), 0, ...
                           'Color', [0.5, 0.5, 0.5], ...
                           'Marker', 'x', 'MarkerSize', 15, 'LineWidth', 3);

            obj.Path = animatedline('Color', [1 0.5 0.5], 'LineWidth', 1.5);
            obj.Path.LineStyle = ':';
            obj.Path.DisplayName = 'Acutual Path';
            obj.Path.Annotation.LegendInformation.IconDisplayStyle = 'on';
            obj.Path.addpoints(obj.Pos(1, 1), obj.Pos(1, 2), obj.Pos(1, 3));

            hold(gca, 'off');

            if Save
                % Create a FileName Based on Time
                filename = ['Videos\Viz3D_', char(datetime('now', 'Format', 'hhmmss')), '.mp4'];

                % Create a Video Object and Configure it
                myVideo = VideoWriter(filename, 'MPEG-4');
                myVideo.FrameRate = 100;
                open(myVideo)
            end

            % Main Simulation Loop
            for i = 1:length(obj.Pos)

%                 subtitle(['Simulation Time: ', num2str(round(obj.t(i), 1)), ' / Iteration = ', num2str(Iter)],'FontWeight', 'Bold')

                % Extract Current Orientation and Position
                Orientation = obj.Orient(i, 1:3);
                Center      = obj.Pos(i, 1:3)';

                % Create a Temp Mat to Hold Rotated MotorCoords
                tmp = RotationMatrix(Orientation) * MotorCoord + Center;

                % Update Newly Calculated Position Coordinates
                set(ArmX, ...
                    'XData', tmp(1, [1, 3]), ...
                    'YData', tmp(2, [1, 3]), ...
                    'ZData', tmp(3, [1, 3]));

                set(ArmY, ...
                    'XData', tmp(1, [2, 4]), ...
                    'YData', tmp(2, [2, 4]), ...
                    'ZData', tmp(3, [2, 4]));

                set(PayLoad, ...
                    'XData', Center(1), ...
                    'YData', Center(2), ...
                    'ZData', Center(3));

                set(Shadow, ...
                    'XData', Center(1), ...
                    'YData', Center(2), ...
                    'ZData', 0);

                obj.Path.addpoints(Center(1), Center(2), Center(3));
              
                % drawnow expose update
                drawnow limitrate

                if Save
                    % Get Frame from the Plot and Write to Video Object
                    frame = getframe(obj.Viz3D);
                    writeVideo(myVideo, frame);
                end

                % Crash Detection And Message
                if    Center(3) < RoomDims.Z(1) || Center(3) > RoomDims.Z(2) ...
                        || Center(2) < RoomDims.Y(1) || Center(2) > RoomDims.Y(2) ...
                        || Center(1) < RoomDims.X(1) || Center(1) > RoomDims.X(2)

                    %                     msgbox('Crashed..!', 'Error', 'error');
                    break;
                end
            end

            if Save
                close(myVideo)
            end
        end
    end
end
