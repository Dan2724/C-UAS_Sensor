classdef simulator
    %SIMULATOR for C-UAS Sensor Placement
    %   This is the main simulator class for the UAS-Sensor Placement. This
    %   is where time will march, and from here all map updates are called.

    properties
        map
        UAS
        sensors
        asset
        time
        dt
        animate
    end

    methods
        function obj = simulator(map, uas, sensors, asset, dt, animate)
            %SIMULATOR 
            obj.map = map;
            obj.UAS = uas;
            obj.sensors = sensors;
            obj.asset = asset;
            obj.time = 0;
            obj.dt = dt;
            obj.animate = animate;
        end

        function result = runSim(obj)
            result = 0;

            xPos0 = obj.UAS.entrance(1);
            yPos0 = obj.UAS.entrance(2);

            xtarget = obj.UAS.target(1);
            ytarget = obj.UAS.target(2);

            xPos = xPos0;
            yPos = yPos0;
            targetUnitVector = (obj.UAS.target - obj.UAS.entrance) / norm(obj.UAS.target - obj.UAS.entrance);

            if obj.animate == true
                obj.map.displayMap

                % Plot asset
                plot(xtarget, ytarget, 'Marker', 'square', 'Color', 'b', 'MarkerSize', 10)

                % Plot sensors
                for i = 1:length(obj.sensors)
                    x = obj.sensors(i).location(1);
                    y = obj.sensors(i).location(2);
                    r = obj.sensors(i).range;

                    rectangle('Position',[x-r, y-r, 2*r, 2*r], ...
                        'Curvature', [1 1], ...
                        'FaceColor', 'b', ...
                        'EdgeColor', 'b', ...
                        'FaceAlpha', 0.05)
                    plot(x, y, 'o', 'Color', 'b')
                end

                %
                xlim([0,obj.map.size.horiz])
                ylim([0,obj.map.size.vert])

                % Set up animation
                UAVTrail = plot(NaN, NaN, 'Color', 'r');
                UAVHead = plot(NaN, NaN, 'Color', 'r', 'Marker', '^');
                UAVx = []; UAVy = [];
            end



            while xPos <= obj.map.size.horiz && yPos <= obj.map.size.vert
                % This while loop is a tad clunky and not necessary

                % Linear movement (Should be moved to separate function
                % based on UAV mode)
                xPos = xPos0 + obj.UAS.speed*obj.time*targetUnitVector(1);
                yPos = yPos0 + obj.UAS.speed*obj.time*targetUnitVector(2);
                dxPos = abs(xtarget - xPos);
                dyPos = abs(ytarget - yPos);
                
                % Determine state based on object collision
                state = obj.checkCollision(xPos, yPos, dxPos, dyPos);
                
                if obj.animate
                    UAVx(end+1) = xPos;
                    UAVy(end+1) = yPos;
                end

                if state == 1 % UAV sensed
                    if obj.animate
                        obj.animateUAVDestroyed([xPos, yPos])
                        obj.updateAnimation(UAVTrail, UAVHead, UAVx, UAVy)
                    end
                    result = 'UAV Destroyed';
                    break
                elseif state == 2 % Asset attacked
                    if obj.animate
                        obj.animateAssetDestroyed([xtarget, ytarget])
                        obj.updateAnimation(UAVTrail, UAVHead, UAVx, UAVy)
                    end
                    result = 'Asset Destroyed';
                    break
                else % UAV safe
                    if obj.animate
                        pause(obj.dt)
                        obj.updateAnimation(UAVTrail, UAVHead, UAVx, UAVy)
                    end
                end

                obj.time = obj.time + obj.dt; % Progress time
            end
        end

        function state = checkCollision(obj, x, y, dx, dy)
            % Sensor collision
            for i = 1:length(obj.sensors)
                r = [x, y] - obj.sensors(i).location;
                if norm(r) <= obj.sensors(i).range
                    state = 1;
                    return
                end
            end

            % Asset collision
            if dx <= obj.UAS.speed*obj.dt && dy <= obj.UAS.speed*obj.dt
                state = 2;
                return
            end
            
            % No collision
            state = 0;
            return
        end
        
    end

    methods(Static)
        % Main animation update (rename to updateUAVAnimation?)
        function updateAnimation(UAVTrail, UAVHead, UAVx, UAVy)
            set(UAVTrail, 'XData', UAVx, 'YData', UAVy)
            set(UAVHead, 'XData', UAVx(end), 'YData', UAVy(end))
            drawnow;
        end
        
        function animateAssetDestroyed(target)
            plot(target(1), target(2), 'Marker', 'x', 'Color', 'r', 'MarkerSize', 12)
        end

        function animateUAVDestroyed(position)
            plot(position(1), position(2), 'Marker', 'x', 'Color', 'g', 'MarkerSize', 12)
        end
    end
end