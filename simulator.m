classdef simulator
    %SIMULATOR for C-UAS Sensor Placement
    %   This is the main simulator class for the UAS-Sensor Placement. This
    %   is where time will march, and from here all map updates are called.

    properties
        map
        UAS
        asset
        time
        dt
        animate
    end

    methods
        function obj = simulator(map, uas, asset, dt, animate)
            %SIMULATOR 
            obj.map = map;
            obj.UAS = uas;
            obj.asset = asset;
            obj.time = 0;
            obj.dt = dt;
            obj.animate = animate;
        end

        function success = runSim(obj)
            success = 0;

            xPos0 = obj.UAS.entrance(1);
            yPos0 = obj.UAS.entrance(2);

            xtarget = obj.UAS.target(1);
            ytarget = obj.UAS.target(2);

            xPos = xPos0;
            yPos = yPos0;
            targetUnitVector = (obj.UAS.target - obj.UAS.entrance) / norm(obj.UAS.target - obj.UAS.entrance);

            if obj.animate == true
                obj.map.displayMap
                plot(xtarget, ytarget, 'Marker', '^', 'Color', 'b')
                UAVPath = animatedline('Color', 'r');
            end

            while xPos <= obj.map.size.horiz && yPos <= obj.map.size.vert
                xline(0,'k')
                yline(0,'k')
                xlim([0,obj.map.size.horiz])
                ylim([0,obj.map.size.vert])
                xPos = xPos0 + obj.UAS.speed*obj.time*targetUnitVector(1);
                yPos = yPos0 + obj.UAS.speed*obj.time*targetUnitVector(2);
                dxPos = abs(xtarget - xPos);
                dyPos = abs(ytarget - yPos);
                if dxPos <= obj.UAS.speed*obj.dt && dyPos <= obj.UAS.speed*obj.dt
                    if obj.animate == true
                        obj.updateAnimation(UAVPath, xtarget, ytarget)
                        obj.animateDestroyed([xtarget, ytarget])
                    end
                    success = 1;
                    break
                end
                
                obj.updateAnimation(UAVPath, xPos, yPos)

                obj.time = obj.time + obj.dt;
            end
        end

        function checkCollision(obj)

        end
    end
    methods(Static)
        function updateAnimation(UAVPath, xPos, yPos)
            addpoints(UAVPath, xPos,yPos);
            drawnow;
        end

        function animateDestroyed(target)
            plot(target(1), target(2), 'Marker', 'x', 'Color', 'r', 'MarkerSize', 12)
        end
    end
end