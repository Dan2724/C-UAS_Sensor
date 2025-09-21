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
    end

    methods
        function obj = simulator(map, uas, asset, dt)
            %SIMULATOR 
            obj.map = map;
            obj.UAS = uas;
            obj.asset = asset;
            obj.time = 0;
            obj.dt = dt;
        end

        function runSim(obj,interval)
            
            xPos0 = obj.UAS.entrance(1);
            yPos0 = obj.UAS.entrance(2);

            xtarget = obj.UAS.target(1);
            ytarget = obj.UAS.target(2);

            plot(xtarget, ytarget, 'Marker', '^', 'Color', 'b')

            xPos = xPos0;
            yPos = yPos0;
            targetUnitVector = (obj.UAS.target - obj.UAS.entrance) / norm(obj.UAS.target - obj.UAS.entrance);

            while xPos <= obj.map.size.horiz && yPos <= obj.map.size.vert
                xline(0,'k')
                yline(0,'k')
                xlim([0,obj.map.size.horiz])
                ylim([0,obj.map.size.vert])
                xPos = xPos0 + obj.UAS.speed*obj.time*targetUnitVector(1);
                yPos = yPos0 + obj.UAS.speed*obj.time*targetUnitVector(2);
                plot(xPos,yPos,'Marker','square','Color','r')
                dxPos = abs(obj.UAS.target(1) - xPos);
                dyPos = abs(obj.UAS.target(2) - yPos);
                if dxPos <= obj.UAS.speed*interval && dyPos <= obj.UAS.speed*interval
                    plot(xtarget, ytarget, 'Marker', 'x', 'Color', 'r')
                    break
                end
                obj.time = obj.time + interval;
            end
        end
    end
end