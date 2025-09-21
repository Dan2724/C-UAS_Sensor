classdef map < handle
    properties
        size
        UAS
        numAsset

    end
    methods
        function obj = map(size)
            obj.size.vert = size;
            obj.size.horiz = size;
        

        end
        function displayMap(obj)
            if ~isempty(obj.UAS)
                plot(0,obj.UAS.entrance,'Marker','square')
            end
            xlim([0,obj.size.horiz])
            ylim([0,obj.size.vert])
            grid on
            hold on
            axis equal
            title("UAS Simulation")
            xlabel("X (m)")
            ylabel("Y (m)")
        end
        function pullUAS(obj,inputUAS)
            obj.UAS = inputUAS;

        end
        function runSim(obj,interval)
            
            xPos0 = obj.UAS.entrance(1);
            yPos0 = obj.UAS.entrance(2);

            xtarget = obj.UAS.target(1);
            ytarget = obj.UAS.target(2);

            plot(xtarget, ytarget, 'Marker', '^', 'Color', 'b')
            hold on

            xPos = xPos0;
            yPos = yPos0;
            targetUnitVector = (obj.UAS.target - obj.UAS.entrance) / norm(obj.UAS.target - obj.UAS.entrance);
            time = 0;

            while xPos <= obj.size.horiz && yPos <= obj.size.vert
                hold on
                xline(0,'k')
                yline(0,'k')
                xlim([0,obj.size.horiz])
                ylim([0,obj.size.vert])
                xPos = xPos0 + obj.UAS.speed*time*targetUnitVector(1);
                yPos = yPos0 + obj.UAS.speed*time*targetUnitVector(2);
                plot(xPos,yPos,'Marker','square','Color','r')
                dxPos = abs(obj.UAS.target(1) - xPos);
                dyPos = abs(obj.UAS.target(2) - yPos);
                if dxPos <= obj.UAS.speed*interval && dyPos <= obj.UAS.speed*interval;
                    break
                end
                time = time + interval;
            end
        end
    end
end