classdef map < handle
    properties
        size
        UAS

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


            % xline(0)
            % yline(0)
        end
        function pullUAS(obj,inputUAS)
            obj.UAS = inputUAS;

        end
        function runSim(obj,interval)
            % xPos0 = obj.UAS.enterance;
            % yPos0 = obj.UAS.enterance;
            enterAngle = obj.UAS.entrance;
            
            if enterAngle == 0
                xPos0 = 0;
                yPos0 = obj.size.vert;

            elseif enterAngle > 0 && enterAngle < 45
                xPos0 = sind(enterAngle)*obj.size.horiz;
                yPos0 = obj.size.vert;

            elseif enterAngle == 45
                yPos0 = obj.size.vert;
                xPos0 = -obj.size.horiz;

            elseif enterAngle > 45 && enterAngle < 90
                xPos0 = obj.size.horiz;
                yPos0 = cosd(enterAngle)*obj.size.vert;

            elseif enterAngle == 90
                xPos0 = -obj.size.horiz;
                yPos0 = 0;

            end

            xPos = xPos0;
            yPos = yPos0;
            time = 0;

            while xPos < obj.size.horiz && yPos < obj.size.vert
                hold on
                xline(0,'k')
                yline(0,'k')
                xlim([0,obj.size.horiz])
                ylim([0,obj.size.vert])
                xPos = xPos0 + obj.UAS.speed*time*cosd(obj.UAS.heading);
                yPos = yPos0 + obj.UAS.speed*time*sind(obj.UAS.heading);
                plot(xPos,yPos,'Marker','square','Color','r')
                if xPos == obj.size.horiz
                    break
                elseif yPos == obj.size.vert
                    break
                end
                time = time + interval;
            end
        end
    end
end