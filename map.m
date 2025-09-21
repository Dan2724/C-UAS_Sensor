classdef map < handle
    properties
        size
        UAS

    end
    methods
        function obj = map(size)
            % sets map size with input (havent figured out how to full
            % integrate this yet
            obj.size.vert = size;
            obj.size.horiz = size;


        end
        function displayMap(obj)
            % if statement determines if there is a UAS object in the
            % properties
            if ~isempty(obj.UAS)
                plot(0,obj.UAS.enterance,'Marker','square')
            end
            xlim([0,obj.size.horiz])
            ylim([0,obj.size.vert])


            % xline(0)
            % yline(0)
        end
        function pullUAS(obj,inputUAS)
            % Inputs UAS object into the map object
            obj.UAS = inputUAS;

        end
        function runSim(obj,interval)
            % xPos0 = obj.UAS.enterance;
            % yPos0 = obj.UAS.enterance;
            enterAngle = obj.UAS.enterance; % creates a temporary angle called enter angle for the motion of the UAS

            % If statements determine where to put the enterance location
            % of the UAS on the map, starts at the edge if statements are
            % necessary (for now) due to angle at 45 degree intervals
            % change which axis is maxed out
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

            % sets initial x and y position for the while loop to run
            xPos = xPos0;
            yPos = yPos0;
            time = 0;

            % while loop runs as long as xPos and yPos are both within the
            % bounds of the map set when it was created
            while xPos < obj.size.horiz && yPos < obj.size.vert
                hold on

                xline(0,'k')
                yline(0,'k')
                xlim([0,obj.size.horiz])
                ylim([0,obj.size.vert])

                % calculates new position for the UAS based on interval
                % change

                xPos = xPos0 + obj.UAS.speed*time*cosd(obj.UAS.heading);
                yPos = yPos0 + obj.UAS.speed*time*sind(obj.UAS.heading);
                plot(xPos,yPos,'Marker','square','Color','r')

                % breaks loop as when the UAS reaches the edge of the map
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