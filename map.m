classdef map < handle
    properties
        size
        UAS

    end
    methods
        function obj = map(vertical, horizontal)
            obj.size.vert = vertical;
            obj.size.horiz = horizontal;


        end
        function displayMap(obj)
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
    end
end