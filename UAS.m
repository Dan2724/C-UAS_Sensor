classdef UAS < handle
    properties
        speed
        enterance
        heading
        
    end
    methods
        function obj = UAS(speed, enterance, heading)
            obj.speed = speed;
            obj.enterance = enterance;
            obj.heading = heading;
        end
    end
end