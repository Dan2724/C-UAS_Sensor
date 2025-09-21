classdef UAS < handle
    properties
        speed
        enterance
        heading
        
    end
    methods

        % All this does is stores the set parameters in the UAS properties
        function obj = UAS(speed, enterance, heading)
            obj.speed = speed;
            obj.enterance = enterance;
            obj.heading = heading;
        end
    end
end