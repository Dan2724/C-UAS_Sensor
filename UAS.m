classdef UAS < handle
    properties
        speed
        entrance
        heading
        
    end
    methods
        function obj = UAS(speed, entrance, heading)
            obj.speed = speed;
            obj.entrance = entrance;
            obj.heading = heading;
        end
    end
end