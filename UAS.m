classdef UAS < handle
    properties
        speed
        entrance
        target
        
    end
    methods
        function obj = UAS(speed, entrance, target)
            obj.speed = speed;
            obj.entrance = entrance;
            obj.target = target;
        end
    end
end