classdef assets < handle
    properties
        location1
    
    end
    methods
        function obj = assets(location1)
            obj.A1 = [location1(1), location1(2)];
        end
    end
end
        