classdef AOR
    %AOR Object
    %   Define rectangle for AOR

    properties
        lowerLeftCorner
        upperRightCorner
    end

    methods
        function obj = AOR(lowerLeftCorner, upperRightCorner)
            %AOR Construct an instance of this class
            %   Detailed explanation goes here
            obj.lowerLeftCorner = lowerLeftCorner;
            obj.upperRightCorner = upperRightCorner;
        end
    end
end