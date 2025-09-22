classdef UAS < handle
    properties
        speed
        entrance
        target
        mode
        position
    end
    methods
        function obj = UAS(speed, entrance, target, mode)
            obj.speed = speed;
            obj.entrance = entrance;
            obj.target = target;
            obj.mode = mode;
        end

        function obj = linearMotion(obj,xPos0,yPos0,xPos,yPos,xtarget,ytarget,time,targetUnitVector)
            obj.position.xPos = xPos0 + obj.speed*time*targetUnitVector(1);
            obj.position.yPos = yPos0 + obj.speed*time*targetUnitVector(2);
            obj.position.dxPos = abs(xtarget - xPos);
            obj.position.dyPos = abs(ytarget - yPos);
        end

        function obj = searchMotion(obj,xPos0,yPos0,xPos,yPos,xtarget,ytarget,time,targetUnitVector,assets)
            for n = 1:length(assets)
                assetDistance(n) = norm([xPos, yPos] - assets(n).location);



            end
            assetDistance <= 20

        end
    end
end
