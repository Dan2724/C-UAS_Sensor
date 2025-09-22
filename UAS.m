classdef UAS < handle
    properties
        speed
        entrance
        target
        mode
        position
        xPos
        yPos
        dxPos
        dyPos
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

        function obj = searchMotion(obj)

        end
    end
end