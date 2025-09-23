classdef UAS < handle
    properties
        speed
        entrance
        target
        mode
        position
        targetUnitVector
    end
    methods
        function obj = UAS(speed, entrance, target, mode)
            obj.speed = speed;
            obj.entrance = entrance;
            obj.target = target;
            obj.mode = mode;
        end

        function obj = linearMotion(obj,xPos0,yPos0,xPos,yPos,xtarget,ytarget,time)
            obj.position.xPos = xPos0 + obj.speed*time*obj.targetUnitVector(1);
            obj.position.yPos = yPos0 + obj.speed*time*obj.targetUnitVector(2);
            obj.position.dxPos = abs(xtarget - xPos);
            obj.position.dyPos = abs(ytarget - yPos);
        end

        function obj = searchMotion(obj,xPos0,yPos0,xPos,yPos,xtarget,ytarget,time,assets)
            for n = 1:length(assets)                                        % Determine if the asset is in range of the UAS
                assetDistance(n) = norm([xPos, yPos] - assets(n).location);

            end
            [distance, idx] = min(assetDistance);                           % Determine which asset is closer if more than one is in range

            if distance <= 20
                turnRadius = 5;
                angleVelo = obj.speed/turnRadius;
                angle = angleVelo*0.05;

                targetAngle = atan2(assets(idx).location(1) - obj.position.xPos,assets(idx).location(2) - obj.position.yPos);
                
                if targetAngle < 0
                    DCM = [cos(angle) -sin(angle);sin(angle) cos(angle)];
                elseif targetAngle > 0
                    DCM = [cos(-angle) -sin(-angle);sin(-angle) cos(-angle)];
                end

                obj.targetUnitVector = DCM*obj.targetUnitVector;
                obj.position.xPos = obj.position.xPos + obj.speed*0.05*obj.targetUnitVector(1);
                obj.position.yPos = obj.position.yPos + obj.speed*0.05*obj.targetUnitVector(2);

            else
                obj.position.xPos = xPos + obj.speed*0.05*obj.targetUnitVector(1);
                obj.position.yPos = yPos + obj.speed*0.05*obj.targetUnitVector(2);
                
            end

        end
    end
end
