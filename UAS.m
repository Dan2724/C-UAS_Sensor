classdef UAS < handle
    properties
        speed
        entrance
        target
        mode
        position
        targetUnitVector
        rangeHistory
    end
    methods
        function obj = UAS(speed, entrance, target, mode)
            obj.speed = speed;
            obj.entrance = entrance;
            obj.target = target;
            obj.mode = mode;
            obj.targetUnitVector = (obj.target - obj.entrance)/norm(obj.target - obj.entrance);
        end

        function obj = linearMotion(obj,xPos,yPos,time)
            obj.position.xPos = xPos + obj.speed*time*obj.targetUnitVector(1);
            obj.position.yPos = yPos + obj.speed*time*obj.targetUnitVector(2);

        end

        function obj = searchMotion(obj,xPos,yPos,time,assets,destroyedAssets,tick)
            for n = 1:length(assets)
                % Determine if the asset is in range of the UAS
                assetDistance(n) = norm([xPos, yPos] - assets(n).location);

            end

            assetDistance(destroyedAssets) = 1000;
            [distance, idx] = min(assetDistance);
            % obj.rangeHistory(tick + 1) = distance;
            % 
            % if obj.rangeHistory(end) > obj.rangeHistory(end-1)
            %     obj.speed = obj.speed - 1;
            % end



            if distance <= 20
                turnRadius = distance/2;
                angleVelo = obj.speed/turnRadius;
                if angleVelo > 1
                    angleVelo = 1; % sets max angular velocity of UAS
                end


                angle = angleVelo*time; % need to adust time fuction
                assetLocation = assets(idx).location - [obj.position.xPos, obj.position.yPos];

                targetAngle = acos(dot(obj.targetUnitVector,assetLocation)/(norm(obj.targetUnitVector)*norm(assetLocation))); % determines the angle between the movement vector and the vector to asset
                rotDir = cross([obj.targetUnitVector,0],[obj.position.xPos,obj.position.yPos,0]-[assets(idx).location,0]); % determines rotational direction
                if abs(targetAngle) > 0.1 % sets threshold for when the UAS will turn
                    if rotDir(3) < 0    % DCM based on rotation speed and direction
                        DCM = [cos(angle) -sin(angle);sin(angle) cos(angle)];
                    elseif rotDir(3) > 0
                        DCM = [cos(-angle) -sin(-angle);sin(-angle) cos(-angle)];
                    end

                    obj.targetUnitVector = (DCM*obj.targetUnitVector')';
                    % need to adjust the time function
                    obj.position.xPos = obj.position.xPos + obj.speed*time*obj.targetUnitVector(1);
                    obj.position.yPos = obj.position.yPos + obj.speed*time*obj.targetUnitVector(2);

                else
                    % need to adjust time function
                    obj.position.xPos = obj.position.xPos + obj.speed*time*obj.targetUnitVector(1);
                    obj.position.yPos = obj.position.yPos + obj.speed*time*obj.targetUnitVector(2);
                end

            else
                obj.position.xPos = xPos + obj.speed*time*obj.targetUnitVector(1);
                obj.position.yPos = yPos + obj.speed*time*obj.targetUnitVector(2);
                
            end

        end
    end
end
