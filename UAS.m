classdef UAS < handle
    properties
        speed
        target
        mode
        position
        targetUnitVector
        rangeHistory

        obstacles

    end

    methods
        function obj = UAS(speed, entrance, target, mode)
            obj.speed = speed;
            obj.position = entrance;
            obj.target = target;
            obj.mode = mode;
            obj.targetUnitVector = (obj.target - obj.position)/norm(obj.target - obj.position);

        end

        function obj = linearMotion(obj,time)
            obj.position = obj.position + obj.speed*time*obj.targetUnitVector;

        end

        function obj = searchMotion(obj,time,assets,destroyedAssets,tick)

            obj.obstacles.assets = assets;

            if ~isempty(destroyedAssets)
                obj.obstacles.assets(destroyedAssets) = [];

            end
              
            for n = 1:length(obj.obstacles.assets)
                assetDistance(n) = norm(obj.position - obj.obstacles.assets(n).location);

            end

            [assetDistance, assetNumber] = min(assetDistance);

            if assetDistance <= 20
                obj.assetFound(assetDistance,assetNumber,time)


            else
                obj.position = obj.position + obj.speed*time*obj.targetUnitVector;

            end
        end

        function assetFound(obj,assetDistance,assetNumber,time)

            turnRadius = assetDistance/2;
            angleVelo = obj.speed/turnRadius;
            if angleVelo > 1
                angleVelo = 1;
            end

            angle = angleVelo*time;
            assetLocation = obj.obstacles.assets(assetNumber).location - obj.position;

            targetAngle = acos(dot(obj.targetUnitVector,assetLocation)/(norm(obj.targetUnitVector)*norm(assetLocation)));
            rotDir = cross([obj.targetUnitVector,0],[obj.position,0]-[obj.obstacles.assets(assetNumber).location,0]);

            if abs(targetAngle) > 0.1
                if rotDir(3) < 0
                    DCM = [cos(angle) -sin(angle);sin(angle) cos(angle)];
                elseif rotDir(3) > 0
                    DCM = [cos(-angle) -sin(-angle);sin(-angle) cos(-angle)];
                end

                obj.targetUnitVector = (DCM*obj.targetUnitVector')';
                obj.position = obj.position + obj.speed*time*obj.targetUnitVector;

            else
                obj.position = obj.position + obj.speed*time*obj.targetUnitVector;

            end

        end

        function turnMotion(obj)

        end
    end
end
