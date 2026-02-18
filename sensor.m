classdef sensor
    %SENSOR Class for sensors

    properties
        location
        range
        model
        params
        peakGain
        boresight
        beamwidth
        xg
        yg
        P
    end

    methods
        function obj = sensor(location, range, model, params, peakGain, boresight, beamwidth)
            obj.location  = location;
            obj.range     = range;
            obj.model     = model;
            obj.params    = params;
            obj.peakGain  = peakGain;
            obj.boresight = boresight;
            obj.beamwidth = beamwidth;
        end

        function [xg, yg, P] = createSensorContours(obj, mapSize, nfzs, allSensors)
            % createSensorContours  Compute per-cell detection probability.
            %
            %   [xg, yg, P] = createSensorContours(obj, mapSize)
            %   [xg, yg, P] = createSensorContours(obj, mapSize, nfzs)
            %   [xg, yg, P] = createSensorContours(obj, mapSize, nfzs, allSensors)
            %
            %   nfzs       – polyshape array.  Grid cells whose LOS to the
            %                sensor is blocked are attenuated by NFZ_ATTENUATION.
            %
            %   allSensors – array of sensor objects (including this one).
            %                Nearby sensors cause co-channel interference that
            %                reduces detection probability.

            if nargin < 3; nfzs       = polyshape.empty; end
            if nargin < 4; allSensors = sensor.empty;    end

            NFZ_ATTENUATION = 0.1;  % probability multiplier when LOS blocked by NFZ

            % ------------------------------------------------------------------
            % Interference parameters
            %   r_int  – distance at which interference is ~60 % of maximum
            %            (set equal to d50 so sensors within detection range
            %             of each other interfere significantly)
            %   k_int  – strength: 1.0 means one full-power neighbour at
            %            distance 0 halves the detection probability
            % ------------------------------------------------------------------
            r_int = obj.params.d50;
            k_int = 1.0;

            [xg, yg] = meshgrid(0:1:mapSize.horiz, 0:1:mapSize.vert);
            dx = xg - obj.location(1);
            dy = yg - obj.location(2);
            d  = sqrt(dx.^2 + dy.^2);

            % --- Base detection probability (logistic) ---
            switch lower(obj.model)
                case 'logistic'
                    d50 = obj.params.d50;
                    k   = obj.params.k;
                    Pd  = 1 ./ (1 + exp((d - d50) / k));
                otherwise
                    error('Unknown sensor model: %s', obj.model);
            end

            % --- Beam pattern gain ---
            theta = atan2d(dy, dx);
            bw    = obj.beamwidth;
            if bw >= 360
                gain = ones(size(d));
            else
                da        = mod(theta - obj.boresight + 180, 360) - 180;
                sigma_ang = (bw / 2) / 1.177;
                gain      = exp(-0.5 * (da ./ sigma_ang).^2);
            end

            % --- Co-channel interference from neighbouring sensors ---
            % For each other sensor j, compute how much it interferes with
            % this sensor at every grid point.  Interference is strongest
            % when j is physically close to this sensor (small d_ij) and
            % falls off as a Gaussian in sensor-to-sensor distance.
            % The result is a scalar per sensor pair — it does not depend on
            % the grid point — so we compute it once and apply uniformly.
            if ~isempty(allSensors) && length(allSensors) > 1
                totalInterference = 0;
                for j = 1:length(allSensors)
                    % Skip self
                    if isequal(allSensors(j).location, obj.location)
                        continue;
                    end
                    d_ij = norm(allSensors(j).location - obj.location);
                    totalInterference = totalInterference + ...
                        exp(-(d_ij^2) / (2 * r_int^2));
                end
                % attenuation in (0, 1]: 1 = no interference, →0 = full interference
                interferenceAttenuation = 1 / (1 + k_int * totalInterference);
                Pd = Pd .* interferenceAttenuation;
            end

            % --- NFZ line-of-sight attenuation ---
            if ~isempty(nfzs)
                sx = obj.location(1);
                sy = obj.location(2);

                rows  = size(xg, 1);
                cols  = size(xg, 2);
                xFlat = xg(:);
                yFlat = yg(:);
                blocked = false(numel(xFlat), 1);

                for n = 1:length(nfzs)
                    vx   = nfzs(n).Vertices(:, 1);
                    vy   = nfzs(n).Vertices(:, 2);
                    numV = length(vx);

                    for j = 1:numV
                        j2  = mod(j, numV) + 1;
                        ex1 = vx(j);  ey1 = vy(j);
                        ex2 = vx(j2); ey2 = vy(j2);

                        dgx = xFlat - sx;
                        dgy = yFlat - sy;
                        dex = ex2 - ex1;
                        dey = ey2 - ey1;

                        denom = dgx .* dey - dgy .* dex;
                        valid = abs(denom) > 1e-10;

                        t = ((ex1 - sx) .* dey - (ey1 - sy) .* dex) ./ denom;
                        u = ((ex1 - sx) .* dgy - (ey1 - sy) .* dgx) ./ denom;

                        hit = valid & (t > 1e-6) & (t < 1) & (u >= 0) & (u <= 1);
                        blocked = blocked | hit;
                    end
                end

                attenuationMask = ones(numel(xFlat), 1);
                attenuationMask(blocked) = NFZ_ATTENUATION;
                attenuationMask = reshape(attenuationMask, rows, cols);

                Pd = Pd .* attenuationMask;
            end

            P = obj.peakGain .* Pd .* gain;
        end
    end
end