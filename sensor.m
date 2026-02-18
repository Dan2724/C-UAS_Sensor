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

        function [xg, yg, P] = createSensorContours(obj, mapSize, nfzs)
            % createSensorContours  Compute per-cell detection probability.
            %
            %   [xg, yg, P] = createSensorContours(obj, mapSize)
            %   [xg, yg, P] = createSensorContours(obj, mapSize, nfzs)
            %
            %   nfzs  – array of polyshape objects (NFZs).  Any grid cell
            %           whose line-of-sight to the sensor is blocked by at
            %           least one NFZ has its probability multiplied by
            %           NFZ_ATTENUATION (default 0.1), modelling signal
            %           blockage by buildings / dense vegetation.

            if nargin < 3
                nfzs = polyshape.empty;
            end

            % Attenuation factor applied when LOS passes through an NFZ.
            NFZ_ATTENUATION = 0.1;

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

            % --- NFZ line-of-sight attenuation ---
            % For each grid point, test whether the straight line from the
            % sensor to that point intersects any NFZ polygon edge.  This is
            % a simple "is the path blocked?" check — no exponential decay,
            % just a binary multiply by NFZ_ATTENUATION.
            if ~isempty(nfzs)
                sx = obj.location(1);
                sy = obj.location(2);

                % Flatten grid so we can loop efficiently
                rows = size(xg, 1);
                cols = size(xg, 2);
                xFlat = xg(:);
                yFlat = yg(:);
                blocked = false(numel(xFlat), 1);

                for n = 1:length(nfzs)
                    % polyshape vertices for this NFZ
                    vx = nfzs(n).Vertices(:, 1);
                    vy = nfzs(n).Vertices(:, 2);
                    numV = length(vx);

                    % Edges of the polygon: (vx(j),vy(j)) -> (vx(j+1),vy(j+1))
                    for j = 1:numV
                        j2  = mod(j, numV) + 1;
                        ex1 = vx(j);  ey1 = vy(j);
                        ex2 = vx(j2); ey2 = vy(j2);

                        % Segment-segment intersection:
                        % Ray from (sx,sy) to (gx,gy); edge from (ex1,ey1) to (ex2,ey2).
                        % Use parametric form and solve 2x2 system.
                        % d_ray = [gx-sx, gy-sy], d_edge = [ex2-ex1, ey2-ey1]
                        % t in [0,1] => hit on ray, u in [0,1] => hit on edge.
                        % Exclude t=0 (the sensor itself) with t > 1e-6.

                        dgx = xFlat - sx;
                        dgy = yFlat - sy;
                        dex = ex2 - ex1;
                        dey = ey2 - ey1;

                        denom = dgx .* dey - dgy .* dex;  % cross product

                        % Parallel rays — no intersection
                        valid = abs(denom) > 1e-10;

                        t = ((ex1 - sx) .* dey - (ey1 - sy) .* dex) ./ denom;
                        u = ((ex1 - sx) .* dgy - (ey1 - sy) .* dgx) ./ denom;

                        hit = valid & (t > 1e-6) & (t < 1) & (u >= 0) & (u <= 1);
                        blocked = blocked | hit;
                    end
                end

                % Apply attenuation mask
                attenuationMask = ones(numel(xFlat), 1);
                attenuationMask(blocked) = NFZ_ATTENUATION;
                attenuationMask = reshape(attenuationMask, rows, cols);

                Pd = Pd .* attenuationMask;
            end

            P = obj.peakGain .* Pd .* gain;
        end
    end
end