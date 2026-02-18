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
            if nargin < 3; nfzs       = polyshape.empty; end
            if nargin < 4; allSensors = sensor.empty;    end

            NFZ_ATTENUATION = 0.1;

            % Interference strength — how much one fully-overlapping neighbour
            % degrades detection in the shared zone.
            % k_int = 1 → neighbour at Pd=1 halves your probability there
            % k_int = 3 → neighbour at Pd=1 reduces your probability to 0.25
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

            % --- Spatially-varying co-channel interference ---
            % At each grid point, interference on THIS sensor equals the sum
            % of detection probabilities of all OTHER sensors at that point.
            % Where two sensors overlap heavily, each degrades the other.
            % The effect is visible as a dip/hole in the contour at the
            % overlap zone — exactly where the interference is strongest.
            if ~isempty(allSensors) && length(allSensors) > 1
                interferenceField = zeros(size(Pd));   % same size as grid

                for j = 1:length(allSensors)
                    % Skip self by location
                    if isequal(allSensors(j).location, obj.location)
                        continue;
                    end

                    % Compute neighbour j's detection probability at every
                    % grid point (logistic, same model assumed)
                    dx_j  = xg - allSensors(j).location(1);
                    dy_j  = yg - allSensors(j).location(2);
                    d_j   = sqrt(dx_j.^2 + dy_j.^2);
                    d50_j = allSensors(j).params.d50;
                    k_j   = allSensors(j).params.k;
                    Pd_j  = 1 ./ (1 + exp((d_j - d50_j) / k_j));

                    interferenceField = interferenceField + Pd_j;
                end

                % Attenuation grid: 1 where no interference, →0 in overlap zones
                interferenceAttenuation = 1 ./ (1 + k_int .* interferenceField);
                Pd = Pd .* interferenceAttenuation;
            end

            % --- NFZ line-of-sight attenuation ---
            if ~isempty(nfzs)
                sx    = obj.location(1);
                sy    = obj.location(2);
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

                        dgx   = xFlat - sx;
                        dgy   = yFlat - sy;
                        dex   = ex2 - ex1;
                        dey   = ey2 - ey1;
                        denom = dgx .* dey - dgy .* dex;
                        valid = abs(denom) > 1e-10;

                        t = ((ex1 - sx) .* dey - (ey1 - sy) .* dex) ./ denom;
                        u = ((ex1 - sx) .* dgy - (ey1 - sy) .* dgx) ./ denom;

                        hit     = valid & (t > 1e-6) & (t < 1) & (u >= 0) & (u <= 1);
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