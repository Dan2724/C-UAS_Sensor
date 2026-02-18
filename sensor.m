classdef sensor
    %SENSOR Class for sensors
    %   Detailed explanation goes here

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
            %SENSOR Construct an instance of this class
            %   Detailed explanation goes here
            obj.location = location;
            obj.range = range;
            obj.model = model;
            obj.params = params;
            obj.peakGain = peakGain;
            obj.boresight = boresight;
            obj.beamwidth = beamwidth;
        end

        function obj = createAttenuationMap(obj)
            
        end

        function [xg, yg, P] = createSensorContours(obj, mapSize)
            [xg, yg] = meshgrid(0:1:mapSize.horiz, 0:1:mapSize.vert);
            dx = xg - obj.location(1);
            dy = yg - obj.location(2);
            d = sqrt(dx.^2 + dy.^2);
            theta = atan2d(yg - obj.location(2), xg - obj.location(1));
            

            

            switch lower(obj.model)
                case 'logistic'
                    d50 = obj.params.d50;
                    k = obj.params.k;
                    Pd = 1 ./ (1 + exp((d - d50)/k));
                otherwise
                    error("Unknown sensor model")
            end

            bw = obj.beamwidth;
            if bw >= 360
                gain = ones(size(d));
            else
                % angular separation
                da = angdiff_deg(theta, s.boresight); % use helper below
                % normalize to half-width: 0 at boresight, 1 at half-angle
                % use Gaussian-like or cos^n shape. We'll use Gaussian:
                sigma_ang = bw/2 / 1.177; % approx convert half-power to sigma
                gain = exp(-0.5 * (da./sigma_ang).^2);
            end

            P = obj.peakGain .* Pd .* gain;
        end
    end
end