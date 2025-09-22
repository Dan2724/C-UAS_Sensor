function entrances = ingressPosns(N, L)
    % ingressPosns: Generate N evenly spaced entrance points
    % around the perimeter of an L x L square map
    %
    % Inputs:
    %   N - number of entrance points
    %   L - side length of the square map
    % Output:
    %   entrances - N x 2 matrix of [x, y] entrance coordinates

    % Total perimeter
    P = 4*L;

    % Evenly spaced distances along the perimeter
    s = linspace(0, P, N+1).';  
    s(end) = [];   % drop duplicate endpoint

    % Preallocate
    x = zeros(N,1);
    y = zeros(N,1);

    % Bottom edge (0 ≤ s < L)
    idx = s < L;
    x(idx) = s(idx);
    y(idx) = 0;

    % Right edge (L ≤ s < 2L)
    idx = (s >= L) & (s < 2*L);
    x(idx) = L;
    y(idx) = s(idx) - L;

    % Top edge (2L ≤ s < 3L)
    idx = (s >= 2*L) & (s < 3*L);
    x(idx) = L - (s(idx) - 2*L);
    y(idx) = L;

    % Left edge (3L ≤ s < 4L)
    idx = (s >= 3*L);
    x(idx) = 0;
    y(idx) = L - (s(idx) - 3*L);

    entrances = [x, y];
end