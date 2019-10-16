function corners = checkHomogeneousCorners(corners)
    % assume corners is either 2xn, 3xn or 4xn
    % check 2D or 3D
    [n, ~] = size(corners);
    if n < 2
        disp("wrong usage")
    elseif n == 2
        corners = [corners; 
                   ones(1, size(corners, 2))];
    elseif mean(corners(3,:), 2) == 1 % 3D but not in homogeneous
        if size(corners, 1) ~= 4
        corners = [corners; 
                   ones(1, size(corners, 2))];
        end
    end
end