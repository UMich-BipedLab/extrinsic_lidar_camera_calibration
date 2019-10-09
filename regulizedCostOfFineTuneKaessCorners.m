function cost = regulizedCostOfFineTuneKaessCorners(theta_x, theta_y, theta_z, T, X, Y, edges_points, P)
    R = rotx(theta_x) * roty(theta_y) * rotz(theta_z);
    x_prime = R(1, :) * X(1:3, :) + T(1);
    y_prime = R(2, :) * X(1:3, :) + T(2);
    z_prime = R(3, :) * X(1:3, :) + T(3);

    L_X_transformed = [x_prime; y_prime; z_prime; ones(size(x_prime))]; % transformed points in LiDAR frame
    C_X_transformed = P * L_X_transformed;
    C_X_transformed = C_X_transformed ./ C_X_transformed(3,:);
    cost_LU = pointToLineDistance([edges_points.LU]', L_X_transformed(1:3, 1)', L_X_transformed(1:3, 2)');
    cost_LL = pointToLineDistance([edges_points.LL]', L_X_transformed(1:3, 2)', L_X_transformed(1:3, 4)');
    cost_RU = pointToLineDistance([edges_points.RU]', L_X_transformed(1:3, 1)', L_X_transformed(1:3, 3)');
    cost_RL = pointToLineDistance([edges_points.RL]', L_X_transformed(1:3, 3)', L_X_transformed(1:3, 4)');
    total_cost = sum(cost_LU) + sum(cost_LL) + sum(cost_RU) + sum(cost_RL); % four sides of the squares


    cost = norm(C_X_transformed(1:2,:) - Y(1:2,:), 'fro') + 0.005*total_cost;
end