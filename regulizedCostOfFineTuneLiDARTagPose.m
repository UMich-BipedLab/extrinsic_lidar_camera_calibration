function cost = regulizedCostOfFineTuneLiDARTagPose(theta_x, theta_y, theta_z, T, X, Y, H_LT, P, target_len)
    R = rotx(theta_x) * roty(theta_y) * rotz(theta_z);
    x_prime = R(1, :) * X(1:3, :) + T(1);
    y_prime = R(2, :) * X(1:3, :) + T(2);
    z_prime = R(3, :) * X(1:3, :) + T(3);
    
    H_LC = eye(4);
    H_LC(1:3,1:3) = R;
    H_LC(1:3, 4) = T;
    L_X_transformed = [x_prime; y_prime; z_prime; ones(size(x_prime))]; % transformed points in LiDAR frame
    C_X_transformed = P * L_X_transformed;
    C_X_transformed = C_X_transformed ./ C_X_transformed(3,:);
    X_at_lidar_frame = inv(H_LT) * inv(H_LC) * L_X_transformed;

    cost_x = 0;
    cost_y = 0;
    cost_z = 0;
    for i = 1:size(L_X_transformed, 2)
        cost_z = cost_z + checkCost(X_at_lidar_frame(3, i), -target_len/2, target_len/2);
        cost_y = cost_y + checkCost(X_at_lidar_frame(2, i), -target_len/2, target_len/2);
        cost_x = cost_x + checkCost(X_at_lidar_frame(1, i), -0.001, 0.001);
    end
    total_cost = cost_x + cost_y + cost_z;
%     cost = norm(C_X_transformed(1:2,:) - Y(1:2,:), 'fro') + 1e7*total_cost; %1e10
    cost = norm(C_X_transformed(1:2,:) - Y(1:2,:), 'fro') + 1e10*total_cost; %1e10
end