function total_cost = computeConstraintCustomizedCost(X, theta_x, theta_y, theta_z, T, target_size, box_width)
    cost_x = 0;
    cost_y = 0;
    cost_z = 0;
    R = rotx(theta_x) * roty(theta_y) * rotz(theta_z);
    x_prime = R(1, :) * X(1:3, :) + T(1);
    y_prime = R(2, :) * X(1:3, :) + T(2);
    z_prime = R(3, :) * X(1:3, :) + T(3);
    for i = 1:size(X, 2)
        cost_z = cost_z + checkCost(z_prime(i), -target_size/2, target_size/2);
        cost_y = cost_y + checkCost(y_prime(i), -target_size/2, target_size/2);
        cost_x = cost_x + checkCost(x_prime(i), -box_width, box_width);
    end
%     total_cost = sqrt(cost_x + cost_y + cost_z);
    total_cost = cost_x + cost_y + cost_z;
end