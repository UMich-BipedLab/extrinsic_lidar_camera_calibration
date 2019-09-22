function cost = cost4Points(theta_x, theta_y, theta_z, T, X, Y, intrinsic)
    R = rotx(theta_x) * roty(theta_y) * rotz(theta_z);

%         function cost = cost4Points(app, xi, X, Y, intrinsic)
%             H = expm(hat(app, xi'));
%             R = H(1:3, 1:3);
%             T = H(1:3, 4);
    x_prime = R(1, :) * X(1:3, :) + T(1);
    y_prime = R(2, :) * X(1:3, :) + T(2);
    z_prime = R(3, :) * X(1:3, :) + T(3);
    L_X_transformed = [x_prime; y_prime; z_prime; ones(size(x_prime))]; % transformed points in LiDAR frame
    C_X_transformed = intrinsic * [eye(3) zeros(3,1)] * L_X_transformed;
    C_X_transformed = C_X_transformed ./ C_X_transformed(3,:);
    cost = norm(C_X_transformed(1:2,:) - Y(1:2,:), 'fro');
end