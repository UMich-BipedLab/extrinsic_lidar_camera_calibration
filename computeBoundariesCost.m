function cost = computeBoundariesCost(opts, X, theta_x, theta_y, theta_z, T)
    H = eye(4);
    H(1:3,1:3) = rotx(theta_x) * roty(theta_y) * rotz(theta_z);
    H(1:3,4) = T';
    X_transformed = H * X;
    cost_x_pos = sum(X_transformed(1, X_transformed(1,:)>0));
    cost_x_neg = sum(X_transformed(1, X_transformed(1,:)<0));
    
    cost_y_pos = sum(X_transformed(2, X_transformed(2,:)>0));
    cost_y_neg = sum(X_transformed(2, X_transformed(2,:)<0));
    
    cost_z_pos = sum(X_transformed(3, X_transformed(3,:)>0));
    cost_z_neg = sum(X_transformed(3, X_transformed(3,:)<0));
    figure(1)
    hold on
    scatter3(X_transformed(1,:), X_transformed(2,:), X_transformed(3,:), 'r.')
end