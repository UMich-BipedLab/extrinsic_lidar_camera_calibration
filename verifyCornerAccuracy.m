function cost = verifyCornerAccuracy(X_verification, Y_verification, P)
    C_X_transformed = P * X_verification;
    C_X_transformed = C_X_transformed ./ C_X_transformed(3,:);
    cost = (norm(C_X_transformed(1:2,:) - Y_verification(1:2,:), 'fro'))^2;
end
