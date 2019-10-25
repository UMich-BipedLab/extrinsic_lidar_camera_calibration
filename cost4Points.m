function cost = cost4Points(theta_x, theta_y, theta_z, T, X, Y, intrinsic)
    

%         function cost = cost4Points(app, xi, X, Y, intrinsic)
%             H = expm(hat(app, xi'));
%             R = H(1:3, 1:3);
%             T = H(1:3, 4);
    H = eye(4);
    H(1:3,1:3) = rotx(theta_x) * roty(theta_y) * rotz(theta_z);
    H(1:3,4) = T';
    P = intrinsic * [eye(3) zeros(3,1)] * H;
    cost = verifyCornerAccuracy(X, Y, P);
end