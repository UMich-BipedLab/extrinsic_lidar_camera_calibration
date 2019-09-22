function [H_LC, P, cost] = optimize4Points(X, Y, intrinsic, display)
    theta_x = optimvar('theta_x', 1, 1,'LowerBound',-180,'UpperBound',180); % 1x1
    theta_y = optimvar('theta_y', 1, 1,'LowerBound',-180,'UpperBound',180); % 1x1
    theta_z = optimvar('theta_z', 1, 1,'LowerBound',-180,'UpperBound',180); % 1x1
    T = optimvar('T', 1, 3,'LowerBound',-0.5,'UpperBound',0.5);
    prob = optimproblem;
    f = fcn2optimexpr(@cost4Points, theta_x, theta_y, theta_z, T, X, Y, intrinsic);
    prob.Objective = f;
    x0.theta_x = 90;
    x0.theta_y = 0;
    x0.theta_z = 90;
    x0.T = [0 0 0];
    options = optimoptions('fmincon', 'MaxIter',5e2, 'TolX', 1e-12, 'Display','off', 'FunctionTolerance', 1e-8, 'MaxFunctionEvaluations', 3e4);

    [sol, fval, ~, ~] = solve(prob, x0, 'Options', options);
    R_final = rotx(sol.theta_x) * roty(sol.theta_y) * rotz(sol.theta_z);
    H_LC = eye(4);
    H_LC(1:3, 1:3) = R_final;
    H_LC(1:3, 4) = sol.T';
    
    if display
        disp('H_LC: ')
        disp('     R:')
        disp(H_LC(1:3, 1:3))
        disp('     T:')
        disp(-inv(H_LC(1:3, 1:3))*H_LC(1:3, 4))
        fval
    end
    P = intrinsic * [eye(3) zeros(3,1)] * H_LC;
    cost = fval;
end