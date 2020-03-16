function opt = optimizeBoundaries(opt, opts, X)
    theta_x = optimvar('theta_x', 1, 1,'LowerBound',-90,'UpperBound',90); % 1x1
    theta_y = optimvar('theta_y', 1, 1,'LowerBound',-90,'UpperBound',90); % 1x1
    theta_z = optimvar('theta_z', 1, 1,'LowerBound',-90,'UpperBound',90); % 1x1
    T = optimvar('T', 1, 3); % 1x3
    prob = optimproblem;
    
    theta_x = 0;
    theta_y = 0;
    theta_z = 0;
    T = [-28, 7, 0];
    computeBoundariesCost(opts, X, theta_x, theta_y, theta_z, T);
    
    
    f = fcn2optimexpr(@computeBoundariesCost, opts, X, ...
                       theta_x, theta_y, theta_z, T);
    prob.Objective = f;
    x0.theta_x = 0;
    x0.theta_y = 0;
    x0.theta_z = 0;
    x0.T = 0;

%             options = optimoptions('fmincon', 'MaxIter',5e2,'Display','iter', 'TolX', 1e-6, 'TolFun', 1e-6, 'MaxFunctionEvaluations', 3e4);
    options = optimoptions('fmincon', 'MaxIter',5e2, 'Display','off', 'TolX', 1e-6, 'TolFun', 1e-6, 'MaxFunctionEvaluations', 3e4);
    max_trail = 5;
    num_tried = 1;
    status = 0;
    while status <=0 
        [sol, fval, status, ~] = solve(prob, x0, 'Options', options);
        if status <=0 
            warning("optimization failed")
        end
        num_tried = num_tried + 1;
        if (num_tried + 1 > max_trail)
            warning("tried too many time, optimization still failed, current status:")
            disp(status)
            break;
        end
    end
    R_final = rotx(sol.theta_x) * roty(sol.theta_y) * rotz(sol.theta_z);
    opt.H_opt = eye(4);
    opt.H_opt(1:3, 1:3) = R_final;
    opt.H_opt(1:3, 4) = sol.T;
    opt.opt_total_cost = fval;
    opt.boundaries
    opt.corners
end