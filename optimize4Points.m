function [H_LC, P, cost] = optimize4Points(opt, X, Y, intrinsic, display)

    for i = 1:1
        theta_x = optimvar('theta_x', 1, 1,'LowerBound',-180,'UpperBound',180); % 1x1
        theta_y = optimvar('theta_y', 1, 1,'LowerBound',-180,'UpperBound',180); % 1x1
        theta_z = optimvar('theta_z', 1, 1,'LowerBound',-180,'UpperBound',180); % 1x1
        T = optimvar('T', 1, 3,'LowerBound',-0.5,'UpperBound',0.5);
        prob = optimproblem;
        f = fcn2optimexpr(@cost4Points, theta_x, theta_y, theta_z, T, X, Y, intrinsic);
        prob.Objective = f;
        noise = (rand([1,6]) - 0.5) * 2; % -1 to 1
        if i==1
            x0.theta_x = opt(1);
            x0.theta_y = opt(2) ;
            x0.theta_z = opt(3);
            x0.T = [0 0 0];
        elseif i==2
            x0.theta_x = 82.0122;
            x0.theta_y = -0.0192 ;
            x0.theta_z = 87.7953;
            x0.T = [0.0228
                   -0.2070
                   -0.0783];
        else
            x0.theta_x = opt(1) + noise(1) * 10;
            x0.theta_y = opt(2) + noise(2) * 10;
            x0.theta_z = opt(3) + noise(3) * 10;
            x0.T = [0 0 0] + noise(4:6) * 0.1;
        end
        
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
        results(i).cost = fval;
        results(i).H = H_LC;
    end
    disp("std:")
    disp(std([results(:).cost]))
    [min_cost, k] = min([results(:).cost]);
    disp("minimum cost:")
    disp(min_cost)
    disp('H_LC: ')
    disp(' R:')
    disp(results(k).H(1:3, 1:3))
    disp(' RPY (XYZ):')
    disp(rad2deg(rotm2eul(results(k).H(1:3, 1:3), "XYZ")))
    disp(' T:')
    disp(-inv(results(k).H(1:3, 1:3))*results(k).H(1:3, 4))
    [max_cost, j] = max([results(:).cost]);
    disp("max cost:")
    disp(max_cost)
    disp('H_LC: ')
    disp(' R:')
    disp(results(j).H(1:3, 1:3))
    disp(' RPY (XYZ):')
    disp(rad2deg(rotm2eul(results(j).H(1:3, 1:3), "XYZ")))
    disp(' T:')
    disp(-inv(results(j).H(1:3, 1:3))*results(j).H(1:3, 4))
end
