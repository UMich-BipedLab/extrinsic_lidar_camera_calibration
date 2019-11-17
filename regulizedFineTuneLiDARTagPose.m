function X = regulizedFineTuneLiDARTagPose(tag_size_array, X, Y, H_LT, P, correspondance_per_pose, display)
    theta_x = optimvar('theta_x', 1, 1,'LowerBound',-5,'UpperBound',5); % 1x1
    theta_y = optimvar('theta_y', 1, 1,'LowerBound',-5,'UpperBound',5); % 1x1
    theta_z = optimvar('theta_z', 1, 1,'LowerBound',-5,'UpperBound',5); % 1x1
    T = optimvar('T', 1, 3,'LowerBound',-0.1,'UpperBound',0.1);
    prob = optimproblem;
    num_pose = size(X, 2)/correspondance_per_pose; % 4 correspondance per pose

    for i = 1 : num_pose
        target_size = tag_size_array(i);
        pose_num = correspondance_per_pose * (i-1) + 1;  
        f = fcn2optimexpr(@regulizedCostOfFineTuneLiDARTagPose, theta_x, theta_y, theta_z, T, ...
                         X(:,pose_num:pose_num+correspondance_per_pose-1), ...
                         Y(:,pose_num:pose_num+correspondance_per_pose-1), ...
                         H_LT(:, pose_num:pose_num+correspondance_per_pose-1), P, target_size);
        prob.Objective = f;
        x0.theta_x = 0;
        x0.theta_y = 0;
        x0.theta_z = 0;
        x0.T = [0 0 0];           

        options = optimoptions('fmincon', 'MaxIter',5e2, 'Display','off', ...
                               'TolX', 1e-12, 'FunctionTolerance', 1e-8, ...
                               'MaxFunctionEvaluations', 3e4, 'StepTolerance', 1e-20);
        [sol, fval, ~, ~] = solve(prob, x0, 'Options', options);
        R_final = rotx(sol.theta_x) * roty(sol.theta_y) * rotz(sol.theta_z);
        H_fine_tune = eye(4);
        H_fine_tune(1:3, 1:3) = R_final;
        H_fine_tune(1:3, 4) = sol.T';
        
        if checkDisplay(display)
            disp('new H_LT: ')
            disp(H_fine_tune)
            disp('cost:')
            disp(fval)
        end
%         dbstop in regulizedFineTuneLiDARTagPose at 40 if fval>=100
%         dbstop in regulizedFineTuneLiDARTagPose at 40 if det(H_fine_tune)==1
        %regulizedCostOfFineTuneLiDARTagPose(sol.theta_x, sol.theta_y, sol.theta_z, sol.T, X(:,pose_num:pose_num+correspondance_per_pose-1), Y(:,pose_num:pose_num+correspondance_per_pose-1),  H_LT(:, pose_num:pose_num+correspondance_per_pose-1), P, target_size)
        X(:,pose_num:pose_num+correspondance_per_pose-1) = H_fine_tune * X(:,pose_num:pose_num+correspondance_per_pose-1);
    end
end
