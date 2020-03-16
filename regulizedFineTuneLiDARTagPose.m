%{
 * Copyright (C) 2013-2020, The Regents of The University of Michigan.
 * All rights reserved.
 * This software was developed in the Biped Lab (https://www.biped.solutions/) 
 * under the direction of Jessy Grizzle, grizzle@umich.edu. This software may 
 * be available under alternative licensing terms; contact the address above.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of the Regents of The University of Michigan.
 * 
 * AUTHOR: Bruce JK Huang (bjhuang[at]umich.edu)
 * WEBSITE: https://www.brucerobot.com/
%}

function [X, bag_data] = regulizedFineTuneLiDARTagPose(tag_size_array, X, Y, H_LT, P, correspondance_per_pose, display, bag_data)
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
        if nargout == 2 && nargin == 8
            bag_data.baseline(which_tag).scan(scan_number).corners = cross_big_3d;
        end
    end
end
