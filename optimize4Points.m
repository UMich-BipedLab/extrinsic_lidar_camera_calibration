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

function [final_H, final_P, min_cost, final_result, results] = optimize4Points(opt, X, Y, intrinsic, display)
        
    % prepare for random initilization (not use for now)
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
        P = intrinsic * [eye(3) zeros(3,1)] * H_LC;
        
        results(i).total_cost = fval;
        results(i).RMSE = sqrt(fval/size(Y,2));
        results(i).H = H_LC;
        results(i).P = P;
        if display
            disp('-- H_LC: ')
            disp('------- R:')
            disp(H_LC(1:3, 1:3))
            disp('------- T:')
            disp(-inv(H_LC(1:3, 1:3))*H_LC(1:3, 4))
            disp('-- cost:')
            disp(results(i).total_cost)
            disp('-- RMSE:')
            disp(results(i).RMSE)
        end 
    end
    
    [min_cost, k] = min([results(:).total_cost]);
    [max_cost, ~] = max([results(:).total_cost]);
    [min_RMSE, ~] = min([results(:).RMSE]);
    final_result.RMSE = min_RMSE;
    final_result.std = std([results(:).total_cost]);
    final_result.min_cost = min_cost;
    final_result.max_cost = max_cost;
    final_H = results(k).H;
    final_P = results(k).P;
    final_result.H = final_H;
    final_result.P = final_P;
    if display
%         disp("std:")
%         disp(final_result.std)
%         disp("minimum cost:")
%         disp(min_cost)
%         disp('H_LC: ')
%         disp(' R:')
%         disp(results(k).H(1:3, 1:3))
%         disp(' RPY (XYZ):')
%         disp(rad2deg(rotm2eul(results(k).H(1:3, 1:3), "XYZ")))
%         disp(' T:')
%         disp(-inv(results(k).H(1:3, 1:3))*results(k).H(1:3, 4))
%         disp("max cost:")
%         disp(max_cost)
%         disp('H_LC: ')
%         disp(' R:')
%         disp(results(j).H(1:3, 1:3))
%         disp(' RPY (XYZ):')
%         disp(rad2deg(rotm2eul(results(j).H(1:3, 1:3), "XYZ")))
%         disp(' T:')
%         disp(-inv(results(j).H(1:3, 1:3))*results(j).H(1:3, 4))
    end
end
