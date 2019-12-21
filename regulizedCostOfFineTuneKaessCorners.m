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

function cost = regulizedCostOfFineTuneKaessCorners(theta_x, theta_y, theta_z, T, X, Y, edges_points, P)
    R = rotx(theta_x) * roty(theta_y) * rotz(theta_z);
    x_prime = R(1, :) * X(1:3, :) + T(1);
    y_prime = R(2, :) * X(1:3, :) + T(2);
    z_prime = R(3, :) * X(1:3, :) + T(3);

    L_X_transformed = [x_prime; y_prime; z_prime; ones(size(x_prime))]; % transformed points in LiDAR frame
    C_X_transformed = P * L_X_transformed;
    C_X_transformed = C_X_transformed ./ C_X_transformed(3,:);
    cost_LU = pointToLineDistance([edges_points.LU]', L_X_transformed(1:3, 1)', L_X_transformed(1:3, 2)');
    cost_LL = pointToLineDistance([edges_points.LL]', L_X_transformed(1:3, 2)', L_X_transformed(1:3, 4)');
    cost_RU = pointToLineDistance([edges_points.RU]', L_X_transformed(1:3, 1)', L_X_transformed(1:3, 3)');
    cost_RL = pointToLineDistance([edges_points.RL]', L_X_transformed(1:3, 3)', L_X_transformed(1:3, 4)');
    total_cost = sum(cost_LU) + sum(cost_LL) + sum(cost_RU) + sum(cost_RL); % four sides of the squares


    cost = norm(C_X_transformed(1:2,:) - Y(1:2,:), 'fro') + 0.005*total_cost;
end