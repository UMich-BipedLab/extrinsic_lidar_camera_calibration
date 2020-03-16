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

function cost = costIoU(theta_x, theta_y, theta_z, T, X, Y, intrinsic)
    H = eye(4);
    H(1:3,1:3) = rotx(theta_x) * roty(theta_y) * rotz(theta_z);
    H(1:3,4) = T';
    P = intrinsic * [eye(3) zeros(3,1)] * H;
    L_X_transformed = P * X; % transformed points in LiDAR frame
    C_X_transformed = L_X_transformed ./ L_X_transformed(3,:);
    cost = 0;
    for i=1:size(C_X_transformed,2)/4
        k = 4*(i-1) + 1;
        vertices_X = [C_X_transformed(1,k) C_X_transformed(1,k+1) C_X_transformed(1,k+3) C_X_transformed(1,k+2); 
                      C_X_transformed(2,k) C_X_transformed(2,k+1) C_X_transformed(2,k+3) C_X_transformed(2,k+2)];
        vertices_Y = [Y(1,k) Y(1,k+1) Y(1,k+3) Y(1,k+2); 
                      Y(2,k) Y(2,k+1) Y(2,k+3) Y(2,k+2)];
%         vertices_X = C_X_transformed(1:2,k:k+3);
%         vertices_Y = Y(1:2,k:k+3);
        
%         conv_X = convhull(vertices_X(1,:)', vertices_X(2,:)');
%         conv_Y = convhull(vertices_Y(1,:)', vertices_Y(2,:)');
%         [vertices_X(1,:), vertices_X(2,:)] = poly2cw(vertices_X(1,conv_X(1:4)), vertices_X(2,conv_X(1:4)));
%         [vertices_Y(1,:), vertices_Y(2,:)] = poly2cw(vertices_Y(1,conv_Y(1:4)), vertices_Y(2,conv_Y(1:4)));
%         pgon_X =  polyshape(vertices_X(1,conv_X), vertices_X(2,conv_X));
%         pgon_Y =  polyshape(vertices_Y(1,conv_Y), vertices_Y(2,conv_Y));
        
        [vertices_X(1,:), vertices_X(2,:)] = poly2cw(vertices_X(1, :), vertices_X(2, :));
        [vertices_Y(1,:), vertices_Y(2,:)] = poly2cw(vertices_Y(1, :), vertices_Y(2, :));
        
        pgon_X =  polyshape(vertices_X(1, :), vertices_X(2, :));
        pgon_Y =  polyshape(vertices_Y(1, :), vertices_Y(2, :));
        if all(issimplified([pgon_X, pgon_Y]))
            [I_polyout,~,~] = intersect(pgon_X, pgon_Y);

            [U_polyout,~,~] = union(pgon_X ,pgon_Y);
            I_area = max(area(I_polyout), 1e-5);
            U_area = area(U_polyout);
            cost = cost + I_area/U_area;
        else
            cost = -1e3;
        end
    end
    cost = -cost;
end