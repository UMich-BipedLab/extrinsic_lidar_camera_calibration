%{
 * Copyright (C) 2020-2030, The Regents of The University of Michigan.
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


function [X_clean, clean_up, remaining_indices, edges] = cleanLiDARTargetWithOneDataSetWithIndices(X_original, target_len, opt)
    N = 4; % clean up using N std for x axis
    M = 3; % clean up using M std for y, z axis
    opt = optimizeCost(opt, X_original, target_len, 0.001);
    if size(X_original, 2) ~= 4
        X = [X_original(1:3, :); 
             ones(1 ,size(X_original, 2))];
    end
    
    X_ref = opt.H_opt * X;
%     distance = sum(X_ref(1:3,:), 1); % L1
    L_infinity = max(abs(X_ref(1:3,:)));  % L infinity
    K = find(L_infinity < (target_len/2)*1.025);
    X_ref_clean_yz = X_ref(:, K); % clean up y and z axis
    L_infinity = max(X_ref_clean_yz(1:3,:));
    K_center = find(L_infinity < target_len/4);
    X_std = std(X_ref_clean_yz(:, K_center), 1, 2);
    Q = find(abs(X_ref(1,:)) < N*(X_std(1))); % clean up x with 2 std
    remaining_indices = intersect(K, Q);
    X_ref_clean = X_ref(:, remaining_indices);
    X_clean = inv(opt.H_opt) * X_ref_clean;
    clean_up.std = N*(X_std(1));
    clean_up.L_infinity = L_infinity;
    clean_up.L_1 = sum(X_ref(1:3,:), 1);
    X_ref_with_all_info = X_original;
    X_ref_with_all_info(1:3, :) = X_ref(1:3,:);
    edges = findEdgePointsInIdealFrame(opt.H_opt, X_ref_with_all_info, target_len);
%     figure(200);
%     clf('reset')
%     scatter3(X_ref(1,:), X_ref(2,:), X_ref(3,:))
%     xlabel('x') 
%     ylabel('y') 
%     zlabel('z') 
%     axis equal
%     hold on;
% %     scatter3(X_ref_clean_yz(1,:), X_ref_clean_yz(2,:), X_ref_clean_yz(3,:))
%     scatter3(X_ref_clean(1,:), X_ref_clean(2,:), X_ref_clean(3,:))
%     axis equal
%     view(90,0)
%     hold off;    
end