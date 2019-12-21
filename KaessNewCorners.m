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

function [cross_big_3d, edges]= KaessNewCorners(target_size, path, pc_mat, pc_iter)
    pc = load(string(path) + string(pc_mat)); 
    pnts = pc.point_cloud; % [scan, point, [X, Y, X, I, R]]
    ransac_threshold = 0.02;

    d=target_size*sqrt(2);% Large Target size

    [U,center,~,RE,~,~,LEupper,LElower,REupper,RElower,~,~,~] = LeftRightEdges_v02(pnts, d, pc_iter);
    img_fig_handles = createFigHandle(3, "picked points vis");

%     [U, center, LEupper, LElower, REupper, RElower, ~, ~] = clickedToFindEdges(img_fig_handles, pnts, d, pc_iter);
    [nL1,nL2,nL3]=size(LEupper);
    LEupperall=reshape(LEupper,nL1,nL2*nL3);
    I=find( (LEupperall(1,:)~= 10) & (LEupperall(2,:)~= 10) ); 
    edges.LU=U*[1 0; 0 1;0 0]*LEupperall(:,I) + center;
    LEupperal_new = LEupperall(:, I);

    [nL1,nL2,nL3]=size(LElower);
    LElowerall=reshape(LElower,nL1,nL2*nL3);
    I=find( (LElowerall(1,:)~= 10) & (LElowerall(2,:)~= 10) ); 
    edges.LL=U*[1 0; 0 1;0 0]*LElowerall(:,I) + center;
    LElowerall_new = LElowerall(:, I);

    [nR1,nR2,nR3]=size(REupper);
    REupperall=reshape(REupper,nR1,nR2*nR3);
    I=find( (REupperall(1,:)~= 10) & (REupperall(2,:)~= 10) ); 
    edges.RU=U*[1 0; 0 1; 0 0]*REupperall(:,I) + center;
    REupperall_new = REupperall(:, I);

    [nR1,nR2,nR3]=size(RElower);
    RElowerall=reshape(RElower,nR1,nR2*nR3);
    I=find( (RElowerall(1,:)~= 10) & (RElowerall(2,:)~= 10) ); 
    edges.RL=U*[1 0; 0 1; 0 0]*RElowerall(:,I) + center;
    RElowerall_new = RElowerall(:, I);

    [x_TL, y_TL, modelInliers_TL] = ransacLine(LEupperal_new(1:2, :)', ransac_threshold);

    [x_BL, y_BL, modelInliers_BL] = ransacLine(LElowerall_new(1:2, :)', ransac_threshold);

    [x_TR, y_TR, modelInliers_TR] = ransacLine(REupperall_new(1:2, :)', ransac_threshold);

    [x_BR, y_BR, modelInliers_BR] = ransacLine(RElowerall_new(1:2, :)', ransac_threshold);


    %
    cross_L=intersection(modelInliers_TL, modelInliers_BL);

    cross_R=intersection(modelInliers_TR, modelInliers_BR);

    cross_T=intersection(modelInliers_TL, modelInliers_TR);


    cross_B=intersection(modelInliers_BR, modelInliers_BL);
    cross_big_2d = [cross_L, cross_R, cross_T, cross_B];

    cross_big_3d = U*[1 0; 0 1;0 0]*cross_big_2d + center;
    cross_big_3d = [cross_big_3d; ones(1,size(cross_big_3d,2))];
    cross_big_3d = sortrows(cross_big_3d', 3, 'descend')';
     
    % figure(103)
    % scatter(LEupperal_new(1,:), LEupperal_new(2,:),'.r'), hold on,
    % scatter(LElowerall_new(1,:), LElowerall_new(2,:), '.b'), hold on,
    % scatter(REupperall_new(1,:), REupperall_new(2,:), '.g'), hold on, grid on, axis equal
    % scatter(RElowerall_new(1,:), RElowerall_new(2,:), '.k'),
    % plot(x_BL, y_BL, 'r-')
    % plot(x_TL, y_TL, 'b-')
    % plot(x_TR, y_TR, 'g-')
    % plot(x_BR, y_BR, 'k-')
    % plot([cross_L(1) cross_B(1)], [cross_L(2) cross_B(2)], 'r-')
    % plot([cross_L(1) cross_T(1)], [cross_L(2) cross_T(2)], 'b-')
    % plot([cross_R(1) cross_T(1)], [cross_R(2) cross_T(2)], 'g-')
    % plot([cross_R(1) cross_B(1)], [cross_R(2) cross_B(2)], 'k-')
    % scatter(cross_big_2d(1,:), cross_big_2d(2,:), 'd')
 end
