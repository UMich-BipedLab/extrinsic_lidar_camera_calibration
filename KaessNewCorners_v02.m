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
function [cross_big_3d, edges, bag]= KaessNewCorners_v02(base_line, bag, path, scan_number, which_tag, pc_iter)
    pc = load(string(path) + string(bag.lidar_target(which_tag).pc_file)); 
    pnts = pc.point_cloud; % [scan, point, [X, Y, X, I, R]]
    ransac_threshold = 0.02;
    base_line.pc_iter = pc_iter;

    d = bag.lidar_target(which_tag).tag_size*sqrt(2);% Large Target size
    
    if base_line.edge_method == 1
        [U,center,~,~,~,~,LEupper,LElower,REupper,RElower,~,~,~, flag_changed] = LeftRightEdges_v02(base_line, pnts, d);
    elseif base_line.edge_method == 2
        [U, center, LEupper, LElower, REupper, RElower, ~, ~, flag_changed] = clickedToFindEdges(base_line, pnts, d);
    elseif base_line.edge_method == 3
        [U, center, LEupper, LElower, REupper, RElower, ~, ~, flag_changed] = L1CostToFindEdges(base_line, pnts, d);
    end
    
    [nL1,nL2,nL3]=size(LEupper);
    LEupperall=reshape(LEupper,nL1,nL2*nL3);
    I=find( (LEupperall(1,:)~= 10) & (LEupperall(2,:)~= 10) ); 
    edges.LU=U*[1 0; 0 1;0 0]*LEupperall(:,I) + center;
    LEupperal_new = LEupperall(:, I);
%     if ~flag_changed
%         edges.LU=U*[1 0; 0 1;0 0]*LEupperall(:,I) + center;
%         LEupperal_new = LEupperall(:, I);
%     else
%         edges.LU=U*[1 0; 0 1;0 0]*LEupperall([2,1],I) + center;
%         LEupperal_new = LEupperall([2, 1], I);
%     end

    [nL1,nL2,nL3]=size(LElower);
    LElowerall=reshape(LElower,nL1,nL2*nL3);
    I=find( (LElowerall(1,:)~= 10) & (LElowerall(2,:)~= 10) ); 
    edges.LL=U*[1 0; 0 1;0 0]*LElowerall(:,I) + center;
    LElowerall_new = LElowerall(:, I);
%     if ~flag_changed
%         edges.LL=U*[1 0; 0 1;0 0]*LElowerall(:,I) + center;
%         LElowerall_new = LElowerall(:, I);
%     else
%         edges.LL=U*[1 0; 0 1;0 0]*LElowerall([2,1],I) + center;
%         LElowerall_new = LElowerall([2,1], I);
%     end


    [nR1,nR2,nR3]=size(REupper);
    REupperall=reshape(REupper,nR1,nR2*nR3);
    I=find( (REupperall(1,:)~= 10) & (REupperall(2,:)~= 10) ); 
    edges.RU=U*[1 0; 0 1; 0 0]*REupperall(:,I) + center;
    REupperall_new = REupperall(:, I);
%     if ~flag_changed
%         edges.RU=U*[1 0; 0 1; 0 0]*REupperall(:,I) + center;
%         REupperall_new = REupperall(:, I);
%     else
%         edges.RU=U*[1 0; 0 1; 0 0]*REupperall([2,1],I) + center;
%         REupperall_new = REupperall([2,1], I);
%     end


    [nR1,nR2,nR3]=size(RElower);
    RElowerall=reshape(RElower,nR1,nR2*nR3);
    I=find( (RElowerall(1,:)~= 10) & (RElowerall(2,:)~= 10) ); 
    edges.RL=U*[1 0; 0 1; 0 0]*RElowerall(:,I) + center;
    RElowerall_new = RElowerall(:, I);
%     if ~flag_changed
%         edges.RL=U*[1 0; 0 1; 0 0]*RElowerall(:,I) + center;
%         RElowerall_new = RElowerall(:, I);
%     else
%         edges.RL=U*[1 0; 0 1; 0 0]*RElowerall([2,1],I) + center;
%         RElowerall_new = RElowerall([2,1], I);
%     end

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
    cross_big_3d = [cross_big_3d; ones(1, size(cross_big_3d, 2))];
    cross_big_3d = sortrows(cross_big_3d', 3, 'descend')';
             
    if nargout > 2 
        bag.baseline(which_tag).scan(scan_number).corners = cross_big_3d;
    end
    if base_line.show_results
    % if 1
        current_img_handle = base_line.img_hangles(3);
        hold(current_img_handle, 'on')
        grid(current_img_handle,  'on')
        axis(current_img_handle, 'equal')
        scatter(current_img_handle, RElowerall_new(1,:), RElowerall_new(2,:), '.m'),
        plot(current_img_handle, x_BL, y_BL, 'r-')
        plot(current_img_handle, x_TL, y_TL, 'b-')
        plot(current_img_handle, x_TR, y_TR, 'g-')
        plot(current_img_handle, x_BR, y_BR, 'k-')
        plot(current_img_handle, [cross_L(1) cross_B(1)], [cross_L(2) cross_B(2)], 'r-')
        plot(current_img_handle, [cross_L(1) cross_T(1)], [cross_L(2) cross_T(2)], 'b-')
        plot(current_img_handle, [cross_R(1) cross_T(1)], [cross_R(2) cross_T(2)], 'g-')
        plot(current_img_handle, [cross_R(1) cross_B(1)], [cross_R(2) cross_B(2)], 'm-')
        scatter(current_img_handle, cross_big_2d(1,:), cross_big_2d(2,:), 'd')


        current_img_handle = base_line.img_hangles(4);
        scatter(current_img_handle, LEupperal_new(1,:), LEupperal_new(2,:),'.r')
        hold(current_img_handle, 'on')
        scatter(current_img_handle, LElowerall_new(1,:), LElowerall_new(2,:), '.b')
        scatter(current_img_handle, REupperall_new(1,:), REupperall_new(2,:), '.g')
        grid(current_img_handle,  'on')
        axis(current_img_handle, 'equal')
        scatter(current_img_handle, RElowerall_new(1,:), RElowerall_new(2,:), '.m'),
        plot(current_img_handle, x_BL, y_BL, 'r-')
        plot(current_img_handle, x_TL, y_TL, 'b-')
        plot(current_img_handle, x_TR, y_TR, 'g-')
        plot(current_img_handle, x_BR, y_BR, 'k-')
        plot(current_img_handle, [cross_L(1) cross_B(1)], [cross_L(2) cross_B(2)], 'r-')
        plot(current_img_handle, [cross_L(1) cross_T(1)], [cross_L(2) cross_T(2)], 'b-')
        plot(current_img_handle, [cross_R(1) cross_T(1)], [cross_R(2) cross_T(2)], 'g-')
        plot(current_img_handle, [cross_R(1) cross_B(1)], [cross_R(2) cross_B(2)], 'm-')
        scatter(current_img_handle, cross_big_2d(1,:), cross_big_2d(2,:), 'd')
        title(current_img_handle, "regressed edges")
        set(get(current_img_handle, 'parent'),'visible','on');


        current_img_handle = base_line.img_hangles(2);
        hold(current_img_handle, 'on')
        scatter3(current_img_handle, cross_big_3d(1,1), cross_big_3d(2,1), cross_big_3d(3,1), 'or')
        scatter3(current_img_handle, cross_big_3d(1,2), cross_big_3d(2,2), cross_big_3d(3,2), 'og')
        scatter3(current_img_handle, cross_big_3d(1,3), cross_big_3d(2,3), cross_big_3d(3,3), 'ob')
        scatter3(current_img_handle, cross_big_3d(1,4), cross_big_3d(2,4), cross_big_3d(3,4), 'om')

        scatter3(current_img_handle, edges.LU(1,:), edges.LU(2,:), edges.LU(3,:), 'sr')
        scatter3(current_img_handle, edges.LL(1,:), edges.LL(2,:), edges.LL(3,:), 'sg')
        scatter3(current_img_handle, edges.RU(1,:), edges.RU(2,:), edges.RU(3,:), 'sb')
        scatter3(current_img_handle, edges.RL(1,:), edges.RL(2,:), edges.RL(3,:), 'sm')

    end
end
