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


function edge = findEdgePointsInIdealFrame(H_TL, X_ref, tag_length)
    % X_ref: points that have been transformed back to the ideal frame
    % X_ref(1,:) : x
    % X_ref(2,:) : y
    % X_ref(3,:) : z
    % X_ref(4,:) : I
    % X_ref(5,:) : R
    % X_ref(6,:) : scan
    % X_ref(7,:) : ExpNmbr
    top_ring = max(X_ref(5, :));
    bottom_ring = min(X_ref(5, :));
    num_scan = max(X_ref(6, :)) - min(X_ref(6, :));
    pick_points = max(1, ceil(num_scan));
    % Ideal frame (top-left, bottom-left, bottom-right, top-right)
%     ideal_frame.x = zeros(1,8);
%     ideal_frame.y = [-tag_length/2, -tag_length/2, -tag_length/2, tag_length/2, tag_length/2, tag_length/2, tag_length/2, -tag_length/2];
%     ideal_frame.z = [tag_length/2, -tag_length/2, -tag_length/2, -tag_length/2, -tag_length/2, tag_length/2, tag_length/2, tag_length/2];
    
    % left, bottom, right, top (y-axis, -z-axis, -y-axis, z-axis)
    edge_list(1).line = [-tag_length/2 0; -tag_length/2 1];
    edge_list(2).line = [0 -tag_length/2; 1 -tag_length/2];
    edge_list(3).line = [tag_length/2 0; tag_length/2 1];
    edge_list(4).line = [0 tag_length/2; 1 tag_length/2];
    edge(4).points = [];
    
    % choose one point per scan per ring
%     for j = 1:num_scan
%         current_scan_points = X_ref(:, X_ref(6,:)==j);
%         for i = bottom_ring:top_ring
%             if size(current_scan_points(2:3, current_scan_points(5,:)==i), 2) > 1
%                 current_ring_points = current_scan_points(1:3, current_scan_points(5,:)==i);
%                 extreme_points = [current_ring_points(:, 1), current_ring_points(:, end)];
%                 extreme_point1 = current_ring_points(:, 1);
%                 extreme_point2 = current_ring_points(:, end);
%                 dist1 = zeros(1, 4);
%                 dist2 = zeros(1, 4);
%                 for k = 1:4
%                     dist1(1, k) = pointToLineDistance(extreme_point1(2:3, :)', edge_list(k).line(1, :), edge_list(k).line(2, :));
%                     dist2(1, k) = pointToLineDistance(extreme_point2(2:3, :)', edge_list(k).line(1, :), edge_list(k).line(2, :));
%                 end
%                 [~, edge1_i] = min(dist1);
%                 [~, edge2_i] = min(dist2);
%                 edge_point_3D_1 = inv(H_TL) * [extreme_point1; 1];
%                 edge_point_3D_2 = inv(H_TL) * [extreme_point2; 1];
% %                 selected_edge_points = [current_ring_points(1:3, indices(selected_indices)); ones(1, length(selected_indices))];
%                 edge(edge1_i).points = [edge(edge1_i).points, edge_point_3D_1];
%                 edge(edge2_i).points = [edge(edge2_i).points, edge_point_3D_2];
%             end
%         end
%     end
    
    
     % choose a few points using all scans
    for i = bottom_ring:top_ring
        current_ring_points = X_ref(:, X_ref(5,:)==i);
%         dist = zeros(length(current_ring_points), 4);
        for k = 1:4
            dist = pointToLineDistance(current_ring_points(2:3,:)', edge_list(k).line(1, :), edge_list(k).line(2, :));
            [value, indices] = mink(dist, pick_points);
            selected_indices = find (value<tag_length/8);
            selected_edge_points = inv(H_TL) * [current_ring_points(1:3, indices(selected_indices)); ones(1, length(selected_indices))];
%             selected_edge_points = [current_ring_points(1:3, indices(selected_indices)); ones(1, length(selected_indices))];
            edge(k).points = [edge(k).points, selected_edge_points];
        end
    end
    
    
%     edge_list = [tag_length/2, -tag_length/2, -tag_length/2, tag_length/2];
%     edge(4).points = [];
%     for i = bottom_ring:top_ring
%         current_ring_points = X_ref(:, X_ref(5,:)==i);
%         first_point = current_ring_points(:, 1);
%         edge_distance = [abs(first_point(2) - edge_list(1)), ...
%                          abs(first_point(3) - edge_list(2)), ...
%                          abs(first_point(2) - edge_list(3)), ...
%                          abs(first_point(3) - edge_list(4))];
%         [~, first_index] = min(edge_distance);
%         first_point = inv(H_TL) * [first_point(1:3); 1];
%         edge(first_index).points = [edge(first_index).points, first_point];
%         last_point = current_ring_points(:, end);
%         edge_distance = [abs(last_point(2) - edge_list(1)), ...
%                          abs(last_point(3) - edge_list(2)), ...
%                          abs(last_point(2) - edge_list(3)), ...
%                          abs(last_point(3) - edge_list(4))];
%         [~, last_index] = min(edge_distance);
%         last_point = inv(H_TL) * [last_point(1:3); 1];
%         edge(last_index).points = [edge(last_index).points, last_point];
%     end
%     fig_hangle = figure(99999);
%     cla(fig_hangle)
%     X_move_back = inv(H_TL) * [X_ref(1:3,:); ones(1, size(X_ref(1, :), 2))];
%     scatter3(X_move_back(1,:), X_move_back(2,:), X_move_back(3,:), 'k.')
% %     scatter3(X_ref(1,:), X_ref(2,:), X_ref(3,:), 'k.')
%     hold on
%     scatter3(edge(1).points(1,:), edge(1).points(2,:), edge(1).points(3,:), 'ro', 'filled')
%     scatter3(edge(2).points(1,:), edge(2).points(2,:), edge(2).points(3,:), 'go', 'filled')
%     scatter3(edge(3).points(1,:), edge(3).points(2,:), edge(3).points(3,:), 'bo', 'filled')
%     scatter3(edge(4).points(1,:), edge(4).points(2,:), edge(4).points(3,:), 'mo', 'filled')
%     title('edge parsing results')
%     xlabel("x")
%     ylabel("y")
%     zlabel("z")
%     axis equal
end