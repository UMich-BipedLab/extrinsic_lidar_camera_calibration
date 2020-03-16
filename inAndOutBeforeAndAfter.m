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

function [count_no_refinement, count_refinement] = inAndOutBeforeAndAfter(bag_indices, opt_num_dataset, opt, bag_data, P1, P2)
	for i = 1:opt_num_dataset % which dataset is this
        current_index = bag_indices(i);
        name = bag_data(current_index).bagfile;
        count_array_no_refinement = zeros(bag_data(current_index).num_tag, opt.num_scan);
        count_array_refinement = zeros(bag_data(current_index).num_tag, opt.num_lidar_target_pose);

        total_num_point = 0;
        for j = 1:bag_data(current_index).num_tag % which tag in the validation dataset
            current_camera_corners = [bag_data(current_index).camera_target(j).corners];
            current_camera_corners = [current_camera_corners(:,1), current_camera_corners(:,2), current_camera_corners(:,4), current_camera_corners(:,3)];

            for k=1:opt.num_lidar_target_pose % which scan in the validation dataset
                current_lidar_target_pc = [bag_data(current_index).lidar_target(j).scan(k).pc_points];
                num_point = size(current_lidar_target_pc,2);
                total_num_point = total_num_point + num_point;
                if size(current_lidar_target_pc, 1) ~= 4
                    current_lidar_target_pc = [current_lidar_target_pc; ones(1,size(num_point, 2))];
                end
                projection_before_refinement = P1 * current_lidar_target_pc;
                projection_before_refinement = projection_before_refinement ./projection_before_refinement(3,:);
                [in_before, on_before] = inpolygon(projection_before_refinement(1,:)', projection_before_refinement(2,:)', ...
                                                   current_camera_corners(1,:)' ,current_camera_corners(2,:)');

                projection_after_refinement = P2 * current_lidar_target_pc;
                projection_after_refinement = projection_after_refinement ./projection_after_refinement(3,:);
                [in_after, on_after] = inpolygon(projection_after_refinement(1,:)', projection_after_refinement(2,:)', ...
                                                 current_camera_corners(1,:)' ,current_camera_corners(2,:)');
                
                count_array_no_refinement(j, k) =  numel(projection_before_refinement(in_before)) + numel(projection_before_refinement(on_before));
                count_array_refinement(j, k) = numel(projection_after_refinement(in_after)) + numel(projection_after_refinement(on_after));
            end
        end
        count_no_refinement(i).name = name;
        count_no_refinement(i).count = sum(sum(count_array_no_refinement, 2), 1); % total cost of this dataset
        count_no_refinement(i).std = std(count_array_no_refinement'); % std of cost of each scan
        count_no_refinement(i).fraction = sum(sum(count_array_no_refinement, 2), 1)/ total_num_point; 
        count_no_refinement(i).total_num_point = total_num_point; 

        count_refinement(i).name = name;
        count_refinement(i).count = sum(sum(count_array_refinement, 2), 1); % total cost of this dataset
        count_refinement(i).std = std(count_array_refinement'); % std of cost of each scan
        count_refinement(i).fraction = sum(sum(count_array_refinement, 2), 1)/ total_num_point; % total cost of this dataset
        count_refinement(i).total_num_point = total_num_point; % total cost of this dataset
	end
end
