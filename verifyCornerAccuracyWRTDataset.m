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

function cost = verifyCornerAccuracyWRTDataset(indices, opt, bag_data, P)
    for i = 1:length(indices) % which validation data
        current_index = indices(i);
        cost_array = zeros(bag_data(current_index).num_tag, opt.num_lidar_target_pose);
        current_num_poses = bag_data(current_index).num_tag * opt.num_lidar_target_pose * opt.correspondance_per_pose;
        for j = 1:bag_data(current_index).num_tag % which tag in the validation dataset
            for k=1:opt.num_lidar_target_pose % which scan in the validation dataset
                current_corners_X = [bag_data(current_index).lidar_target(j).scan(k).corners];
                current_corners_Y = [bag_data(current_index).camera_target(j).corners];
                scan_cost = verifyCornerAccuracy(current_corners_X(:, 1:4), current_corners_Y(:, 1:4), P);
                cost_array(j, k) = scan_cost;
            end
        end
        cost(i).name = bag_data(current_index).bagfile;
        cost(i).total_cost = sum(sum(cost_array, 2), 1);
        cost(i).num_pose = current_num_poses;
        cost(i).RMSE = sqrt(sum(sum(cost_array, 2), 1)/current_num_poses); % total cost of this dataset
%         cost(i).std = std(cost_array'); % std of cost of each scan
    end
end
