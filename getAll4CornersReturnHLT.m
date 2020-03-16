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

function [bag_data, H_LT] = getAll4CornersReturnHLT(tag_num, opt, path, bag_data, opts)
% scan_num: scan number of these corner
% num_scan: how many scans accumulated to get the corners
% function [LiDARTag, AprilTag, H_LT] = get4CornersReturnHLT(tag_num, opt, mat_file_path, pc_mat_file, bag_file, target_len, pc_iter, num_scan)
    
%     if show.lidar_target_optimization
%         target_len = bag_data.lidar_target(tag_num).tag_size; 
%         pc = loadPointCloud(path.mat_file_path, bag_data.lidar_target(tag_num).pc_file);
%         for pc_iter = 1:5
%             X = getPayload(pc, pc_iter, opts.num_scan);
%             opt_temp_draw = opt.H_TL;
%             [X_clean, scan_total_draw(pc_iter).clean_up] = cleanLiDARTargetWithOneDataSet(X, target_len, opt_temp_draw);
% 
%             cost
%             opt_temp_draw = optimizeCost(opt_temp_draw, X_clean, target_len, scan_total_draw(pc_iter).clean_up.std/2);
%             target_lidar = [0 -target_len/2 -target_len/2 1;
%                             0 -target_len/2  target_len/2 1;
%                             0  target_len/2  target_len/2 1;
%                             0  target_len/2 -target_len/2 1]';
% 
%             corners = opt_temp_draw.H_opt \ target_lidar;
%             corners = sortrows(corners', 3, 'descend')';
%             centroid = mean(corners(1:3,:), 2);
%             normals = cross(corners(1:3,1)-corners(1:3,2), corners(1:3,1)-corners(1:3,3));
%             normals = normals/(norm(normals));
%             if normals(1) > 0
%                 normals = -normals;
%             end
%         end
%     end

    pc = loadPointCloud(path.mat_file_path, bag_data.lidar_target(tag_num).pc_file);
    num_scan_total = size(pc, 1) - opts.num_scan;
    target_len = bag_data.lidar_target(tag_num).tag_size; 
    if opts.optimizeAllCorners
        scan_total(num_scan_total) = struct();
        scan_total(num_scan_total).clean_up = [];
        scan_total(num_scan_total).corners = [];
        scan_total(num_scan_total).four_corners_line = [];
        scan_total(num_scan_total).pc_points_original = [];
        scan_total(num_scan_total).pc_points = [];
        scan_total(num_scan_total).centroid = [];
        scan_total(num_scan_total).normal_vector = [];
        scan_total(num_scan_total).H = []; %% H_LT
        
        disp("------------------------------------------------------------")
        disp("Optimizing LiDAR vertices using PARFOR, the numbers WILL NOT BE in ORDERED")
        disp("------------------------------------------------------------")
        tic
        parfor pc_iter = 1:num_scan_total
            if mod(pc_iter, 10) == 0 || pc_iter == num_scan_total || pc_iter == 1
                fprintf("--- Working on scan: %i/%i\n", pc_iter, num_scan_total)
            end
%             fprintf("--- Working on scan: %i/%i\n", pc_iter, num_scan_total)
            X = getPayload(pc, pc_iter, opts.num_scan);
            opt_temp = opt.H_TL;
            [X_clean, scan_total(pc_iter).clean_up] = cleanLiDARTargetWithOneDataSet(X, target_len, opt_temp);

            % cost
            opt_temp = optimizeCost(opt_temp, X_clean, target_len, scan_total(pc_iter).clean_up.std/2);
            target_lidar = [0 -target_len/2 -target_len/2 1;
                            0 -target_len/2  target_len/2 1;
                            0  target_len/2  target_len/2 1;
                            0  target_len/2 -target_len/2 1]';

            corners = opt_temp.H_opt \ target_lidar;
            corners = sortrows(corners', 3, 'descend')';
            [centroid, normals] = computeCentroidAndNormals(corners);
            scan_total(pc_iter).corners = corners;
            scan_total(pc_iter).four_corners_line = point3DToLineForDrawing(corners);
            scan_total(pc_iter).pc_points_original = X;
            scan_total(pc_iter).pc_points = X_clean;
            scan_total(pc_iter).centroid = centroid;
            scan_total(pc_iter).normal_vector = normals;
            scan_total(pc_iter).H = inv(opt_temp.H_opt);
        end
        time_elp = toc;
        fprintf("Spent %f on optimizing corners of %i scans", time_elp, num_scan_total)
        % 
        for pc_iter = 1:num_scan_total
            for j = 1:num_scan_total
                if j == pc_iter
                    similarity_table(pc_iter).scan(j).diff = 1000;
                else
                    difference = scan_total(j).H \ scan_total(pc_iter).H;
                    logR = logm(difference(1:3,1:3));
                    v = [-logR(1,2) logR(1,3) -logR(2,3) difference(1:3,4)'];
                    similarity_table(pc_iter).scan(j).diff = norm(v);
                end
            end
            similarity_table(pc_iter).mins = sum(mink([similarity_table(pc_iter).scan(:).diff], opts.num_lidar_target_pose));
        end
        if ~exist(path.save_dir, 'dir')
            mkdir(path.save_dir)
        end
        save(path.save_dir + extractBetween(bag_data.bagfile,"",".bag") + '_' + tag_num + '_' + '_all_scan_corners.mat', 'similarity_table', 'scan_total');
    else
        load(path.load_all_vertices + extractBetween(bag_data.bagfile,"",".bag") + '_' + tag_num + '_' + '_all_scan_corners.mat');
    end
    
    if opts.use_top_consistent_vertices
        [~, chosen_scan] = min([similarity_table(:).mins]);
        [~, chosen_scans] = mink([similarity_table(chosen_scan).scan(:).diff], opts.num_lidar_target_pose);
    else 
        chosen_scans = randperm(num_scan_total, opts.num_lidar_target_pose); 
    end
    H_LT = [];
    
    for scan_num = 1:opts.num_lidar_target_pose
        if opts.use_top_consistent_vertices || opts.randperm_to_fine_vertices
            current_scan = chosen_scans(scan_num);
        else
            current_scan = opts.num_scan*(scan_num-1) + 1;
        end
        bag_data.lidar_target(tag_num).scan(scan_num).corners = scan_total(current_scan).corners;
        bag_data.lidar_target(tag_num).scan(scan_num).four_corners_line = point3DToLineForDrawing(scan_total(current_scan).corners);
        bag_data.lidar_target(tag_num).scan(scan_num).pc_points_original = scan_total(current_scan).pc_points_original;
        bag_data.lidar_target(tag_num).scan(scan_num).pc_points = scan_total(current_scan).pc_points;
        bag_data.lidar_target(tag_num).scan(scan_num).centroid = scan_total(current_scan).centroid;
        bag_data.lidar_target(tag_num).scan(scan_num).normal_vector = scan_total(current_scan).normal_vector;
        bag_data.lidar_target(tag_num).scan(scan_num).H = scan_total(current_scan).H;
        H_LT = [H_LT scan_total(current_scan).H];
    end

    
    if opts.refineAllCorners
        num_scan_total = size(scan_total, 2) - opts.num_scan;
        refinement_scan_total(num_scan_total) = struct();
        refinement_scan_total(num_scan_total).original_corners = [];
        refinement_scan_total(num_scan_total).refined_corners = [];
        refinement_scan_total(num_scan_total).refined_centroid = [];
        refinement_scan_total(num_scan_total).refined_normal_vector = [];
        refinement_scan_total(num_scan_total).refined_H = [];
        refinement_scan_total(num_scan_total).refined_P = [];
        
        disp("\n------------------------------------------------------------")
        disp("Refining LiDAR vertices using PARFOR, the numbers WILL NOT BE in ORDERED")
        disp("------------------------------------------------------------")
        tic
        parfor pc_iter = 1:num_scan_total
            if mod(pc_iter, 10) == 0 || pc_iter == num_scan_total || pc_iter == 1
                fprintf("--- Working on refinement scan: %i/%i\n", pc_iter, num_scan_total)
            end
            
            if opts.use_top_consistent_vertices
                [~, chosen_scan] = min([similarity_table(:).mins]);
                [~, chosen_scans] = mink([similarity_table(chosen_scan).scan(:).diff], opts.num_lidar_target_pose);
            else 
                chosen_scans = randperm(num_scan_total, opts.num_lidar_target_pose); 
            end            
            X_train = [scan_total(pc_iter:pc_iter + opts.num_lidar_target_pose - 1).corners];
            Y_train = [repmat(bag_data.camera_target(tag_num).corners, 1, opts.num_lidar_target_pose)];
            H_LT = [scan_total(pc_iter:pc_iter + opts.num_lidar_target_pose - 1).H];
            
            switch opts.calibration_method
                case "4 points"
                    for i = 0: opts.num_refinement-1
%                         disp('---------------------')
%                         disp(' Optimizing H_LC ...')
%                         disp('---------------------')
% 
% 
%                         disp('---------------------')
%                         disp('--- SR_H_LC ...')
%                         disp('---------------------')
                        % square with refinement
                        show_pnp_numerical_result = 0;
                        [SR_H_LC, SR_P, SR_opt_total_cost, SR_final, SR_All] = optimize4Points(opt.H_LC.rpy_init, ...
                                                                                               X_train, Y_train, ... 
                                                                                               opt.intrinsic_matrix, ...
                                                                                               show_pnp_numerical_result);

                        if i == opts.num_refinement-1
                            break;
                        else
%                             disp('------------------')
%                             disp(' Refining SR_H_LC ...')
%                             disp('------------------')
                            X_refined_corners = [];
                            for scan_num = 1:opts.num_lidar_target_pose
                                current_scan = pc_iter + scan_num;
                                X_train_tmp = [scan_total(current_scan).corners];
                                Y_train_tmp = [bag_data.camera_target(tag_num).corners];
                                H_LT = scan_total(current_scan).H;
                                tag_size = bag_data.lidar_target(tag_num).tag_size;
                                refinement_scan_total(pc_iter).refined_corners = regulizedFineTuneEachLiDARTagPose(tag_size, ...
                                                                                                X_train_tmp, Y_train_tmp, H_LT, SR_P, ...
                                                                                                show_pnp_numerical_result);
                                refinement_scan_total(pc_iter).original_corners = X_train_tmp;
                                X_refined_corners = [X_refined_corners, refinement_scan_total(pc_iter).refined_corners];
                                X_refined_corners = sortrows(X_refined_corners', 3, 'descend')';
                                [centroid, normals] = computeCentroidAndNormals(X_refined_corners);
                            end
                            X_train = X_refined_corners;
%                             X_not_square_refinement = regulizedFineTuneKaessCorners(X_not_square_refinement, Y_base_line,...
%                                                                                     X_base_line_edge_points, NSR_P, ...
%                                                                                     opts.correspondance_per_pose, show_pnp_numerical_result);
                        end
                        refinement_scan_total(pc_iter).refined_centroid = centroid;
                        refinement_scan_total(pc_iter).refined_normal_vector = normals;
                        refinement_scan_total(pc_iter).refined_H = SR_H_LC;
                        refinement_scan_total(pc_iter).refined_P = SR_P;
                    end

                case "IoU"
                    disp("refinement for IoU hasn't been implenmented yet. Will come soon!")
            end
        end
        time_elp = toc;
        fprintf("Spent %f on refining corners of %i scans", time_elp, num_scan_total)
        if ~exist(path.save_dir, 'dir')
            mkdir(path.save_dir)
        end
%         save(path.save_dir + extractBetween(bag_data.bagfile,"",".bag") + '_' + tag_num + '_' + '_all_scan_refined_corners.mat', 'refinement_scan_total');
        save(path.save_dir + extractBetween(bag_data.bagfile,"",".bag") + '_' + tag_num + '_' + path.event_name + '_all_scan_refined_corners.mat', 'refinement_scan_total');
    
    else
        load(path.load_all_vertices + extractBetween(bag_data.bagfile,"",".bag") + '_' + tag_num + '_' + path.event_name + '_all_scan_refined_corners.mat');
    end
    
    for scan_num = 1:opts.num_lidar_target_pose
        if opts.use_top_consistent_vertices || opts.randperm_to_fine_vertices
            current_scan = chosen_scans(scan_num);
        else
            current_scan = opts.num_scan*(scan_num-1) + 1;
        end
        bag_data.lidar_target(tag_num).scan(scan_num).refined_corners = refinement_scan_total(current_scan).refined_corners;
        bag_data.lidar_target(tag_num).scan(scan_num).refined_centroid = refinement_scan_total(current_scan).refined_centroid;
        bag_data.lidar_target(tag_num).scan(scan_num).refined_normal_vector = refinement_scan_total(current_scan).refined_normal_vector;
        bag_data.lidar_target(tag_num).scan(scan_num).refined_H = refinement_scan_total(current_scan).refined_H;
        bag_data.lidar_target(tag_num).scan(scan_num).refined_P = refinement_scan_total(current_scan).refined_P;
    end
end

