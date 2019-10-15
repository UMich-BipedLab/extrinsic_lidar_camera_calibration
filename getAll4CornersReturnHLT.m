function [bag_data, H_LT] = getAll4CornersReturnHLT(optimizeAllCorners, tag_num, opt, path, bag_data, num_scan, num_pose)
% scan_num: scan number of these corner
% num_scan: how many scans accumulated to get the corners
% function [LiDARTag, AprilTag, H_LT] = get4CornersReturnHLT(tag_num, opt, mat_file_path, pc_mat_file, bag_file, target_len, pc_iter, num_scan)
    if optimizeAllCorners
        pc = loadPointCloud(path.mat_file_path, bag_data.lidar_target(tag_num).pc_file);
        target_len = bag_data.lidar_target(tag_num).tag_size;
        num_scan_total = size(pc, 1)-num_scan;
        tic
        scan_total(num_scan_total) = struct();
        scan_total(num_scan_total).clean_up = [];
        scan_total(num_scan_total).corners = [];
        scan_total(num_scan_total).four_corners_line = [];
        scan_total(num_scan_total).pc_points_original = [];
        scan_total(num_scan_total).pc_points = [];
        scan_total(num_scan_total).centroid = [];
        scan_total(num_scan_total).normal_vector = [];
        scan_total(num_scan_total).H = [];
        parfor pc_iter = 1:num_scan_total
%             if mod(pc_iter, 10) == 0 || pc_iter == num_scan_total || pc_iter == 1
%                 fprintf("--- Working on scan: %i/%i\n", pc_iter, num_scan_total)
%             end
            fprintf("--- Working on scan: %i/%i\n", pc_iter, num_scan_total)
            X = getPayload(pc, pc_iter, num_scan);
            opt_temp = opt;
            [X_clean, scan_total(pc_iter).clean_up] = cleanLiDARTargetWithOneDataSet(X, target_len, opt_temp);

            % cost
            opt_temp = optimizeCost(opt_temp, X_clean, target_len, scan_total(pc_iter).clean_up.std/2);
            target_lidar = [0 -target_len/2 -target_len/2 1;
                            0 -target_len/2  target_len/2 1;
                            0  target_len/2  target_len/2 1;
                            0  target_len/2 -target_len/2 1]';

            corners = opt_temp.H_opt \ target_lidar;
            corners = sortrows(corners', 3, 'descend')';
            centroid = mean(corners(1:3,:), 2);
            normals = cross(corners(1:3,1)-corners(1:3,2), corners(1:3,1)-corners(1:3,3));
            normals = normals/(norm(normals));
            if normals(1) > 0
                normals = -normals;
            end
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
            similarity_table(pc_iter).mins = sum(mink([similarity_table(pc_iter).scan(:).diff], num_pose));
        end
        save(path.save_dir + extractBetween(bag_data.bagfile,"",".bag") + '_all_scan_corners.mat');
    else
        load(path.load_dir + extractBetween(bag_data.bagfile,"",".bag") + "_all_scan_corners.mat");
    end
    
    [~, chosen_scan] = min([similarity_table(:).mins]);
    [~, chosen_scans] = mink([similarity_table(chosen_scan).scan(:).diff], num_pose);
    
    H_LT = [];
    for scan_num = 1:num_pose
        current_scan = chosen_scans(scan_num);
        bag_data.lidar_target(tag_num).scan(scan_num).corners = scan_total(current_scan).corners;
        bag_data.lidar_target(tag_num).scan(scan_num).four_corners_line = point3DToLineForDrawing(scan_total(current_scan).corners);
        bag_data.lidar_target(tag_num).scan(scan_num).pc_points_original = scan_total(current_scan).pc_points_original;
        bag_data.lidar_target(tag_num).scan(scan_num).pc_points = scan_total(current_scan).pc_points;
        bag_data.lidar_target(tag_num).scan(scan_num).centroid = scan_total(current_scan).centroid;
        bag_data.lidar_target(tag_num).scan(scan_num).normal_vector = scan_total(current_scan).normal_vector;
        bag_data.lidar_target(tag_num).scan(scan_num).H = scan_total(current_scan).H;
        H_LT = [H_LT scan_total(current_scan).H];
    end    
end

