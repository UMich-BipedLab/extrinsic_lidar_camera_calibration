function [bag_data, H_LT] = getAll4CornersReturnHLT(tag_num, opt, mat_file_path, bag_data, num_scan, num_pose)
% scan_num: scan number of these corner
% num_scan: how many scans accumulated to get the corners
% function [LiDARTag, AprilTag, H_LT] = get4CornersReturnHLT(tag_num, opt, mat_file_path, pc_mat_file, bag_file, target_len, pc_iter, num_scan)
    pc = loadPointCloud(mat_file_path, bag_data.lidar_target(tag_num).pc_file);
    target_len = bag_data.lidar_target(tag_num).tag_size;
    tic
    num_scan_total = size(pc, 1)-num_scan;
   
    for pc_iter = 1:num_scan_total
        fprintf("--- Working on scan: %i/%i\n", pc_iter, num_scan_total)
        X = getPayload(pc, pc_iter, num_scan);
        opt_temp = opt;
        [X_clean, scan_total(pc_iter).scan(1)] = cleanLiDARTargetWithOneDataSet(X, target_len, opt_temp);

        % cost
        opt_temp = optimizeCost(opt_temp, X_clean, target_len, scan_total(pc_iter).scan(1).clean_up.std/2);
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
        scan_total(pc_iter).scan(1).corners = corners;
        scan_total(pc_iter).scan(1).four_corners_line = point3DToLineForDrawing(corners);
        scan_total(pc_iter).scan(1).pc_points_original = X;
        scan_total(pc_iter).scan(1).pc_points = X_clean;
        scan_total(pc_iter).scan(1).centroid = centroid;
        scan_total(pc_iter).scan(1).normal_vector = normals;
        scan_total(pc_iter).scan(1).H = inv(opt_temp.H_opt);
    end
    fprintf("Spent %f on optimizing corners of %i scans", toc, num_scan_total)
    % 
    for pc_iter = 1:num_scan_total
        for j = 1:num_scan_total
            if j == pc_iter
                similarity_table(pc_iter).scan(j).diff = 1000;
            else
                similarity_table(pc_iter).scan(j).diff = norm(logm(scan_total(j).scan(1).H \ scan_total(pc_iter).scan(1).H));
            end
        end
        similarity_table(pc_iter).mins = sum(mink([similarity_table(pc_iter).scan(:).diff], num_pose));
    end
    
    [~, chosen_scan] = min([similarity_table(:).mins]);
    [~, chosen_scans] = mink([similarity_table(chosen_scan).scan(:).diff], num_pose);
    
    H_LT = [];
    for scan_num = 1:num_pose
        current_scan = chosen_scans(scan_num);
        bag_data.lidar_target(tag_num).scan(scan_num).corners = scan_total(current_scan).scan(1).corners;
        bag_data.lidar_target(tag_num).scan(scan_num).four_corners_line = point3DToLineForDrawing(scan_total(current_scan).scan(1).corners);
        bag_data.lidar_target(tag_num).scan(scan_num).pc_points_original = scan_total(current_scan).scan(1).pc_points_original;
        bag_data.lidar_target(tag_num).scan(scan_num).pc_points = scan_total(current_scan).scan(1).pc_points;
        bag_data.lidar_target(tag_num).scan(scan_num).centroid = scan_total(current_scan).scan(1).centroid;
        bag_data.lidar_target(tag_num).scan(scan_num).normal_vector = scan_total(current_scan).scan(1).normal_vector;
        H_LT = [H_LT scan_total(current_scan).scan(1).H];
    end    
end
