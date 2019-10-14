function [bag_data, H_LT] = get4CornersReturnHLT(scan_num, tag_num, opt, mat_file_path, bag_data, pc_iter, num_scan)
% scan_num: scan number of these corner
% num_scan: how many scans accumulated to get the corners
% function [LiDARTag, AprilTag, H_LT] = get4CornersReturnHLT(tag_num, opt, mat_file_path, pc_mat_file, bag_file, target_len, pc_iter, num_scan)
    pc = loadPointCloud(mat_file_path, bag_data.lidar_target(tag_num).pc_file);
    X = getPayload(pc, pc_iter, num_scan);
    target_len = bag_data.lidar_target(tag_num).tag_size;
    
    % clean data
%     meanData = mean(X(1:3, :), 2);
%     error = abs(X(1:3, :) - meanData);
%     distance = sum(error, 1);
%     K = find(distance < target_len*1.015);
%     X_clean = X(:, K);
%     meanClean=mean(X_clean(1:3, :), 2);
    [X_clean, bag_data] = cleanLiDARTarget(scan_num, tag_num, bag_data, X, target_len, opt);

    % cost
    opt = optimizeCost(opt, X_clean, target_len, bag_data.lidar_target(tag_num).scan(scan_num).clean_up.std/2);
    target_lidar = [0 -target_len/2 -target_len/2 1;
                    0 -target_len/2  target_len/2 1;
                    0  target_len/2  target_len/2 1;
                    0  target_len/2 -target_len/2 1]';
    
    corners = inv(opt.H_opt) * target_lidar;
    corners = sortrows(corners', 3, 'descend')';
    centroid = mean(corners(1:3,:), 2);
    normals = cross(corners(1:3,1)-corners(1:3,2), corners(1:3,1)-corners(1:3,3));
    normals = normals/(norm(normals));
    if normals(1) > 0
        normals = -normals;
    end
    
    bag_data.lidar_target(tag_num).scan(scan_num).corners = corners;
    bag_data.lidar_target(tag_num).scan(scan_num).four_corners_line = point3DToLineForDrawing(corners);
    bag_data.lidar_target(tag_num).scan(scan_num).pc_points_original = X;
    bag_data.lidar_target(tag_num).scan(scan_num).pc_points = X_clean;
    bag_data.lidar_target(tag_num).scan(scan_num).centroid = centroid;
    bag_data.lidar_target(tag_num).scan(scan_num).normal_vector = normals;


%             HCamera = getAprilTagPose(app, app.image_num);
    % AprilTag.corners = getAprilTagCorners(bag_file, tag_num);
%             AprilTag.undistorted_corners = getApriltagUndistortedCorners(app, AprilTag.corners);
    % AprilTag.four_corners_line = [];
    H_LT = inv(opt.H_opt);
end
