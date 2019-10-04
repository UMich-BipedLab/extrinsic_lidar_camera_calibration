function [bag_data, H_LT] = get4CornersReturnHLT(scan_num, tag_num, opt, mat_file_path, bag_data, pc_iter, num_scan)
% scan_num: scan number of these corner
% num_scan: how many scans accumulated to get the corners
% function [LiDARTag, AprilTag, H_LT] = get4CornersReturnHLT(tag_num, opt, mat_file_path, pc_mat_file, bag_file, target_len, pc_iter, num_scan)
    pc = loadPointCloud(mat_file_path, bag_data.lidar_target(tag_num).pc_file);
    X = getPayload(pc, pc_iter, num_scan);
    
    % cost
    target_len = bag_data.lidar_target(tag_num).tag_size;
    opt = optimizeCost(opt, X, target_len);
    target_lidar = [0 -target_len/2 -target_len/2 1;
                    0 -target_len/2  target_len/2 1;
                    0  target_len/2  target_len/2 1;
                    0  target_len/2 -target_len/2 1]';
    corners = inv(opt.H_opt) * target_lidar;
    corners = sortrows(corners', 3, 'descend')';
    bag_data.lidar_target(tag_num).scan(scan_num).corners = corners;
    bag_data.lidar_target(tag_num).scan(scan_num).four_corners_line = point3DToLineForDrawing(corners);
    bag_data.lidar_target(tag_num).scan(scan_num).pc_points = X;

%             HCamera = getAprilTagPose(app, app.image_num);
    % AprilTag.corners = getAprilTagCorners(bag_file, tag_num);
%             AprilTag.undistorted_corners = getApriltagUndistortedCorners(app, AprilTag.corners);
    % AprilTag.four_corners_line = [];
    H_LT = inv(opt.H_opt);
end
