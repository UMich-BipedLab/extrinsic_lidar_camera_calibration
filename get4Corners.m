function [LiDARTag, AprilTag] = get4Corners(opt, mat_file_path, bagfile, target_len, pc_iter, num_scan)
    pc = loadPointCloud(mat_file_path, bagfile);
    X = getPayload(pc, pc_iter, num_scan);
    
    % cost
    opt = optimizeCost(opt, X, target_len);
    target_lidar = [0 -target_len/2 -target_len/2 1;
                    0 -target_len/2  target_len/2 1;
                    0  target_len/2  target_len/2 1;
                    0  target_len/2 -target_len/2 1]';
    LiDARTag.corners = inv(opt.H_opt) * target_lidar;
    LiDARTag.corners = sortrows(LiDARTag.corners', 3, 'descend')';
    LiDARTag.four_corners_line = point3DToLineForDrawing(LiDARTag.corners);
    LiDARTag.points = X;

%             HCamera = getAprilTagPose(app, app.image_num);
    AprilTag.corners = getAprilTagCorners(bagfile);
%             AprilTag.undistorted_corners = getApriltagUndistortedCorners(app, AprilTag.corners);
    AprilTag.four_corners_line = [];
end