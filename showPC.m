
% name = "velodyne_points-wavefield3-small--2019-09-07-20-18.mat";
% name = "velodyne_points-wavefield4-small--2019-09-07-19-13.mat";
name = "velodyne_points-wavefield5-big--2019-09-07-20-24.mat";
name = "velodyne_points-wavefield_3tag_closer_small--2019-10-02-11-58.mat";
% pc = load('/home/brucebot/workspace/griztag/src/matlab/matlab/LiDARTag_data/' + name);
pc = load('/home/brucebot/workspace/griztag/src/matlab/matlab/' + name);
pc_point = pc.point_cloud;


scan = 50;
pc_scan_point = (reshape(pc_point(scan, :, 1:3), size(pc_point(scan, :, 1:3), 2),[]))';
pc_scan_point = removeZeros(pc_scan_point);

pointcloud = pointCloud(pc_scan_point');
figure(3)
pcshow(pointcloud)






%%
function payload_removed_zeros = removeZeros(payload)

    for j = 1:size(payload, 2)
        if ~any(all(payload(:, j), 2))
            payload = payload(:, 1:j-1);
            break;
        end
    end
    payload_removed_zeros = payload;
end

