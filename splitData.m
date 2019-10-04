function scan = splitData(X, num_dataset, num_scan, num_LiDARTag_pose)
    num_corner = 4;
    num_total_corner = 4 * num_LiDARTag_pose * num_scan * num_dataset;
    num_switch_scan = num_dataset*num_corner*num_LiDARTag_pose;
    num_skip = num_corner * num_LiDARTag_pose; % jump to the same corner in the same dataset
    for i = 1:num_scan
        for j = 1:num_dataset
            for k = 1:num_skip
                current_corner = num_switch_scan * (i-1) + num_skip*(j-1) + k;
                scan(i).dataset(j).corner(k).corner = X(:, current_corner);
            end
        end
    end
end
