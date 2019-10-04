function cost = verifyCornerAccuracyWRTDataset(validation_indices, num_tag_array, opt, bag_data, P)
    for i = 1:opt.num_validation % which validation data
        current_index = validation_indices(i);
        cost_array = zeros(bag_data(current_index).num_tag, opt.num_lidar_target_pose);

        for j = 1:bag_data(current_index).num_tag % which tag in the validation dataset
            for k=1:opt.num_lidar_target_pose % which scan in the validation dataset
                current_corners_X = [bag_data(current_index).lidar_target(j).scan(k).corners];
                current_corners_Y = [bag_data(current_index).camera_target(j).corners];
                scan_cost = verifyCornerAccuracy(current_corners_X(:, 1:4), current_corners_Y(:, 1:4), P);
                cost_array(j, k) = scan_cost;
            end
        end
        cost(i).total_cost = sum(sum(cost_array, 2), 1); % total cost of this dataset
        cost(i).std = std(cost_array'); % std of cost of each scan
    end
end
