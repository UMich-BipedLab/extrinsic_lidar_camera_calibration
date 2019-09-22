function cost = verifyCornerAccuracyWRTDataset(num_verification, num_LiDARTag_pose, num_tag, ...
                X_verification, Y_verification, P)
    scan_verification_X = splitData(X_verification, num_verification, num_LiDARTag_pose, num_tag);
    scan_verification_Y = splitData(Y_verification, num_verification, num_LiDARTag_pose, num_tag);

    for i = 1:num_verification
        total_cost_verification = 0;
        small_tag_std = [];
        big_tag_std = [];
        mix_std = [];
        for j = 1:num_LiDARTag_pose
            current_corners_X = [scan_verification_X(j).dataset(i).corner(:).corner];
            current_corners_Y = [scan_verification_Y(j).dataset(i).corner(:).corner];
            small_tag_cost = verifyCornerAccuracy(current_corners_X(:, 1:4), current_corners_Y(:, 1:4), P);
            big_tag_cost = verifyCornerAccuracy(current_corners_X(:, 5:8), current_corners_Y(:, 5:8), P);
            total_cost_verification = total_cost_verification + small_tag_cost + big_tag_cost;
            small_tag_std = [small_tag_std small_tag_cost];
            big_tag_std = [big_tag_std big_tag_cost];
            mix_std = [mix_std small_tag_std big_tag_std];
        end
        cost(i).total_cost = total_cost_verification;
        cost(i).mix_std = std(mix_std);
        cost(i).small_tag_std = std(small_tag_std);
        cost(i).big_tag_std = std(big_tag_std);
    end
end