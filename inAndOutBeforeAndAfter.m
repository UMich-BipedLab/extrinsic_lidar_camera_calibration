function [before_scan, after_scan] = inAndOutBeforeAndAfter(X, Y, P1, P2, ...
                num_verification, num_LiDARTag_pose, num_tag)
%             correspondance_per_scan = 4;
%             num_scan = size(X, 2)/correspondance_per_scan; % 4 correspondance per scan
    scan_verification_Y = splitData(Y, num_verification, num_LiDARTag_pose, num_tag);

    before_scan = [];
    after_scan = [];
     for i = 1:num_verification     

        before_count = 0;
        after_count = 0;
        size_big = 0;
        size_small = 0;
        for j = 1:num_LiDARTag_pose
            current_X_small = [X(j).dataset(i).payload_small];
            if size(current_X_small, 1) ~= 4
                current_X_small = [current_X_small; ones(1,size(current_X_small,2))];
            end
            
            size_small = size_small + size(current_X_small, 2);
            current_corners_small_Y = [scan_verification_Y(j).dataset(i).corner(1:4).corner];
            current_corners_small_Y = [current_corners_small_Y(:,1), current_corners_small_Y(:,2), current_corners_small_Y(:,4), current_corners_small_Y(:,3)];
            x_before_small = P1 * current_X_small;
            x_before_small = x_before_small ./x_before_small(3,:);
            [in_before_small, on_before_small] = inpolygon(x_before_small(1,:)', x_before_small(2,:)', ...
                current_corners_small_Y(1,:)' ,current_corners_small_Y(2,:)');

            x_after_small = P2 * current_X_small;
            x_after_small = x_after_small ./x_after_small(3,:);
            [in_after_small, on_after_small] = inpolygon(x_after_small(1,:)', x_after_small(2,:)', current_corners_small_Y(1,:)' ,current_corners_small_Y(2,:)');

            current_X_big = [X(j).dataset(i).payload_big];
            
            if size(current_X_big, 1) ~= 4
                current_X_big = [current_X_big; ones(1,size(current_X_big,2))];
            end
            
            size_big = size_big + size(current_X_big, 2);
            current_corners_big_Y = [scan_verification_Y(j).dataset(i).corner(5:8).corner];
            current_corners_big_Y = [current_corners_big_Y(:,1), current_corners_big_Y(:,2), current_corners_big_Y(:,4), current_corners_big_Y(:,3)];
            x_before_big = P1 * current_X_big;
            x_before_big = x_before_big ./x_before_big(3,:);
            [in_before_big, on_before_big] = inpolygon(x_before_big(1,:)', x_before_big(2,:)', ...
                current_corners_big_Y(1,:)' ,current_corners_big_Y(2,:)');

            x_after_big = P2 * current_X_big;
            x_after_big = x_after_big ./x_after_big(3,:);
            [in_after_big, on_after_big] = inpolygon(x_after_big(1,:)', x_after_big(2,:)', current_corners_big_Y(1,:)' ,current_corners_big_Y(2,:)');

            before_count = before_count + numel(x_before_small(in_before_small)) + numel(x_before_small(on_before_small)) + numel(x_before_big(in_before_big)) + numel(x_before_big(on_before_big));
            after_count = after_count + numel(x_after_small(in_after_small)) + numel(x_after_small(on_after_small)) + numel(x_after_big(in_after_big)) + numel(x_after_big(on_after_big));
        end
        ave_count_before = (size_big + size_small - before_count)/(size_big + size_small);
        ave_count_after = (size_big + size_small - after_count)/(size_big + size_small);
        before_scan = [before_scan, ave_count_before];
        after_scan = [after_scan, ave_count_after];
     end