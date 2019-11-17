function showAllLinedLiDARTag(image_hadle, bagfile, LiDARTag, display)
    if strcmpi("display", display)
        display = 1;
    else
        display = 0;
    end
    hold(image_hadle, 'on');
%             axis(app.LiDARTagFig, 'equal');
% BagData(current_index).lidar_target(j).scan(i)
    for i = 1:size([LiDARTag.scan], 2)
        scatter3(image_hadle, LiDARTag.scan(i).pc_points(1,:), LiDARTag.scan(i).pc_points(2,:), LiDARTag.scan(i).pc_points(3,:), '.')
        scatter3(image_hadle, LiDARTag.scan(i).corners(1,:), LiDARTag.scan(i).corners(2,:), LiDARTag.scan(i).corners(3,:), 'ro')
        plot3(image_hadle, LiDARTag.scan(i).four_corners_line(1,:), LiDARTag.scan(i).four_corners_line(2,:), LiDARTag.scan(i).four_corners_line(3,:));
        n_vec = [LiDARTag.scan(i).centroid, LiDARTag.scan(i).centroid + LiDARTag.scan(i).normal_vector];
        plot3(image_hadle, n_vec(1, :), n_vec(2, :), n_vec(3, :));
    end
    title(image_hadle, bagfile);
    if display
        set(get(image_hadle,'parent'),'visible','on');% show the current axes
        axis 'equal'
    end
end