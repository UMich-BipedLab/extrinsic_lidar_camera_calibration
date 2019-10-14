function showLinedLiDARTag(image_hadle, bag_file, LiDARTag, display)
    if strcmpi("display", display)
        display = 1;
    else
        display = 0;
    end
    hold(image_hadle, 'on');
%             axis(app.LiDARTagFig, 'equal');
    
    scatter3(image_hadle, LiDARTag.pc_points(1,:), LiDARTag.pc_points(2,:), LiDARTag.pc_points(3,:), '.')
    scatter3(image_hadle, LiDARTag.corners(1,:), LiDARTag.corners(2,:), LiDARTag.corners(3,:), 'ro')
    plot3(image_hadle, LiDARTag.four_corners_line(1,:), LiDARTag.four_corners_line(2,:), LiDARTag.four_corners_line(3,:));
    arrow = [LiDARTag.centroid (LiDARTag.normal_vector + LiDARTag.centroid)]; % draw arrow
    plot3(image_hadle, arrow(1,:), arrow(2,:), arrow(3,:))
    title(image_hadle, bag_file)
    if display
        set(get(image_hadle,'parent'),'visible','on');% show the current axes
        axis 'equal'
    end
end
