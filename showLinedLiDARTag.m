function showLinedLiDARTag(image_hadle, bagfile, LiDARTag, display)
    hold(image_hadle, 'on');
%             axis(app.LiDARTagFig, 'equal');
    scatter3(image_hadle, LiDARTag.pc_points(1,:), LiDARTag.pc_points(2,:), LiDARTag.pc_points(3,:), '.')
    scatter3(image_hadle, LiDARTag.corners(1,:), LiDARTag.corners(2,:), LiDARTag.corners(3,:), 'ro')
    plot3(image_hadle, LiDARTag.four_corners_line(1,:), LiDARTag.four_corners_line(2,:), LiDARTag.four_corners_line(3,:));
    n_vec = [LiDARTag.centroid, LiDARTag.centroid + LiDARTag.normal_vector];
    plot3(image_hadle, n_vec(1, :), n_vec(2, :), n_vec(3, :));
    title(image_hadle, bagfile);
    
    if checkDisplay(display)
        set(get(image_hadle,'parent'),'visible','on');% show the current axes
        axis 'equal'
    end
end