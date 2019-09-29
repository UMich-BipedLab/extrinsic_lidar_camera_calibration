function showLinedAprilTag(fig_handle, AprilTag_corners, AprilTag_corners_lines, display)
    if strcmpi("display", display)
        display = 1;
    else
        display = 0;
    end
    hold(fig_handle, 'on');
    scatter(fig_handle, AprilTag_corners(1,:), AprilTag_corners(2,:), 10, 'ro')
    plot(fig_handle, AprilTag_corners_lines(1,:), AprilTag_corners_lines(2,:))
    
    if display
        set(get(fig_handle,'parent'),'visible','on');% show the current axes
        axis 'equal'
    end
end