function showLinedAprilTag(fig_handle, camera_target, display)
    if strcmpi("display", display)
        display = 1;
    else
        display = 0;
    end
    hold(fig_handle, 'on');
    scatter(fig_handle, camera_target.corners(1,:), camera_target.corners(2,:), 20, 'yo')
    plot(fig_handle, camera_target.four_corners_line(1,:), camera_target.four_corners_line(2,:))
    
    if display
        set(get(fig_handle,'parent'),'visible','on');% show the current axes
        axis 'equal'
    end
end
