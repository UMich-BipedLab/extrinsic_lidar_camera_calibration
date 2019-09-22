function prjectBackToImage(image_handle, P, X, marker_size , marker_color, legend, display, clean)
    if size(X,1) ~= 4
        X = X';
    end
    if nargin < 7
        clean=0;
    else
        clean = strcmpi("clean", clean);
    end

    if clean && isvalid(img_handle)
        img_handle.cla;
    end
    
    if strcmpi("display", display)
        display = 1;
    else
        display = 0;
    end
    
    hold(image_handle, 'on');
    projected_points = P * X;
    projected_points = projected_points ./ projected_points(3,:);
%             projected_points = distortedToUndistorted(app, projected_points(1:2,:)');
    scatter(image_handle, projected_points(1,:), projected_points(2,:), marker_size, marker_color, 'DisplayName', legend)
    hold(image_handle, 'off');
    if display
        set(get(image_handle,'parent'),'visible','on');% show the current axes
    end
end