function [in, on] = getDataFromClickedPointsOnAImage(h, num_click, data, display)
    [xv, yv] = ginput(num_click);
    xq = data(1,:);
    yq = data(2,:);
    [in, on] = inpolygon(xq, yq, xv, yv);
    if checkDisplay(display)
        numel(xq(in))
        numel(xq(on))
        current_img_handle = h;
        hold(current_img_handle, 'on');
        plot(xv,yv) % polygon
        hold on
        plot(xq(in),yq(in),'r+') % points inside
        set(get(current_img_handle, 'parent'),'visible','on');% show the current axes
        axis equal
        hold(current_img_handle, 'off');
    end
end