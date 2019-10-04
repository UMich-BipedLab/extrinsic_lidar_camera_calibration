function loadBagImg(img_handle, bag_file, display, clean)
    if strcmpi("display", display)
        display = 1;
    else
        display = 0;
    end

    if nargin < 4
        clean=0;
    else
        clean = strcmpi("clean", clean);
    end
    
    if clean && isvalid(img_handle)
        img_handle.cla;
    end
    % load image
    hold(img_handle, 'on');
    bagselect = rosbag(bag_file);
    bagselect2 = select(bagselect,'Time',...
        [bagselect.StartTime bagselect.StartTime + 1],'Topic','/camera/color/image_raw');
    allMsgs = readMessages(bagselect2);
    [img,~] = readImage(allMsgs{1});
    imshow(img, 'Parent', img_handle);
    img_handle.XLim = [0 size(img,2)];
    img_handle.YLim = [0 size(img,1)];
    hold(img_handle, 'off');
    
    if display
        set(get(img_handle,'parent'),'visible','on');% show the current axes
    end
end