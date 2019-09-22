% testing code
% clc
% handle = createFigHandle(2,"test");
% plot(handle(1),[1 2 3],[2 4 6]);
% set(get(handle(1),'parent'),'visible','on');% show the current axes


function fig_handle = createFigHandle(num_handles, name)
%     fig_handle = zeros(1,num_handles);
%     fig_handle = cell(1, num_handles);
    fig_handle = [];
    for i=1:num_handles
        fig = figure('Name', name + "-" + num2str(i), ...
                               'NumberTitle', 'off', 'Visible', 'off');
        fig_handle = [fig_handle; axes('parent', fig)];
    end
end