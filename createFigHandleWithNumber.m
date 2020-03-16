%{
 * Copyright (C) 2020-2030, The Regents of The University of Michigan.
 * All rights reserved.
 * This software was developed in the Biped Lab (https://www.biped.solutions/) 
 * under the direction of Jessy Grizzle, grizzle@umich.edu. This software may 
 * be available under alternative licensing terms; contact the address above.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of the Regents of The University of Michigan.
 * 
 * AUTHOR: Bruce JK Huang (bjhuang[at]umich.edu)
 * WEBSITE: https://www.brucerobot.com/
%}

% testing code
% clc, clear
% handle = t_createFigHandleWithNumber(2, 5,"test");
% plot(handle(1),[1 2 3],[2 4 6]);
% plot(handle(2),[10 20 30],[20 40 60]);
% set(get(handle(1),'parent'),'visible','on');% show the current axes
% set(get(handle(2),'parent'),'visible','on');% show the current axes


function fig_handle = createFigHandleWithNumber(num_handles, start_number, name)
%     fig_handle = zeros(1,num_handles);
%     fig_handle = cell(1, num_handles);
    start_number = max(1, start_number);
    fig_handle = [];
    for i = start_number : start_number+num_handles-1
        if ishandle(i)
            fig = figure(i);
            clf(fig,'reset');
            set(fig, 'Name', name + "-" + num2str(i-start_number+1), 'Visible', 'on');
        else
            fig = figure(i);
            set(fig, 'Name', name + "-" + num2str(i-start_number+1), 'Visible', 'off');
        end
        fig_handle = [fig_handle; axes('parent', fig)];
    end
end