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

function loadBagImg(img_handle, bag_file_path, bag_file, display, clean, camera_topic_name)
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
    bagselect = rosbag(bag_file_path + bag_file);
    bagselect2 = select(bagselect,'Time',...
        [bagselect.StartTime bagselect.StartTime + 1],'Topic', camera_topic_name);
    allMsgs = readMessages(bagselect2);
    [img,~] = readImage(allMsgs{1});
    imshow(img, 'Parent', img_handle);
    img_handle.XLim = [0 size(img,2)];
    img_handle.YLim = [0 size(img,1)];
    title(img_handle, bag_file)
    hold(img_handle, 'off');
    
    if display
        set(get(img_handle,'parent'),'visible','on');% show the current axes
    end
end
