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

function showAllLinedLiDARTag(image_hadle, bagfile, LiDARTag, display)
    if strcmpi("display", display)
        display = 1;
    else
        display = 0;
    end
    hold(image_hadle, 'on');
%             axis(app.LiDARTagFig, 'equal');
% BagData(current_index).lidar_target(j).scan(i)
    for i = 1:size([LiDARTag.scan], 2)
        scatter3(image_hadle, LiDARTag.scan(i).pc_points(1,:), LiDARTag.scan(i).pc_points(2,:), LiDARTag.scan(i).pc_points(3,:), '.')
        scatter3(image_hadle, LiDARTag.scan(i).corners(1,:), LiDARTag.scan(i).corners(2,:), LiDARTag.scan(i).corners(3,:), 'ro')
        plot3(image_hadle, LiDARTag.scan(i).four_corners_line(1,:), LiDARTag.scan(i).four_corners_line(2,:), LiDARTag.scan(i).four_corners_line(3,:));
        n_vec = [LiDARTag.scan(i).centroid, LiDARTag.scan(i).centroid + LiDARTag.scan(i).normal_vector];
        plot3(image_hadle, n_vec(1, :), n_vec(2, :), n_vec(3, :));
    end
    title(image_hadle, bagfile);
    if display
        set(get(image_hadle,'parent'),'visible','on');% show the current axes
        axis 'equal'
    end
end