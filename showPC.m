%{
 * Copyright (C) 2013-2020, The Regents of The University of Michigan.
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


% name = "velodyne_points-wavefield3-small--2019-09-07-20-18.mat";
% name = "velodyne_points-wavefield4-small--2019-09-07-19-13.mat";
name = "velodyne_points-wavefield5-big--2019-09-07-20-24.mat";
name = "velodyne_points-wavefield_3tag_closer_small--2019-10-02-11-58.mat";
% pc = load('/home/brucebot/workspace/griztag/src/matlab/matlab/LiDARTag_data/' + name);
pc = load('/home/brucebot/workspace/griztag/src/matlab/matlab/' + name);
pc_point = pc.point_cloud;


scan = 50;
pc_scan_point = (reshape(pc_point(scan, :, 1:3), size(pc_point(scan, :, 1:3), 2),[]))';
pc_scan_point = removeZeros(pc_scan_point);

pointcloud = pointCloud(pc_scan_point');
figure(3)
pcshow(pointcloud)






%%
function payload_removed_zeros = removeZeros(payload)

    for j = 1:size(payload, 2)
        if ~any(all(payload(:, j), 2))
            payload = payload(:, 1:j-1);
            break;
        end
    end
    payload_removed_zeros = payload;
end

