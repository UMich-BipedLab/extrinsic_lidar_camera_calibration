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

function image_data = refineImageRightCorner(opts, image_data, display)
    extend_fac = 2;
%     corner_array = [1 1 2 3
%                     2 3 4 4];
    corner_array = [1 1;
                    2 3];

    bagselect = rosbag(image_data.path + image_data.name);
    bagselect2 = select(bagselect,'Time',...
        [bagselect.StartTime bagselect.StartTime + 1],'Topic','/camera/color/image_raw');
    allMsgs = readMessages(bagselect2);
    [img,~] = readImage(allMsgs{1});
    gray = rgb2gray(img);
    BW = edge(gray, 'Canny', [0.04]);

    if checkDisplay(display)
        figure(1000)
        clf('reset')
        hold on
        imshow(img)
        title(image_data.name)

        figure(2000)
        clf('reset')
        hold on
        imshow(BW)
        title(image_data.name)
    end

    for i = 1: length(corner_array)
        p1 = image_data.camera_target.corners(1:2, corner_array(1,i));
        p2 = image_data.camera_target.corners(1:2, corner_array(2,i));

        vec = [p1 - p2];
        vec_normlized = vec/norm(vec);
        vec_p = [vec_normlized(2); -vec_normlized(1)];
        p1_ext = p1 + extend_fac * vec_normlized + extend_fac * vec_p;
        p2_ext = p2 - extend_fac * vec_normlized + extend_fac * vec_p;

        p3_ext = p2 - extend_fac * vec_normlized - extend_fac * vec_p;
        p4_ext = p1 + extend_fac * vec_normlized - extend_fac * vec_p;


        corners = [p1_ext, p2_ext, p3_ext, p4_ext];
        [x2, y2] = poly2cw(corners(1,:)', corners(2,:)');
        [img_y, img_x] = size(gray);
        x_dim = linspace(1, img_x, img_x);
        x_dim = repmat(x_dim,1,img_y);
        y_dim = linspace(1, img_y, img_y);
        y_dim = repelem(y_dim,1, img_x);

        [in, ~] = inpolygon(x_dim, y_dim, x2, y2);
        x_dim = x_dim(in);
        y_dim = y_dim(in);

        data_current = [];
        for t = 1:size(x_dim, 2)
            if BW(y_dim(t), x_dim(t))
                data_current = [data_current, [x_dim(t); y_dim(t)]];
            end
        end
        [x, y, line_model, inlier_pts] = ransacLineWithInlier(data_current', 0.1);

        if checkDisplay(display)
            figure(2000)
            hold on
            scatter(image_data.camera_target.corners(1,:), image_data.camera_target.corners(2,:))
            scatter(p1(1), p1(2))
            scatter(p2(1), p2(2))
            scatter(p1_ext(1), p1_ext(2))
            scatter(p2_ext(1), p2_ext(2))
            scatter(p3_ext(1), p3_ext(2))
            scatter(p4_ext(1), p4_ext(2))
            scatter(x_dim, y_dim, 'g.');
            scatter(inlier_pts(:,1), inlier_pts(:,2), 'm.');
            plot(x, y, '-', 'LineWidth',2, 'MarkerSize',10, 'color', [0.8500, 0.3250, 0.0980])

            figure(1000)
            hold on 
            scatter(image_data.camera_target.corners(1,:),image_data.camera_target.corners(2,:))
            scatter(p1(1), p1(2))
            scatter(p2(1), p2(2))
            plot(x, y, '-', 'LineWidth',2, 'MarkerSize',10, 'color', [0.8500, 0.3250, 0.0980])
        end

        t_edge(i).x = x;
        t_edge(i).y = y;
        t_edge(i).line = line_model;
        image_data.camera_target.lines(i).x = x;
        image_data.camera_target.lines(i).y = y;
        image_data.camera_target.lines(i).line = line_model;
    end

    cross_big_2d = [];
    for i = 1: opts.num_corner
        point = intersection(t_edge(corner_array(1,i)).line, ...
                             t_edge(corner_array(2,i)).line);
        cross_big_2d = [cross_big_2d, point];
    end
    cross_big_2d = sortrows(cross_big_2d', 2, 'ascend')';
    cross_big_2d = [cross_big_2d; ones(1, size(cross_big_2d,2))];
    image_data.camera_target.corners = cross_big_2d;

    if checkDisplay(display)
%                 disp("Afrer modification")
%                 disp([BagData(k).camera_target(j).corners])
        figure(2000)
        scatter(cross_big_2d(1,:), cross_big_2d(2,:), 'filled');
        hold off

        figure(1000)
        scatter(cross_big_2d(1,:), cross_big_2d(2,:), 'g','filled');
        hold off
        drawnow
    end
    if checkDisplay(display)
%             disp("press any key to continue")
%             pause;
%             clc
    end
end
