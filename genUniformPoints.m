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


function [x, y, z] = genUniformPoints(app)
    % Gaussian noise
    % rand_z = getNoise(app, app.noise_mu_(3), app.noise_sigma_(3), app.num_points_);
    % rand_y = getNoise(app, app.noise_mu_(2), app.noise_sigma_(2), app.num_points_);
    % rand_x = getNoise(app, app.noise_mu_(1), app.noise_sigma_(1), app.num_points_);

    % uniform noise
    rand_x =  app.tag_size_ * app.noise_sigma_(1) + ...
        (-app.tag_size_ * app.noise_sigma_(1) - app.tag_size_ * app.noise_sigma_(1) ) * ...
        rand(1, app.num_points_);
    rand_y =  app.tag_size_ * app.noise_sigma_(2) + ...
        (-app.tag_size_ * app.noise_sigma_(2) - app.tag_size_ * app.noise_sigma_(2) ) * ...
        rand(1, app.num_points_);
    rand_z =  app.tag_size_ * app.noise_sigma_(3) + ...
        (-app.tag_size_ * app.noise_sigma_(3) - app.tag_size_ * app.noise_sigma_(3) ) * ...
        rand(1, app.num_points_);


    % generate two sets of points seperately
    % noise free points
    %             noise_free_z = rand(1, app.num_points_target) * app.tag_size_ - app.tag_size_/2;
    %             noise_free_y = rand(1, app.num_points_target) * app.tag_size_ - app.tag_size_/2;
    %             noise_free_x = rand(1, app.num_points_target) * 0.0001;
    %             app.noise_free_points_ = [noise_free_x;
    %                                       noise_free_y;
    %                                       noise_free_z;
    %                                       ones(1, app.num_points_target)];
    %
    %             % noisy data
    %             z = rand(1, app.num_points_) * app.tag_size_ - app.tag_size_/2  + rand_z;
    %             y = rand(1, app.num_points_) * app.tag_size_ - app.tag_size_/2  + rand_y;
    %             x = rand(1, app.num_points_) * 0.0001        + rand_x;
    noise_free_z = rand(1, app.num_points_) * app.tag_size_ - app.tag_size_/2;
    noise_free_y = rand(1, app.num_points_) * app.tag_size_ - app.tag_size_/2;
    noise_free_x = rand(1, app.num_points_) * 0.0001;
    app.noise_free_points_ = [noise_free_x;
        noise_free_y;
        noise_free_z;
        ones(1, app.num_points_)];

    % noisy data
    z = noise_free_z + rand_z;
    y = noise_free_y + rand_y;
    x = noise_free_x + rand_x;
end