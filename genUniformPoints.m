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