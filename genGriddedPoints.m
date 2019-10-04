function [x,y,z] = genGriddedPoints(app)
    % contruct grid on the target frame
    %             n = ceil(sqrt(app.num_points_target));
    %             y = linspace(0, app.target_len_, n);
    %             z = linspace(0, app.target_len_, n);
    %             app.noise_free_points_ = zeros(4, n^2); % x
    %
    %             for i = 1:n
    %                 for j = 1:n
    %                     app.noise_free_points_(2,(i-1)*n + j) = y(j) - app.target_len_/2;
    %                     app.noise_free_points_(3,(i-1)*n + j) = z(i) - app.target_len_/2;
    %                 end
    %             end
    n = ceil(sqrt(app.num_points_target));
    [y, z] = genGrid(app, n, app.target_len_);
    app.noise_free_points_ = zeros(4, n^2); % x
    app.noise_free_points_(2,:) = y;
    app.noise_free_points_(3,:) = z;
    app.noise_free_points_(4,:) = ones(1 ,n^2);

    % mimic LiDAR's performance
    rand_z = getNoise(app, 0, app.noise_sigma_(3), n^2); % ring
    rand_y = getNoise(app, 0, app.noise_sigma_(2), n^2); % noise on ring
    rand_x = getNoise(app, 0, app.noise_sigma_(1), n^2); % noise on depth

    % noisy data
    z = app.noise_free_points_(3,:) + rand_z;
    y = app.noise_free_points_(2,:) + rand_y;
    x = app.noise_free_points_(1,:) + rand_x;
end