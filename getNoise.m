function rand = getNoise(app, mu, noise, num_points)
    rand = zeros(1, num_points);
    for i = 1:num_points
        rand(i) = normrnd(mu, noise);
    end
end