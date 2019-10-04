function point_cloud = loadPointCloud(path, name)
    if nargin == 1
        file = path;
    else
        file = path + name;
    end
    
    if ~isfile(file)
        disp('Invalid path when loading dataset')
        fprintf("path: %s\n", file);
        return
    else
        pc = load(file);
        point_cloud = pc.point_cloud; % [scan, point, [X, Y, Z, I, R]]
    end
end