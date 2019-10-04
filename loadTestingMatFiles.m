function pc_all = loadTestingMatFiles(path, mat_files)
    % load pc 
    for i = 1:size(mat_files,2)
        testing_pc = loadPointCloud(path, mat_files(i));
        
        % randomly find a scan to project
        num_scan = size(testing_pc, 1);
        scan = randi([1 num_scan],1);
        
        points = testing_pc(scan, :, 1:3);
        points = (reshape(points, size(points, 2),[]))';
        for j = 1:size(points, 2)
            if ~any(all(points(:, j), 2))
                    pc_all(i).mat_pc = [points(:, 1:j-1); ...
                        ones(1, size(points(:, 1:j-1), 2))];
                break;
            end
        end
    end
end