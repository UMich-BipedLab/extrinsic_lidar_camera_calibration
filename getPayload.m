function accumulated_payload = getPayload(point_cloud, pc_iter, num_scan)

    accumulated_payload = [];
    for i = 0:num_scan-1
        points = point_cloud(pc_iter+i, :, 1:3);
        points = (reshape(points, size(points, 2),[]))';
        for i = 1:size(points, 2)
            if ~any(all(points(:, i), 2))
                points = points(:, 1:i-1);
                break;
            end
        end
        accumulated_payload = [accumulated_payload points];
    end
    accumulated_payload = [accumulated_payload; ones(1, size(accumulated_payload, 2))];
end