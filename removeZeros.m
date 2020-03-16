function payload_removed_zeros = removeZeros(payload)
    for j = 1:size(payload, 2)
        if ~any(all(payload(:, j), 2))
            payload = payload(:, 1:j-1);
            break;
        end
    end
    payload_removed_zeros = payload;
end

