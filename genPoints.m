function [x, y, z] = genPoints(method)
    switch method
        case 'Square'
            [x, y, z] = genSquarePoints(app);
        case 'Strip'
            [x, y, z] = genStripPoints(app);
        case 'Rectangle'
            [x1, y1, z1] = genSquarePoints(app);
            [~, y2, ~] = genSquarePoints(app);
            x = x1;
            y = [y1 y2];
            z = z1;
    end
end