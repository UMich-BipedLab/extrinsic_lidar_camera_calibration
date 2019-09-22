function line = point2DToLineForDrawing(points)
    line = [points(1,1), points(2,1);
            points(1,2), points(2,2);
            points(1,2), points(2,2);
            points(1,4), points(2,4);
            points(1,4), points(2,4);
            points(1,3), points(2,3);
            points(1,3), points(2,3);
            points(1,1), points(2,1)]';
end