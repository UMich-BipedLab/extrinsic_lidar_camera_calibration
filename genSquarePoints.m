function [x,y,z] = genSquarePoints(app)
    method = app.DistributionDropDown.Value;
    switch method
        case 'Uniform'
            [x, y, z] = genUniformPoints(app);
        case 'Grid'
            [x, y, z] = genGriddedPoints(app);
    end
end