function [x,y,z] = genStripPoints(app)
    method = app.DistributionDropDown.Value;
    switch method
        case 'Uniform'
            [x, y, z] = genUniformPoints(app);
        case 'Grid'
            [x, y, z] = genGriddedPoints(app);
    end
    % overwrite noie-free points to strip-like shape
    genStrippedPoints(app, app.target_len_);
end