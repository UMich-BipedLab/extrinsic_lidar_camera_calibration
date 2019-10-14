function opt = optimizeCost(opt, X, target_size, box_width)

    R_init = rotx(opt.rpy_init(1)) * roty(opt.rpy_init(2)) * rotz(opt.rpy_init(3));
    opt.H_init(1:3, 1:3) = R_init;

    if opt.UseCentroid
        centroid = mean(X, 2);
        opt.H_init(1:3, 4) = -centroid(1:3);
    else
        opt.H_init(1:3, 4) = opt.T_init;
    end

    switch opt.method
        case 'Customize'
            opt.metric = "PointToAxis";
            opt.unit = "L1-inspired";
            tic;
            optimizeCustomizedCost(opt);
            opt.computation_time = toc;
        case 'Constraint Customize'
            opt.metric = "PointToAxis";
            opt.unit = "L1-inspired";
            tic;
            opt = optimizeConstraintCustomizedCost(opt, X, target_size, box_width);
            opt.computation_time = toc;
        case 'Coherent Point Drift'
            opt.metric = "--";
            opt.unit = "RMSE";
            tic;
            optimizeCPD(opt);
            opt.computation_time = toc;
        case 'Iterative Closest Point (point)'
            opt.metric = "PointToPoint";
            opt.unit = "RMSE";
            tic;
            optimizeICPPoint(opt);
            opt.computation_time = toc;
        case 'Iterative Closest Point (plane)'
            opt.metric = "PointToPlane";
            opt.unit = "RMSE";
            tic;
            optimizeICPPlane(opt);
            opt.computation_time = toc;
        case 'Normal-distributions Transform'
            opt.metric = "--";
            opt.unit = "RMSE";
            tic;
            optimizeNDT(opt);
            opt.computation_time = toc;
        case 'GICP-SE3'
            tic;
            optimizeGICP_SE3(opt)
            opt.computation_time = toc;
        case 'GICP-SE3 (plane)'
            tic;
            optimizeGICPPlane_SE3(opt)
            opt.computation_time = toc;
        case 'GICP-SE3-costimized'
            tic;
            optimizeCustomizedGICP_SE3(opt);
            opt.computation_time = toc;
        case 'Two Hollow Strips'
            tic;
            optimizeHollowStrips(opt);
            opt.computation_time = toc;
        case '3D IoU'
        case 'Project'
            tic;
            OptimizeUsingRobustNormalVector(opt);
            opt.computation_time = toc;
        otherwise
            disp('no such optimization method')
            return
    end
end