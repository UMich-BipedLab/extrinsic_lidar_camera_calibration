%{
 * Copyright (C) 2020-2030, The Regents of The University of Michigan.
 * All rights reserved.
 * This software was developed in the Biped Lab (https://www.biped.solutions/) 
 * under the direction of Jessy Grizzle, grizzle@umich.edu. This software may 
 * be available under alternative licensing terms; contact the address above.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of the Regents of The University of Michigan.
 * 
 * AUTHOR: Bruce JK Huang (bjhuang[at]umich.edu)
 * WEBSITE: https://www.brucerobot.com/
%}

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
        case 'The Wall'
            opt.metric = "PointToAxis";
            opt.unit = "L1-inspired";
            tic;
            opt = optimizeBoundaries(opt, opts, X);
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