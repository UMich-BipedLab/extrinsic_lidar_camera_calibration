%{
 * Copyright (C) 2013-2020, The Regents of The University of Michigan.
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

clc, clear, close all
%%%%%%%%%%%%%%%%%%%%%
%%% camera parameters
%%%%%%%%%%%%%%%%%%%%%
intrinsic_matrix = [616.3681640625, 0.0,            319.93463134765625;
                    0.0,            616.7451171875, 243.6385955810547;
                    0.0, 0.0, 1.0];
distortion_param = [0.099769, -0.240277, 0.002463, 0.000497, 0.000000];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% parameters of user setting
%%% skip (0/1/2):
%        0: optimize lidar target's corners 
%           and then calibrate 
%        1: skip optimize lidar target's corners
%        2: just shown calibration results
%%% more_analysis (0/1): print more stuff at the end
%%% validation_flag (0/1): validate the calibration result if one has
%%% base_line_method (1/2): 
%                   1: ransac edges seperately and the intersect edges to
%                      estimate corners
%                   2: apply geometry contrain to estimate the corners
%%% calibration_method: 
%                     "4 points"
%                     "IoU"
%%% path.load_dir: directory of saved files
%%% load_all_vertices: pre-calculated vertices (pick the top-5 consistent)
%%% bag_file_path: bag files of images 
%%% mat_file_path: mat files of extracted lidar target's point clouds
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
opts.optimizeAllCorners = 0;
opts.use_top_consistent_vertices = 1;
skip = 0; 
more_analysis = 0;
validation_flag = 1;
base_line_method = 2;
calibration_method = "4 points";
load_dir = "Paper-C71/06-Oct-2019 13:53:31/";
path.load_dir = "NewPaper/04-Nov-2019 14:59:07/";
path.load_all_vertices = "NewPaper/16-Nov-2019 14:53:58/";
path.bag_file_path = "/home/brucebot/workspace/griztag/src/griz_tag/bagfiles/matlab/";
path.mat_file_path = "../../LiDARTag_data/";

% save into results into folder         
path.save_name = "NewPaper";
diary Debug % save terminal outputs

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% show figures
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
show_image_refinement = 0;
show_pnp_numerical_result = 0; % show numerical results
show_lidar_target = 0;
% show.lidar_target_optimization = 1;
show_camera_target = 0;
show_training_results = 0; % 1
show_validation_results = 0; %1 
show_testing_results = 0; %1

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% parameters for optimization of lidar targets
% num_refinement: how many rounds of refinement
% num_lidar_target_pose: how many lidar target poses to optimize H_LC 
% num_scan: accumulate how many scans to optimize a lidar target's corners
% correspondance_per_pose (int): how many correspondance on a target
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
opts.num_refinement = 5 ; % 4 rounds of refinement
opts.num_lidar_target_pose = 5; % how many LiDARTag poses to optimize H_LC (5) (2)
opts.num_scan = 5; % how many scans accumulated to optimize one LiDARTag pose (3)
opts.correspondance_per_pose = 4; % 4 correspondance on a target

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% We have tried several methods to recover the unobserable lidar target's 
%%% corners, those will be added soon
% method:
%        Constraint Customize: Using proposed method stated in the paper
%        Customize: coming soon
%        Coherent Point Drift: coming soon
%        Iterative Closest Point (point): coming soon
%        Iterative Closest Point (plane): coming soon
%        Normal-distributions Transform: coming soon
%        GICP-SE3: coming soon
%        GICP-SE3 (plane): coming soon
%        GICP-SE3-costimized: coming soon
%        Two Hollow Strips: coming soon
%        Project: coming soon
%%% optimization parameters
%   H_TL: optimization for LiDAR target to ideal frame to get corners
%   H_LC: optimization for LiDAR to camera transformation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
opt.H_TL.rpy_init = [45 2 3];
opt.H_TL.T_init = [2, 0, 0];
opt.H_TL.H_init = eye(4);
opt.H_TL.method = "Constraint Customize"; 
opt.H_TL.UseCentroid = 1;
opt.H_LC.rpy_init = [90 0 90];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% training, validation and testing datasets
%%% random_select (0/1): randomly select training sets
%%% trained_ids: a list of ids of training sets
% training sets (targets included): 
%  -- used all of them to optimize a H_LC
% validation sets (targets included): 
%  -- used the optimized H_LC to validate the results
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
random_select = 0;
trained_ids = [4]; % 3,10,11
skip_indices = [1, 2, 3, 12]; %% skip non-standard 
[BagData, TestData] = getBagData();
bag_with_tag_list  = [BagData(:).bagfile];
bag_testing_list = [TestData(:).bagfile];
test_pc_mat_list = [TestData(:).pc_file];
opts.num_training = length(trained_ids); 
opts.num_validation = length(bag_with_tag_list) - length(skip_indices) - opts.num_training;    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

total_num_dataset = length([BagData(:).bagfile]);
calibration = parforInit(total_num_dataset, opts);
c = datestr(datetime); 
path.save_dir = path.save_name + "/" + c + "/";
mkdir(path.save_dir);
    

if skip == 0
    parfor index = 1:total_num_dataset
    disp("Refining corners of camera targets ...")
    [BagData, TestData] = getBagData();
    BagData = refineImageCorners(path.bag_file_path, BagData, skip_indices, show_image_refinement);
    if  ismember(index, skip_indices)
        continue
    else
        trained_ids = index; 
    end

    % overwrite
    bag_training_indices = trained_ids;
    bag_validation_indices = linspace(1, length(bag_with_tag_list), length(bag_with_tag_list));
    bag_validation_indices([trained_ids skip_indices]) = [];
    bag_chosen_indices = [bag_training_indices, bag_validation_indices];

    % create figure handles
    training_img_fig_handles = createFigHandle(opts.num_training, "training_img");
    training_pc_fig_handles = createFigHandle(opts.num_training, "training_pc");
    validation_fig_handles = createFigHandle(opts.num_validation, "validation_img");
    validation_pc_fig_handles = createFigHandle(opts.num_validation, "validation_pc");
    testing_fig_handles = createFigHandle(size(bag_testing_list, 2), "testing");

    ans_error_big_matrix = [];
    ans_counting_big_matrix = [];

    disp("********************************************")
    disp(" Chosen dataset")
    disp("********************************************")
    disp("-- Skipped: ")
    disp(bag_with_tag_list(skip_indices))
    disp("-- Training set: ")
    disp(bag_with_tag_list(bag_training_indices))            
    disp("-- Validation set: ")
    disp([bag_with_tag_list(bag_validation_indices)])
    disp("-- Chosen set: ")
    disp(bag_with_tag_list(bag_chosen_indices))

    disp("********************************************")
    disp(" Chosen parameters")
    disp("********************************************")
    fprintf("-- validation flag: %i \n", validation_flag)
    fprintf("-- number of training set: %i\n", size(bag_training_indices, 2))
    fprintf("-- number of validation set: %i\n", size(bag_validation_indices, 2))
    fprintf("-- number of refinement: %i\n", opts.num_refinement)
    fprintf("-- number of LiDARTag's poses: %i\n", opts.num_lidar_target_pose)
    fprintf("-- number of scan to optimize a LiDARTag pose: %i\n", opts.num_scan)


    % loading training image
    for k = 1:opts.num_training
        current_index = bag_training_indices(k);
        loadBagImg(training_img_fig_handles(k), path.bag_file_path, bag_with_tag_list(current_index), "not display", "not clean");

        if skip==1 || skip == 2
            for j = 1:BagData(current_index).num_tag
                for i = 1:size(BagData(current_index).lidar_target(j).scan(:))
                    showLinedLiDARTag(training_pc_fig_handles(k), ...
                                      BagData(current_index).bagfile, ...
                                      BagData(current_index).lidar_target(j).scan(i), show_lidar_target);
                    showLinedAprilTag(training_img_fig_handles(k), ...
                                      BagData(current_index).camera_target(j), show_camera_target);
                end
            end
        end
    end

    if validation_flag
        for k = 1:opts.num_validation
            current_index = bag_validation_indices(k);
            loadBagImg(validation_fig_handles(k), path.bag_file_path, bag_with_tag_list(current_index), "not display", "not clean");

            if skip==1 || skip == 2
                for j = 1:BagData(current_index).num_tag
                    for i = 1:size(BagData(current_index).lidar_target(j).scan(:))
                        showLinedLiDARTag(validation_pc_fig_handles(k), ...
                                          BagData(current_index).bagfile, ...
                                          BagData(current_index).lidar_target(j).scan(i), show_lidar_target);
                        showLinedAprilTag(validation_fig_handles(k), ...
                                          BagData(current_index).camera_target(j), show_camera_target);
                    end
                end
            end
        end
    end


    if skip == 0
        disp("********************************************")
        disp(" Optimizing LiDAR Target Corners")
        disp("********************************************")
        X_train = []; % training corners of lidar targets in 3D
        Y_train = []; % training corners of image targets in 2D
        train_num_tag_array = []; % number of tag in each training data (need to be used later)
        train_tag_size_array = []; % size of tag in each training data (need to be used later)
        validation_num_tag_array = []; % number of tag in each training data (need to be used later)
        validation_tag_size_array = []; % size of tag in each training data (need to be used later)
        X_validation = []; % validation corners of lidar targets in 3D
        Y_validation = []; % validation corners of image targets in 2D
        H_LT_big = [];
        X_base_line_edge_points = [];
        X_base_line = [];
        Y_base_line = [];
        N_base_line = [];
        validation_counter = 1;
        training_counter = 1;

        for k = 1:length(bag_chosen_indices)
            current_index = bag_chosen_indices(k);
            fprintf("Working on %s -->", bag_with_tag_list(current_index))
            % skip undesire index
            if any(ismember(current_index, skip_indices))
                continue
            end
            % if don't want to get validation set, skip
            % everything else but the traing set
            if ~validation_flag
                if ~any(ismember(bag_training_indices, current_index))
                    continue;
                end
            end
            % training set
            if any(ismember(bag_training_indices, current_index))
                X_training_tmp = [];
                Y_training_tmp = [];
                H_LT_tmp = [];

                for j = 1:BagData(current_index).num_tag
                    fprintf("----Tag %i/%i", j, BagData(current_index).num_tag)
                    % optimize lidar targets corners
                    [BagData(current_index), H_LT] = getAll4CornersReturnHLT(j, opt, ...
                                                         path, BagData(current_index), ...
                                                         opts);
                    % draw camera targets 
                    BagData(current_index).camera_target(j).four_corners_line = ...
                                                point2DToLineForDrawing(BagData(current_index).camera_target(j).corners);
                    showAllLinedLiDARTag(training_pc_fig_handles(training_counter), ...
                                         BagData(current_index).bagfile, ...
                                         BagData(current_index).lidar_target(j), show_lidar_target);
                    showLinedAprilTag(training_img_fig_handles(training_counter), ...
                                      BagData(current_index).camera_target(j), show_camera_target);
                    drawnow
                    X_training_tmp = [X_training_tmp, BagData(current_index).lidar_target(j).scan(:).corners];
                    Y_training_tmp = [Y_training_tmp, repmat(BagData(current_index).camera_target(j).corners, 1, opts.num_lidar_target_pose)];
                    H_LT_tmp = [H_LT_tmp, H_LT];
                    train_num_tag_array = [train_num_tag_array, repmat(BagData(current_index).num_tag, 1, opts.num_lidar_target_pose)];
                    train_tag_size_array = [train_tag_size_array, repmat(BagData(current_index).lidar_target(j).tag_size, 1, opts.num_lidar_target_pose)];
                end

                % 4 x M*i, M is correspondance per scan, i is scan
                X_train = [X_train, X_training_tmp]; 

                % 3 x M*i, M is correspondance per image, i is image
                Y_train = [Y_train, Y_training_tmp]; 
                H_LT_big = [H_LT_big, H_LT_tmp];
                fprintf(" Got training set: %s\n", bag_with_tag_list(current_index))
                training_counter = training_counter + 1;

                % base line
                for i = 1:opts.num_lidar_target_pose
                    pc_iter = opts.num_scan*(i-1) + 1;
                    if base_line_method==1
                        [corners_big, edges] = KaessNewCorners(BagData(current_index).lidar_target(1).tag_size, ...
                                                path.mat_file_path, BagData(current_index).lidar_target(1).pc_file, pc_iter);
                    elseif base_line_method==2
                        [corners_big, edges]= KaessNewConstraintCorners(BagData(current_index).lidar_target(1).tag_size,...
                                                path.mat_file_path, BagData(current_index).lidar_target(1).pc_file, pc_iter);
                    end
                    X_base_line = [X_base_line, corners_big];
                    Y_base_line = [Y_base_line, BagData(current_index).camera_target(1).corners]; %% use big tag
                    X_base_line_edge_points = [X_base_line_edge_points, edges];
                end

            else
                %%% validation set
                X_validation_tmp = [];
                Y_validation_tmp = [];

                for j = 1:BagData(current_index).num_tag
                    [BagData(current_index), ~] = getAll4CornersReturnHLT(j, opt, ...
                                                         path, BagData(current_index), opts);
                    BagData(current_index).camera_target(j).four_corners_line = ...
                                                point2DToLineForDrawing(BagData(current_index).camera_target(j).corners);
                    showAllLinedLiDARTag(validation_pc_fig_handles(validation_counter), ...
                                         BagData(current_index).bagfile, ...
                                         BagData(current_index).lidar_target(j), show_lidar_target);
                    showLinedAprilTag(validation_fig_handles(validation_counter), ...
                                      BagData(current_index).camera_target(j), show_camera_target);
                    drawnow
                    X_validation_tmp = [X_validation_tmp, BagData(current_index).lidar_target(j).scan(:).corners];
                    Y_validation_tmp = [Y_validation_tmp, BagData(current_index).camera_target(j).corners];
                    validation_num_tag_array = [validation_num_tag_array, BagData(current_index).num_tag];
                    validation_tag_size_array = [validation_tag_size_array, BagData(current_index).lidar_target(j).tag_size];
                end

                % 4 x M*i, M is correspondance per scan, i is scan
                X_validation = [X_validation, X_validation_tmp]; 

                % 3 x M*i, M is correspondance per image, i is image
                Y_validation = [Y_validation, Y_validation_tmp]; 
                fprintf(" Got verificatoin set: %s\n", bag_with_tag_list(current_index))
                validation_counter = validation_counter + 1;
            end
        end
        drawnow
    end

    %%
    if ~(skip == 2)
        X_square_no_refinement = X_train;
        X_not_square_refinement = X_base_line;
        disp("********************************************")
        disp(" Calibrating...")
        disp("********************************************")
        switch calibration_method
            case "4 points"
                %%%  one shot calibration (*-NR)
                % square withOUT refinement
                disp('---------------------')
                disp('SNR ...')
                disp('---------------------')
    %             [SNR_H_LC, SNR_P, SNR_opt_total_cost] = optimize4Points(opt.H_LC.rpy_init,...
    %                                                                     X_square_no_refinement, Y_train, ...
    %                                                                     intrinsic_matrix, display);
                [SNR_H_LC, SNR_P, SNR_opt_total_cost, SNR_final, SNR_All] = optimize4Points(opt.H_LC.rpy_init,...
                                                                        X_square_no_refinement, Y_train, ...
                                                                        intrinsic_matrix, show_pnp_numerical_result);                                                    
                calibration(index).H_SNR = SNR_H_LC;
                calibration(index).P_SNR = SNR_P;
                calibration(index).RMSE.SNR = SNR_opt_total_cost;
                calibration(index).All.SNR = SNR_All;


                % NOT square withOUT refinement
                disp('---------------------')
                disp('NSNR ...')
                disp('---------------------')
                [NSNR_H_LC, NSNR_P, NSNR_opt_total_cost, NSNR_final, NSNR_All] = optimize4Points(opt.H_LC.rpy_init, ...
                                                                           X_base_line, Y_base_line, ... 
                                                                           intrinsic_matrix, show_pnp_numerical_result); 
                calibration(index).H_NSNR = NSNR_H_LC;
                calibration(index).P_NSNR = NSNR_P;
                calibration(index).RMSE_NSNR = NSNR_opt_total_cost;
                calibration(index).All.NSNR = NSNR_All;

                for i = 0: opts.num_refinement-1
                    disp('---------------------')
                    disp(' Optimizing H_LC ...')
                    disp('---------------------')

                    % square with refinement
                    [SR_H_LC, SR_P, SR_opt_total_cost, SR_final, SR_All] = optimize4Points(opt.H_LC.rpy_init, ...
                                                                         X_train, Y_train, ... 
                                                                         intrinsic_matrix, show_pnp_numerical_result); 
                    calibration(index).H_SR = SR_H_LC;
                    calibration(index).P_SR = SR_P;
                    calibration(index).RMSE_SR = SR_opt_total_cost;
                    calibration(index).All.SR = SR_All;

                    % NOT square with refinement
                    [NSR_H_LC, NSR_P, NSR_opt_total_cost, NSR_final, NSR_All] = optimize4Points(opt.H_LC.rpy_init, ...
                                                                            X_not_square_refinement, Y_base_line, ...
                                                                            intrinsic_matrix, show_pnp_numerical_result); 
                    calibration(index).H_NSR = NSR_H_LC;
                    calibration(index).P_NSR = NSR_P;
                    calibration(index).RMSE_NSR = NSR_opt_total_cost;
                    calibration(index).All.NSR = NSR_All;

                    if i == opts.num_refinement-1
                        break;
                    else
                        disp('------------------')
                        disp(' Refining H_LT ...')
                        disp('------------------')
                        X_train = regulizedFineTuneLiDARTagPose(train_tag_size_array, ...
                                                                X_train, Y_train, H_LT_big, SR_P, ...
                                                                opts.correspondance_per_pose, show_pnp_numerical_result);
                        X_not_square_refinement = regulizedFineTuneKaessCorners(X_not_square_refinement, Y_base_line,...
                                                                                X_base_line_edge_points, NSR_P, ...
                                                                                opts.correspondance_per_pose, show_pnp_numerical_result);
                    end
                end

            case "IoU"
                % one shot calibration (*-NR)
                [SNR_H_LC, SNR_P, SNR_opt_total_cost, ~, ~] = optimize4Points(opt.H_LC.rpy_init, ...
                                                                        X_square_no_refinement, Y_train, ...
                                                                        intrinsic_matrix, show_pnp_numerical_result); % square withOUT refinement
                [NSNR_H_LC, NSNR_P, NSNR_opt_total_cost, ~, ~] = optimize4Points(opt.H_LC.rpy_init, ...
                                                                           X_base_line, Y_base_line, ...
                                                                           intrinsic_matrix, show_pnp_numerical_result); % NOT square withOUT refinement

                if Alternating
                    for i = 1: opts.num_refinement
                        disp('---------------------')
                        disp(' Optimizing H_LC ...')
                        disp('---------------------')

                        [SR_H_LC, SR_P, SR_opt_total_cost] = optimizeIoU(X_train, Y_train, intrinsic_matrix); % square with refinement
                        [NSR_H_LC, NSR_P, NSR_opt_total_cost] = optimizeIoU(X_not_square_refinement, Y_base_line, intrinsic_matrix); % NOT square with refinement
                        if i == opts.num_refinement
                            break;
                        else
                            disp('------------------')
                            disp(' Refining H_LT ...')
                            disp('------------------')

                            X_train = regulizedFineTuneLiDARTagPose(train_tag_size_array, ...
                                                                    X_train, Y_train, H_LT_big, SR_P, ...
                                                                    opts.correspondance_per_pose, show_pnp_numerical_result);
                            X_not_square_refinement = regulizedFineTuneKaessCorners(X_not_square_refinement, ...
                                                                    Y_base_line, X_base_line_edge_points, NSR_P, ...
                                                                    opts.correspondance_per_pose, show_pnp_numerical_result);
                        end
                    end
                end
        end
    end


    disp("****************** NSNR-training ******************")
    disp('NSNR_H_LC: ')
    disp(' R:')
    disp(NSNR_H_LC(1:3, 1:3))
    disp(' RPY (XYZ):')
    disp(rad2deg(rotm2eul(NSNR_H_LC(1:3, 1:3), "XYZ")))
    disp(' T:')
    disp(-inv(NSNR_H_LC(1:3, 1:3))*NSNR_H_LC(1:3, 4))
    disp("========= Error =========")
    disp(' Training Total Error (pixel)')
    disp(sqrt(NSNR_opt_total_cost))
    disp(' Training Error Per Corner (pixel)')
    disp(sqrt(NSNR_opt_total_cost/size(Y_base_line, 2)))
    calibration(index).error_struc.training_results.id = [bag_training_indices(:)]';
    calibration(index).error_struc.training_results.name = [BagData(bag_training_indices(:)).bagfile];
    calibration(index).error_struc.training_results.NSNR_RMSE = [sqrt(NSNR_opt_total_cost/size(Y_base_line, 2))];

    disp("****************** NSR-training ******************")
    disp('NSR_H_LC: ')
    disp(' R:')
    disp(NSR_H_LC(1:3, 1:3))
    disp(' RPY (XYZ):')
    disp(rad2deg(rotm2eul(NSR_H_LC(1:3, 1:3), "XYZ")))
    disp(' T:')
    disp(-inv(NSR_H_LC(1:3, 1:3))*NSR_H_LC(1:3, 4))
    disp("========= Error =========")
    disp(' Training Total Error (pixel)')
    disp(sqrt(NSR_opt_total_cost))
    disp(' Training Error Per Corner (pixel)')
    disp(sqrt(NSR_opt_total_cost/size(Y_base_line, 2))) 
    calibration(index).error_struc.training_results.NSR_RMSE = [sqrt(NSR_opt_total_cost/size(Y_base_line, 2))];

    disp("****************** SNR-training ******************")
    disp('SNR_H_LC: ')
    disp(' R:')
    disp(SNR_H_LC(1:3, 1:3))
    disp(' RPY (XYZ):')
    disp(rad2deg(rotm2eul(SNR_H_LC(1:3, 1:3), "XYZ")))
    disp(' T:')
    disp(-inv(SNR_H_LC(1:3, 1:3))*SNR_H_LC(1:3, 4))
    disp("========= Error =========")
    disp(' Training Total Error (pixel)')
    disp(sqrt(SNR_opt_total_cost))
    disp(' Training Error Per Corner (pixel)')
    disp(sqrt(SNR_opt_total_cost/size(Y_train, 2)))
    calibration(index).error_struc.training_results.SNR_RMSE = [sqrt(SNR_opt_total_cost/size(Y_train, 2))];

    disp("****************** SR-training ******************")
    disp('H_LC: ')
    disp(' R:')
    disp(SR_H_LC(1:3, 1:3))
    disp(' RPY (XYZ):')
    disp(rad2deg(rotm2eul(SR_H_LC(1:3, 1:3), "XYZ")))
    disp(' T:')
    disp(-inv(SR_H_LC(1:3, 1:3))*SR_H_LC(1:3, 4))
    disp("========= Error =========")
    disp(' Training Total Error (pixel)')
    disp(sqrt(SR_opt_total_cost))
    disp(' Training Error Per Corner (pixel)')
    disp(sqrt(SR_opt_total_cost/size(Y_train, 2)))
    calibration(index).error_struc.training_results.SR_RMSE = [sqrt(SR_opt_total_cost/size(Y_train, 2))];


    [t_SNR_count, t_SR_count]   = inAndOutBeforeAndAfter(bag_training_indices, ...
                                                         opts.num_training, opts, BagData, SNR_P, SR_P);
    [t_NSNR_count, t_NSR_count] = inAndOutBeforeAndAfter(bag_training_indices, ...
                                                         opts.num_training, opts, BagData, NSNR_P, NSR_P);
                                                     
    [SNR_count, SR_count]       = inAndOutBeforeAndAfter(bag_validation_indices, ...
                                                         opts.num_validation, opts, BagData, SNR_P, SR_P);
    [NSNR_count, NSR_count]     = inAndOutBeforeAndAfter(bag_validation_indices, ...
                                                         opts.num_validation, opts, BagData, NSNR_P, NSR_P);
    calibration(index).count.training.SNR = t_SNR_count;
    calibration(index).count.training.SR = t_SR_count;
    calibration(index).count.training.NSR = t_NSR_count;
    calibration(index).count.training.NSNR = t_NSNR_count;

    calibration(index).count.validation.SNR = SNR_count;
    calibration(index).count.validation.SR = SR_count;
    calibration(index).count.validation.NSR = NSR_count;
    calibration(index).count.validation.NSNR = NSNR_count;

    SR_training_cost = verifyCornerAccuracyWRTDataset(bag_training_indices, opts, BagData, SR_P);
    SNR_training_cost = verifyCornerAccuracyWRTDataset(bag_training_indices, opts, BagData, SNR_P);
    NSR_training_cost = verifyCornerAccuracyWRTDataset(bag_training_indices, opts, BagData, NSR_P);
    NSNR_training_cost = verifyCornerAccuracyWRTDataset(bag_training_indices, opts, BagData, NSNR_P);
    for i = 1:opts.num_training
        disp('------')
        current_index = bag_training_indices(i);
        fprintf("---dataset: %s\n", bag_with_tag_list(current_index))
        calibration(index).error_struc.training(i).id = bag_training_indices(i);   
        calibration(index).error_struc.training(i).name = extractBetween(BagData(bag_training_indices(i)).bagfile,"",".bag");
        disp("-- RMS Error Per Corner (pixel)")
        disp(' NSNR training RMS Error Per Corner (pixel)')
        disp(NSNR_training_cost(i).RMSE)
        calibration(index).error_struc.training(i).NSNR_RMSE = [NSNR_training_cost(i).RMSE];
        disp(' NSR training RMS Error Per Corner (pixel)')
        disp(NSR_training_cost(i).RMSE)
        calibration(index).error_struc.training(i).NSR_RMSE = [NSR_training_cost(i).RMSE];
        disp(' SNR training RMS Error Per Corner (pixel)')
        disp(SNR_training_cost(i).RMSE)
        calibration(index).error_struc.training(i).SNR_RMSE = [SNR_training_cost(i).RMSE];
        disp(' SR training RMS Error Per Corner (pixel)')
        disp(SR_training_cost(i).RMSE)
        calibration(index).error_struc.training(i).SR_RMSE = [SR_training_cost(i).RMSE];
        calibration(index).error_struc.training(i).improve_S = ([SNR_training_cost(i).RMSE] - [SR_training_cost(i).RMSE]) ./ [SNR_training_cost(i).RMSE];
        calibration(index).error_struc.training(i).improve_NS = ([NSNR_training_cost(i).RMSE] - [NSR_training_cost(i).RMSE]) ./ [NSNR_training_cost(i).RMSE];
    end
    % 
    % %%% verify corner accuracy
    if validation_flag
        SR_validation_cost = verifyCornerAccuracyWRTDataset(bag_validation_indices, opts, BagData, SR_P);
        SNR_validation_cost = verifyCornerAccuracyWRTDataset(bag_validation_indices, opts, BagData, SNR_P);
        NSR_validation_cost = verifyCornerAccuracyWRTDataset(bag_validation_indices, opts, BagData, NSR_P);
        NSNR_validation_cost = verifyCornerAccuracyWRTDataset(bag_validation_indices, opts, BagData, NSNR_P);

    %     [t_SNR_count, t_SR_count]   = inAndOutBeforeAndAfter(bag_training_indices, ...
    %                                                          opts.num_training, opts, BagData, SNR_P, SR_P);
    %     [t_NSNR_count, t_NSR_count] = inAndOutBeforeAndAfter(bag_training_indices, ...
    %                                                          opts.num_training, opts, BagData, NSNR_P, NSR_P);
    %     [SNR_count, SR_count]       = inAndOutBeforeAndAfter(bag_validation_indices, ...
    %                                                          opts.num_validation, opts, BagData, SNR_P, SR_P);
    %     [NSNR_count, NSR_count]     = inAndOutBeforeAndAfter(bag_validation_indices, ...
    %                                                          opts.num_validation, opts, BagData, NSNR_P, NSR_P);
        for i = 1:opts.num_validation
            current_index = bag_validation_indices(i);
            calibration(index).error_struc.validation(i).id = bag_validation_indices(i);   
            calibration(index).error_struc.validation(i).name = extractBetween(BagData(bag_validation_indices(i)).bagfile,"",".bag");
            calibration(index).error_struc.validation(i).NSNR_RMSE = [NSNR_validation_cost(i).RMSE];
            calibration(index).error_struc.validation(i).NSR_RMSE = [NSR_validation_cost(i).RMSE];
            calibration(index).error_struc.validation(i).SNR_RMSE = [SNR_validation_cost(i).RMSE];
            calibration(index).error_struc.validation(i).SR_RMSE = [SR_validation_cost(i).RMSE];
            calibration(index).error_struc.validation(i).improve_S = ([SNR_validation_cost(i).RMSE] - [SR_validation_cost(i).RMSE])./[SNR_validation_cost(i).RMSE];
            calibration(index).error_struc.validation(i).improve_NS = ([NSNR_validation_cost(i).RMSE] - [NSR_validation_cost(i).RMSE])./[NSNR_validation_cost(i).RMSE];
        end
    end   

        %{
        disp("***************** Training point counting *****************")
        disp("project full pc (SR)")
        disp(struct2table(t_SR_count))
        disp("project full pc (SNR)")
        disp(struct2table(t_SNR_count))
        disp("project full pc (NSR)")
        disp(struct2table(t_NSR_count))
        disp("project full pc (NSNR)")
        disp(struct2table(t_NSNR_count))
        disp("diff")
        disp(([t_NSR_count(:).count] - [t_NSNR_count(:).count])./[t_NSNR_count(:).count])
        disp(([t_SR_count(:).count] - [t_SNR_count(:).count])./[t_SNR_count(:).count])

        disp("***************** validation point counting *****************")
        disp("project full pc (SR)")
        disp(struct2table(SR_count))
        disp("project full pc (SNR)")
        disp(struct2table(SNR_count))
        disp("project full pc (NSR)")
        disp(struct2table(NSR_count))
        disp("project full pc (NSNR)")
        disp(struct2table(NSNR_count))
        disp("diff")
        disp(([NSR_count(:).count] - [NSNR_count(:).count])./[NSNR_count(:).count])
        disp(([SR_count(:).count] - [SNR_count(:).count])./[SNR_count(:).count])
        disp("********************************************")

    end
        %}


    % if ~validation_flag
    %     save(path.save_dir + 'validation_cost' , 'SR_validation_cost', 'SNR_validation_cost', 'NSR_validation_cost', 'NSNR_validation_cost');
    % end

    %%% draw results
    % project training target points 
    for i = 1:opts.num_training % which dataset
        current_index = bag_training_indices(i);

        for j = 1:BagData(current_index).num_tag % which target
            current_corners_SR = [BagData(current_index).lidar_target(j).scan(:).corners];
            current_X_SR = [BagData(current_index).lidar_target(j).scan(:).pc_points];
            projectBackToImage(training_img_fig_handles(i), SR_P, current_corners_SR, 5, 'g*', "training_SR", "not display", "Not-Clean");
            projectBackToImage(training_img_fig_handles(i), SNR_P, current_corners_SR, 5, 'm*', "training_SR", "not display", "Not-Clean");
            projectBackToImage(training_img_fig_handles(i), SR_P, current_X_SR, 3, 'r.', "training_SR", "not display", "Not-Clean");

            showLinedAprilTag(training_img_fig_handles(i), BagData(current_index).camera_target(j), show_training_results);
        end
     end
    drawnow

    % project validation results
    if validation_flag
        for i = 1:opts.num_validation % which dataset
            current_index = bag_validation_indices(i);
            for j = 1:BagData(current_index).num_tag % which target
                current_corners = [BagData(current_index).lidar_target(j).scan(:).corners];
                current_target_pc = [BagData(current_index).lidar_target(j).scan(:).pc_points];
                current_corners = checkHomogeneousCorners(current_corners);
                current_target_pc = checkHomogeneousCorners(current_target_pc);
                projectBackToImage(validation_fig_handles(i), SR_P, current_corners, 5, 'g*', ...
                                  "validation_SR", "not display", "Not-Clean");
                projectBackToImage(validation_fig_handles(i), SNR_P, current_corners, 5, 'm*', ...
                                  "validation_SR", "not display", "Not-Clean");
                projectBackToImage(validation_fig_handles(i), SR_P, current_target_pc, 5, 'r.', ...
                                  "validation_SR", "not display", "Not-Clean");
                showLinedAprilTag(validation_fig_handles(i), BagData(current_index).camera_target(j), show_validation_results);              
            end
        end
    end

    % project testing results
    % load testing images and testing pc mat
    testing_set_pc = loadTestingMatFiles(path.mat_file_path, test_pc_mat_list);
    for i = 1: size(bag_testing_list, 2)
        loadBagImg(testing_fig_handles(i), path.bag_file_path, bag_testing_list(i), "not display", "Not clean"); 
        projectBackToImage(testing_fig_handles(i), SR_P, testing_set_pc(i).mat_pc, 3, 'g.', "testing", show_testing_results, "Not-Clean");
    end
    drawnow
    disp("********************************************") 
    disp("---- Projected using:")
    disp(SR_P)
    disp('--- H_LC: ')
    disp('-- R:')
    disp(SR_H_LC(1:3, 1:3))
    disp('-- RPY (XYZ):')
    disp(rad2deg(rotm2eul(SR_H_LC(1:3, 1:3), "XYZ")))
    disp('-- T:')
    disp(-inv(SR_H_LC(1:3, 1:3))*SR_H_LC(1:3, 4))
    disp("********************************************")
    end
    save(path.save_dir + 'roundRobin.mat', 'calibration', 'validation_flag', 'opt', 'opts');
else
    load(path.load_dir + 'roundRobin.mat', 'calibration', 'validation_flag', 'opt', 'opts');
end


%% Calculate covariance of transformation
for index=1:total_num_dataset
    if  ismember(index, skip_indices)
        continue
    end
    
    % SR
    w = logm(calibration(index).H_SR(1:3,1:3));
    [v] = unskew(w);
    anglular(index).SR = v;
    translation(index).SR = calibration(index).H_SR(1:3,4);
    
    % SNR
    w = logm(calibration(index).H_SNR(1:3,1:3));
    [v] = unskew(w);
    anglular(index).SNR = v;
    translation(index).SNR = calibration(index).H_SNR(1:3,4);
    
     % NSR
    w = logm(calibration(index).H_NSR(1:3,1:3));
    [v] = unskew(w);
    anglular(index).NSR = v;
    translation(index).NSR = calibration(index).H_NSR(1:3,4);
    
     % NSNR
    w = logm(calibration(index).H_NSNR(1:3,1:3));
    [v] = unskew(w);
    anglular(index).NSNR = v;
    translation(index).NSNR = calibration(index).H_NSNR(1:3,4);
end
mean_struc.SR = mean([anglular(:).SR; translation(:).SR]')';
mean_struc.SNR = mean([anglular(:).SNR; translation(:).SNR]')';
mean_struc.NSR = mean([anglular(:).NSR; translation(:).NSR]')';
mean_struc.NSNR = mean([anglular(:).NSNR; translation(:).NSNR]')';

cov_struct.SR = zeros(6,6);
cov_struct.SNR = zeros(6,6);
cov_struct.NSR = zeros(6,6);
cov_struct.NSNR = zeros(6,6);
for index=1:total_num_dataset
    if  ismember(index, skip_indices)
        continue
    end
    
    % SR
    X = [anglular(index).SR; translation(index).SR];
    cov_struct.SR =  cov_struct.SR + (X - mean_struc.SR) * (X - mean_struc.SR)';
    
    % SNR
    X = [anglular(index).SNR; translation(index).SNR];
    cov_struct.SNR =  cov_struct.SNR + (X - mean_struc.SNR) * (X - mean_struc.SNR)';
    
    % NSR
    X = [anglular(index).NSR; translation(index).NSR];
    cov_struct.NSR =  cov_struct.NSR + (X - mean_struc.NSR) * (X - mean_struc.NSR)';
    
    % NSNR
    X = [anglular(index).NSNR; translation(index).NSNR];
    cov_struct.NSNR =  cov_struct.NSNR + (X - mean_struc.NSNR) * (X - mean_struc.NSNR)';
end

cov_struct.SR = cov_struct.SR/(total_num_dataset-length(skip_indices));
cov_struct.SNR = cov_struct.SNR/(total_num_dataset-length(skip_indices));
cov_struct.NSR = cov_struct.NSR/(total_num_dataset-length(skip_indices));
cov_struct.NSNR = cov_struct.NSNR/(total_num_dataset-length(skip_indices));
sqrt(trace(cov_struct.SR))
sqrt(trace(cov_struct.SNR))
sqrt(trace(cov_struct.NSR))
sqrt(trace(cov_struct.NSNR))

%% Calculate in-and-out points 
analysis(total_num_dataset).H_SR(total_num_dataset).diff_H = struct();
for index=1:total_num_dataset
    if  ismember(index, skip_indices)
        analysis(index).validation_L1 = [];
        analysis(index).validation_geometry = [];
        analysis(index).training_L1 = [];
        analysis(index).training_geometry = [];
        continue
    end
    analysis(index).name = calibration(index).count.validation.SR.name;
    analysis(index).validation_L1 = ([calibration(index).count.validation.SR.count] - ...
                                        [calibration(index).count.validation.SNR.count]);
    analysis(index).validation_L1_fraction = [analysis(index).validation_L1] ./ [calibration(index).count.validation.SNR.total_num_point];
    analysis(index).validation_geometry = ([calibration(index).count.validation.NSR.count] - ...
                                              [calibration(index).count.validation.NSNR.count]);
    analysis(index).validation_geometry_fraction = [analysis(index).validation_geometry] ./ [calibration(index).count.validation.SNR.total_num_point];


    analysis(index).training_L1 = ([calibration(index).count.training.SR.count] - [calibration(index).count.training.SNR.count]);
    analysis(index).training_L1_fraction = [analysis(index).training_L1] ./ [calibration(index).count.training.SNR.total_num_point];
    analysis(index).training_geometry = ([calibration(index).count.training.NSR.count] - [calibration(index).count.training.NSNR.count]);
    analysis(index).training_geometry_fraction = [analysis(index).training_geometry] ./ [calibration(index).count.training.SNR.total_num_point];

    analysis(index).validation_L1_sum = sum([calibration(index).count.validation.SR.count] - ...
                                        [calibration(index).count.validation.SNR.count]);
    analysis(index).validation_geometry_sum = sum([calibration(index).count.validation.NSR.count] - ...
                                              [calibration(index).count.validation.NSNR.count]);

    analysis(index).training_L1_sum = sum([calibration(index).count.training.SR.count] - [calibration(index).count.training.SNR.count]);
    analysis(index).training_geometry_sum = sum([calibration(index).count.training.NSR.count] - [calibration(index).count.training.NSNR.count]);


    disp("***************************************************************************************")
    disp("***************************************************************************************")
    disp("==================")
    disp(" training results")
    disp("==================")
    disp(struct2table(calibration(index).error_struc.training_results))
    disp("==================")
    disp(" training error")
    disp("==================")
    disp(struct2table(calibration(index).error_struc.training))

    if validation_flag
        disp("==================")
        disp(" validation error")
        disp("==================")
        disp(struct2table(calibration(index).error_struc.validation))
    end

    % H_SR
    H_current = calibration(index).H_SR;
    for j=1:total_num_dataset
        if  ismember(j, skip_indices) || index==j
            continue
        end
        difference = H_diff(H_current, calibration(j).H_SR);
        analysis(index).H_SR(j).diff_H = difference;
    end
    analysis(index).H_SR_std = std([analysis(index).H_SR(:).diff_H]);
    analysis(index).H_SR_sum = sum([analysis(index).H_SR(:).diff_H]);

    % H_SNR
    H_current = calibration(index).H_SNR;
    for j=1:total_num_dataset
        if  ismember(j, skip_indices)
            continue
        end
        difference = H_diff(H_current, calibration(j).H_SNR);
        analysis(index).H_SNR(j).diff_H = difference;
    end
    analysis(index).H_SNR_std = std([analysis(index).H_SNR(:).diff_H]);
    analysis(index).H_SNR_sum = sum([analysis(index).H_SNR(:).diff_H]);

    % H_NSR
    H_current = calibration(index).H_NSR;
    for j=1:total_num_dataset
        if  ismember(j, skip_indices)
            continue
        end
        difference = H_diff(H_current, calibration(j).H_NSR);
        analysis(index).H_NSR(j).diff_H = difference;
    end
    analysis(index).H_NSR_std = std([analysis(index).H_NSR(:).diff_H]);
    analysis(index).H_NSR_sum = sum([analysis(index).H_NSR(:).diff_H]);

    % H_NSNR
    H_current = calibration(index).H_NSNR;
    for j=1:total_num_dataset
        if  ismember(j, skip_indices)
            continue
        end
        difference = H_diff(H_current, calibration(j).H_NSNR);
        analysis(index).H_NSNR(j).diff_H = difference;
    end
    analysis(index).H_NSNR_std = std([analysis(index).H_NSNR(:).diff_H]);
    analysis(index).H_NSNR_sum = sum([analysis(index).H_NSNR(:).diff_H]);
end
    
if more_analysis
    disp("L1 validation")
    disp((reshape([analysis(:).validation_L1],[total_num_dataset-length(skip_indices)-1,total_num_dataset-length(skip_indices)])'))
    disp("L1 validation fraction")
    disp((100*reshape([analysis(:).validation_L1_fraction],[total_num_dataset-length(skip_indices)-1,total_num_dataset-length(skip_indices)])'))

    disp("Geometry validation")
    disp((reshape([analysis(:).validation_geometry],[total_num_dataset-length(skip_indices)-1,total_num_dataset-length(skip_indices)])'))
    disp("Geometry validation fraction")
    disp((100*reshape([analysis(:).validation_geometry_fraction],[total_num_dataset-length(skip_indices)-1,total_num_dataset-length(skip_indices)])'))

    disp("L1 training")
    disp(([analysis(:).training_L1]))
    disp("L1 training fraction")
    disp((100*[analysis(:).training_L1_fraction]))
    disp("Geometry training")
    disp(([analysis(:).training_geometry]))
    disp("Geometry training fraction")
    disp((100*[analysis(:).training_geometry_fraction]))    

    % plot
    figure()
    hold on
    x = linspace(1,length([analysis(:).H_SR_std]), length([analysis(:).H_SR_std]));
    plot(x, [analysis(:).H_SR_std])
    plot(x, [analysis(:).H_SNR_std])
    plot(x, [analysis(:).H_NSR_std])
    plot(x, [analysis(:).H_NSNR_std])
    legend("H_{SR}-std","H_{SNR}-std","H_{NSR}-std","H_{NSNR}-std")

    figure()
    hold on
    plot(x, 100*[[analysis(:).H_SNR_std] - [analysis(:).H_SR_std]] ./ [analysis(:).H_SNR_std])
    plot(x, 100*[[analysis(:).H_NSNR_std] - [analysis(:).H_NSR_std]] ./ [analysis(:).H_NSNR_std])
    legend("H_{SR}-std - H_{SNR}-std","H_{NSR}-std - H_{NSNR}-std")


    analysis_summary.sum.H_SR_std = sum([analysis(:).H_SR_std]);
    analysis_summary.sum.H_SNR_std = sum([analysis(:).H_SNR_std]);
    analysis_summary.sum.H_NSR_std = sum([analysis(:).H_NSR_std]);
    analysis_summary.sum.H_NSNR_std = sum([analysis(:).H_NSNR_std]); 
    disp("==================")
    disp("sum of std")
    disp("==================")
    disp(struct2table(analysis_summary.sum))
    save(path.save_dir + 'analysis.mat', 'analysis_summary', 'analysis');

    % print out means and normal vectors
    for i = 1 : length(bag_chosen_indices)
        current_index = bag_chosen_indices(i);
    %     disp('=====================================')
    %     fprintf("-dataset: %s\n", bag_with_tag_list(current_index))
        centroid_vec = [];
        nv_vec = [];

        for j = 1:BagData(current_index).num_tag % which target
    %         fprintf("---------------tag: %i\n", j)
            lidar_target(j).results(i).name = BagData(current_index).bagfile;
            lidar_target(j).results(i).mean_centriod = mean([BagData(current_index).lidar_target(j).scan(:).centroid], 2)';
            lidar_target(j).results(i).mean_NV = mean([BagData(current_index).lidar_target(j).scan(:).normal_vector], 2)';
            lidar_target(j).results(i).std_mean = std([BagData(current_index).lidar_target(j).scan(:).centroid]');
            lidar_target(j).results(i).std_NV = std([BagData(current_index).lidar_target(j).scan(:).normal_vector]');
            lidar_target(j).results(i).std_NV_norm_100 = 100*norm(lidar_target(j).results(i).std_NV);
            centroid_vec = [centroid_vec; lidar_target(j).results(i).mean_centriod];
            nv_vec = [nv_vec; lidar_target(j).results(i).mean_NV];
    %         disp("----------pose:")
    %         disp("--centroid:")
    %         disp(lidar_target(j).results(i).mean_centriod)
    %         disp("--normal vector:")
    %         disp(lidar_target(j).results(i).mean_NV)

    %         disp("----------std:")
    %         disp("--centroid:")
    %         disp(lidar_target(j).results(i).std_mean)        
    %         disp("--normal vector:")
    %         disp(lidar_target(j).results(i).std_NV)
        end
    %     disp("-----diff between tags:")
    %     disp("--centroid:")
        results_diff(i).name = BagData(current_index).bagfile;
        results_diff(i).diff_centroid = diff(centroid_vec, 1, 1);
        results_diff(i).distance = sqrt(sum((centroid_vec(:,1)- centroid_vec(:,2)).^2, 1));
    %     disp(results_diff(i).diff_centroid)
    %     disp("--normal vector:")
        results_diff(i).diff_NV = diff(nv_vec, 1, 1);
        results_diff(i).diff_NV_deg = rad2deg(acos(nv_vec(1)'*nv_vec(2)));
    %     disp(results_diff(i).diff_NV)
    end

    disp('======================================================================')
    disp("big target")
    disp(struct2table(lidar_target(1).results(:)))
    disp('======================================================================')
    disp("small target")
    disp(struct2table(lidar_target(2).results(:)))
    disp('======================================================================')
    disp(struct2table(results_diff))
end
% analysis_summary.std.H_SR_std = std([analysis(:).H_SR_std]);
% analysis_summary.std.H_SNR_std = std([analysis(:).H_SNR_std]);
% analysis_summary.std.H_NSR_std = std([analysis(:).H_NSR_std]);
% analysis_summary.std.H_NSNR_std = std([analysis(:).H_NSNR_std]);
% disp("std of std")
% disp(struct2table(analysis_summary.std))

% disp()
% 
% disp("sum of H_SNR std ")
% disp(sum([analysis(:).H_SNR_std]))
% 
% disp("sum of H_NSR std ")
% disp(sum([analysis(:).H_NSR_std]))
% 
% disp("sum of H_NSNR std ")
% disp(sum([analysis(:).H_NSNR_std]))
