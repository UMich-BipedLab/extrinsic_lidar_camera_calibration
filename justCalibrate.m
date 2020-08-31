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

clc, clear


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Camera Parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
intrinsic_matrix = [351.3009289812843, 0.0,            580.5;  % cam0
                    0.0,            351.3009289812843, 335.0;  % cam0
                    0.0, 0.0, 1.0];
opt.intrinsic_matrix = intrinsic_matrix;            

% Initial guess Lidar to Camera
global_H_LC.rpy_init = [90.0 0.0 -180.0];
global_H_LC.T_init = [0.0314, -0.1, 0.0838];

% Initial guess Tag to Lidar
global_H_TL.rpy_init = [2 -48 3];
global_H_TL.T_init = [0, -3, 0];

% Camera topic name (rectified images)
camera_topic_name = "/output/image";
image_is_color = 0; % 1: undistorted color image input, 0: undistorted_grayscale_image_input

% train data id from getBagData.m
trained_ids = [2, 3, 5]; % 
skip_indices = [1]; %% skip non-standard

% validate the calibration result if one has validation dataset(s)
% (Yes:1; No: 0)
% Note: A validation dataset is the same as training set, i.e. it has to
% have calibration targets in the scene; However, a testing set does not
% need targets in the scene. 
validation_flag = 1; 


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Data path Parameters

%%% path.load_dir: directory of saved files
%%% load_all_vertices: pre-calculated vertices from prior results with good vertice extraction (only used if opts.optimizeAllCorners := 0; and opts.refineAllCorners := 0; -> just tuning optimization and relying on saved vertices)
%%% bag_file_path: bag files of images (Needs to contain at least the undistorted images, bagile names as specified in getBagData.m)
%%% mat_file_path: mat files of extracted lidar target's point clouds (point_clouds of each target as well as whole point_cloud, as specified in getBagData.m)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
path.load_dir = "/";
path.load_all_vertices = "preextracted_tag_vertices/cam0/";
path.bag_file_path = "bagfiles/cam0_undistorted/";
path.mat_file_path = "matfiles/";
path.event_name = '';


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% parameters of user setting
%%% optimizeAllCorners (0/1): <default: 1>
%                             optimize all lidar targets vertices for
%                             different datasets
%                   [NOTE]: this usually only needs to be done ONCE.
%
%%% refineAllCorners (0/1): <default: 0>
%                           reifne all lidar targets vertices for
%                           different datasets
%                   [NOTE]: this usually only needs to be done ONCE.
%
%%% use_top_consistent_vertices (0/1): <default: 0>
%                                     find the top-5 consistent scans and
%                                     use the vertices for calibration
%                   
%
%%% randperm_to_fine_vertices (0/1): <default: 0>
%                                   reifne all lidar targets vertices for
%                                   different datasets
%                   [NOTE]: this usually only needs to be done ONCE.
% 
%%% skip (0/1/2): <default: 1>
%        0: optimize lidar target's corners
%           and then calibrate 
%        1: skip optimize lidar target's corners (if you have done so)
%        2: just shown calibration results
%
%%% debug (0/1):  <default: 0>
%               print more stuff at the end to help debugging
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
opts.optimizeAllCorners = 0;
opts.refineAllCorners = 0;
opts.use_top_consistent_vertices = 0;
opts.randperm_to_fine_vertices = 0;
skip = 0; 
debug = 0;


%=========================================================================%
%============== You usually do not need change setting below =============%
%=========================================================================%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Baseline %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% optimized_method (1/2): <default: 1>
%                   1: ransac edges seperately and the intersect edges to
%                      estimate corners 
%                   2: apply geometry contrain to estimate the corners 
%                      <currently not supported>
%
%%% edge_method (1/2/3): <default: 3>
%                   1: JWG's method
%                   2: Manual pick edge points 
%                      -- top-left, bottom-left, top-right, bottom-left
%                   3: L1-cost to assign edge points 
%
%%% more_tags (0/1): <default: 1> 
%                    if use more tags in a scene for the baseline 
%
%%% show_results (0/1): <default: 0>
%                       show baseline results
%
%%% L1_cleanup (0/1) : <default: 0>
%                       Cleanup the data using L1-inspired cost
%
%%% num_scan (int) : <default: 5>
%                    how many scans accumulated to optimize one LiDARTag pose
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
base_line.optimized_method = 1;
base_line.edge_method = 3;  % Currently only method 3 is in working condition
base_line.more_tags = 1;
base_line.show_results = 0;
base_line.L1_cleanup = 0;
base_line.num_scan = 5;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% calibration_method:  <default: "4 points">
%                     "4 points"
%                     "IoU"
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
opts.calibration_method = "4 points";
% opts.calibration_method = "IoU";

% save into results into folder         
path.save_name = "results";
diary Debug % save terminal outputs


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% show figures
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
show_image_refinement = 0;
show_pnp_numerical_result = 0;
show_lidar_target = 0;
show.lidar_target_optimization = 0;
show_camera_target = 0;
show_training_results = 0;
show_validation_results = 1; 
show_testing_results = 1;
show_baseline_results = 0;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% parameters for optimization of lidar targets
% num_refinement: how many rounds of refinement
% num_lidar_target_pose: how many lidar target poses to optimize H_LC 
% num_scan: accumulate how many scans to optimize a lidar target's corners
% correspondance_per_pose (int): how many correspondance on a target
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
opts.num_refinement = 5 ; % 4 rounds of refinement
opts.num_lidar_target_pose = 1; % (5) how many LiDARTag poses to optimize H_LC (5) (2)
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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
opt.H_TL.rpy_init = global_H_TL.rpy_init;
opt.H_TL.T_init = global_H_TL.T_init;
opt.H_TL.H_init = eye(4);
opt.H_TL.method = "GICP-SE3"; 
opt.H_TL.UseCentroid = 0;

% Set H_LC initial guess
opt.H_LC.rpy_init = global_H_LC.rpy_init;
opt.H_LC.T_init = global_H_LC.T_init;


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
[BagData, TestData] = getBagData();
bag_with_tag_list  = [BagData(:).bagfile];
bag_testing_list = [TestData(:).bagfile];
test_pc_mat_list = [TestData(:).pc_file];
opts.num_training = length(trained_ids); 
opts.num_validation = length(bag_with_tag_list) - length(skip_indices) - opts.num_training;    


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp("Refining corners of camera targets ...")
BagData = refineImageCorners(path.bag_file_path, BagData, skip_indices, show_image_refinement, camera_topic_name, image_is_color);

% create figure handles
training_img_fig_handles = createFigHandle(opts.num_training, "training_img");
training_pc_fig_handles = createFigHandle(opts.num_training, "training_pc");
validation_fig_handles = createFigHandle(opts.num_validation, "validation_img");
validation_pc_fig_handles = createFigHandle(opts.num_validation, "validation_pc");
testing_fig_handles = createFigHandle(size(bag_testing_list, 2), "testing");
base_line.img_hangles = createFigHandle(6, "base_line_vis"); %% don't change

if random_select
    % get training indices
    bag_training_indices = randi([1, length(bag_with_tag_list)], 1, opts.num_training);

    % make sure they are not the same and not consists of undesire index
    while length(unique(bag_training_indices)) ~=  length(bag_training_indices) || ...
            any(ismember(bag_training_indices, skip_indices)) 
        bag_training_indices = randi([1, length(bag_with_tag_list)], 1, opts.num_training);
    end
    
    % get validation indices
    bag_validation_indices = randi(length(bag_with_tag_list), 1, opts.num_validation);

    % make sure they are not the same and not consists of undesire index
    while length(unique(bag_validation_indices)) ~=  length(bag_validation_indices) || ...`
          any(ismember(bag_validation_indices, skip_indices)) || ...
          any(ismember(bag_validation_indices, bag_training_indices)) 
       bag_validation_indices = randi(length(bag_with_tag_list), 1, opts.num_validation);
    end
else
    % overwrite
    bag_training_indices = trained_ids;
    bag_validation_indices = linspace(1, length(bag_with_tag_list), length(bag_with_tag_list));
    bag_validation_indices([trained_ids skip_indices]) = [];
end
bag_chosen_indices = [bag_training_indices, bag_validation_indices];

ans_error_big_matrix = [];
ans_counting_big_matrix = [];

if skip
    load(path.load_dir + 'saved_chosen_indices.mat');
    load(path.load_dir + 'saved_parameters.mat');
end

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
c = datestr(datetime); 
path.save_dir = path.save_name + "/" + c + "/";


if ~skip
    mkdir(path.save_dir);
    save(path.save_dir + 'saved_parameters.mat', 'opts', 'validation_flag');
    save(path.save_dir + 'saved_chosen_indices.mat', 'skip_indices', 'bag_training_indices', 'bag_validation_indices', 'bag_chosen_indices');
else
    load(path.load_dir + "X_base_line.mat");
    load(path.load_dir + "X_train.mat");
    load(path.load_dir + "Y.mat")
    load(path.load_dir + "save_validation.mat")
    load(path.load_dir + "array.mat")
    load(path.load_dir + "BagData.mat")
end

% loading training image
for k = 1:opts.num_training
    current_index = bag_training_indices(k);
    loadBagImg(training_img_fig_handles(k), path.bag_file_path, bag_with_tag_list(current_index), "not display", "not clean", camera_topic_name);
    
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
        loadBagImg(validation_fig_handles(k), path.bag_file_path, bag_with_tag_list(current_index), "not display", "not clean", camera_topic_name);

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
                if base_line.optimized_method == 1
                    [X_corners_big, edges_big] = KaessNewCorners_v02(base_line, BagData(current_index), ...
                                                                     path.mat_file_path, i, 1, pc_iter, opt.H_TL.rpy_init, opt.H_TL.T_init);
                     Y_corner_big = [BagData(current_index).camera_target(1).corners];
                    if BagData(current_index).num_tag > 1
                        if base_line.more_tags == 1
                            for j = 2:BagData(current_index).num_tag
                                [corners_tmp, edges_tmp] = KaessNewCorners_v02(base_line, BagData(current_index), ...
                                                                               path.mat_file_path, i, j, pc_iter, opt.H_TL.rpy_init, opt.H_TL.T_init);
                                X_corners_big = [X_corners_big, corners_tmp];
                                edges_big = [edges_big, edges_tmp];
                                Y_corner_big = [Y_corner_big, BagData(current_index).camera_target(j).corners];
                            end
                        end
                    end
                elseif base_line.optimized_method == 2
                    error("Currently not supported this baseline method: %i",  base_line.optimized_method)
                end
                X_base_line = [X_base_line, X_corners_big];
                Y_base_line = [Y_base_line, Y_corner_big]; 
                X_base_line_edge_points = [X_base_line_edge_points, edges_big];
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
            
            % baseline corners of validation datasets
            for i = 1:opts.num_lidar_target_pose
                pc_iter = opts.num_scan*(i-1) + 1;
                BagData(current_index).baseline = [];
                if base_line.optimized_method == 1
                    [~, ~, BagData(current_index)] = KaessNewCorners_v02(base_line, BagData(current_index), ...
                                                                         path.mat_file_path, i, 1, pc_iter, opt.H_TL.rpy_init, opt.H_TL.T_init);
                    if BagData(current_index).num_tag > 1
                        if base_line.more_tags == 1
                            for j = 2:BagData(current_index).num_tag
                                [~, ~, BagData(current_index)] = KaessNewCorners_v02(base_line, BagData(current_index), ...
                                                                                     path.mat_file_path, i, j, pc_iter, opt.H_TL.rpy_init, opt.H_TL.T_init);
                            end
                        end
                    end
                elseif base_line.optimized_method == 2
                    error("Currently not supported this baseline method: %i",  base_line.optimized_method)
                end
            end
            
            fprintf(" Got verificatoin set: %s\n", bag_with_tag_list(current_index))
            validation_counter = validation_counter + 1;
        end
    end
    drawnow
    save(path.save_dir + 'X_base_line.mat', 'X_base_line');
    save(path.save_dir + 'X_train.mat', 'X_train', 'H_LT_big', 'X_base_line_edge_points');
    save(path.save_dir + 'array.mat', 'train_num_tag_array', 'train_tag_size_array', 'validation_num_tag_array', 'validation_tag_size_array');
    save(path.save_dir + 'Y.mat', 'Y_train', 'Y_base_line');
    save(path.save_dir + 'BagData.mat', 'BagData');
    save(path.save_dir + 'save_validation.mat', 'X_validation', 'Y_validation');
end

disp("Vertices and corners obtained!")
%% Use vertices and corners to calibrate 
if ~(skip == 2)
    X_square_no_refinement = X_train;
    X_not_square_refinement = X_base_line;
    disp("********************************************")
    disp(" Calibrating...")
    disp("********************************************")
    switch opts.calibration_method
        case "4 points"
            %%%  one shot calibration (*-NR)
            % square withOUT refinement
            disp('---------------------')
            disp('SNR ...')
            disp('---------------------')
%             [SNR_H_LC, SNR_P, SNR_opt_total_cost] = optimize4Points(opt.H_LC.rpy_init, opt.H_LC.T_init, ...
%                                                                     X_square_no_refinement, Y_train, ...
%                                                                     intrinsic_matrix, display);
            [SNR_H_LC, SNR_P, SNR_opt_total_cost, SNR_final, SNR_All] = optimize4Points(opt.H_LC.rpy_init, opt.H_LC.T_init, ...
                                                                    X_square_no_refinement, Y_train, ...
                                                                   intrinsic_matrix, show_pnp_numerical_result);                                                    
            calibration(1).H_SNR = SNR_H_LC;
            calibration(1).P_SNR = SNR_P;
            calibration(1).RMSE.SNR = SNR_opt_total_cost;
            calibration(1).All.SNR = SNR_All; 
            
            % NOT square withOUT refinement
            disp('---------------------')
            disp('NSNR ...')
            disp('---------------------')
            [NSNR_H_LC, NSNR_P, NSNR_opt_total_cost, NSNR_final, NSNR_All] = optimize4Points(opt.H_LC.rpy_init, opt.H_LC.T_init, ...
                                                                       X_base_line, Y_base_line, ... 
                                                                       intrinsic_matrix, show_pnp_numerical_result); 
            calibration(1).H_NSNR = NSNR_H_LC;
            calibration(1).P_NSNR = NSNR_P;
            calibration(1).RMSE_NSNR = NSNR_opt_total_cost;
            calibration(1).All.NSNR = NSNR_All;
            

        case "IoU"
            % one shot calibration (*-NR)
            [SNR_H_LC, SNR_P, SNR_opt_total_cost, ~, SNR_All] = optimizeIoU(opt.H_LC.rpy_init, ...
                                                                            X_square_no_refinement, Y_train, ...
                                                                            intrinsic_matrix, show_pnp_numerical_result); % square withOUT refinement
            calibration(1).H_SNR = SNR_H_LC;
            calibration(1).P_SNR = SNR_P;
            calibration(1).RMSE.SNR = SNR_opt_total_cost;
            calibration(1).All.SNR = SNR_All;
            
            [NSNR_H_LC, NSNR_P, NSNR_opt_total_cost, ~, NSNR_All] = optimizeIoU(opt.H_LC.rpy_init, ...
                                                                                X_base_line, Y_base_line, ...
                                                                                intrinsic_matrix, show_pnp_numerical_result); % NOT square withOUT refinement
            calibration(1).H_NSNR = NSNR_H_LC;
            calibration(1).P_NSNR = NSNR_P;
            calibration(1).RMSE_NSNR = NSNR_opt_total_cost;
            calibration(1).All.NSNR = NSNR_All;
            
            
    end
    if skip == 0
        save(path.save_dir + 'calibration.mat', 'calibration');
        save(path.save_dir + 'save_validation.mat', 'X_validation', 'Y_validation');
        save(path.save_dir + 'NSNR.mat', 'NSNR_H_LC', 'NSNR_P', 'NSNR_opt_total_cost');
        save(path.save_dir + 'SNR.mat', 'SNR_H_LC', 'SNR_P', 'SNR_opt_total_cost');
    elseif skip == 1
        save(path.load_dir + 'calibration.mat', 'calibration');
        save(path.load_dir + 'save_validation.mat', 'X_validation', 'Y_validation');
        save(path.load_dir + 'NSNR.mat', 'NSNR_H_LC', 'NSNR_P', 'NSNR_opt_total_cost');
        save(path.load_dir + 'SNR.mat', 'SNR_H_LC', 'SNR_P', 'SNR_opt_total_cost');
    end
else
    % load saved data
    load(path.load_dir + 'calibration.mat');
    load(path.load_dir + "NSNR.mat");
    load(path.load_dir + "SNR.mat");
    load(path.load_dir + "save_validation.mat")
end
calibration(1).error_struc.training_results.id = [bag_training_indices(:)]';
calibration(1).error_struc.training_results.name = [BagData(bag_training_indices(:)).bagfile];
calibration(1).error_struc.training_results.NSNR_RMSE = [sqrt(NSNR_opt_total_cost/size(Y_base_line, 2))];
calibration(1).error_struc.training_results.SNR_RMSE = [sqrt(SNR_opt_total_cost/size(Y_train, 2))];


SNR_training_cost = verifyCornerAccuracyWRTDataset(bag_training_indices, opts, BagData, SNR_P);
NSNR_training_cost = verifyCornerAccuracyWRTDataset(bag_training_indices, opts, BagData, NSNR_P);

for i = 1:opts.num_training
        current_index = bag_training_indices(i);
        calibration(1).error_struc.training(i).id = bag_training_indices(i);   
        calibration(1).error_struc.training(i).name = extractBetween(BagData(bag_training_indices(i)).bagfile,"",".bag");
        calibration(1).error_struc.training(i).NSNR_RMSE = [NSNR_training_cost(i).RMSE];
        calibration(1).error_struc.training(i).SNR_RMSE = [SNR_training_cost(i).RMSE];
end

%%% verify corner accuracy
if validation_flag
    SNR_validation_cost = verifyCornerAccuracyWRTDataset(bag_validation_indices, opts, BagData, SNR_P);
    NSNR_validation_cost_on_its_own = verifyCornerAccuracyWRTDatasetOnItsOwnMethod(base_line, ...
                                                                                   bag_validation_indices, opts, BagData, NSNR_P);
    for i = 1:opts.num_validation
        calibration(1).error_struc.validation(i).id = bag_validation_indices(i);   
        calibration(1).error_struc.validation(i).name = extractBetween(BagData(bag_validation_indices(i)).bagfile,"",".bag");
        calibration(1).error_struc.validation(i).NSNR_RMSE_validate_its_own = [NSNR_validation_cost_on_its_own(i).RMSE];
        calibration(1).error_struc.validation(i).SNR_RMSE = [SNR_validation_cost(i).RMSE];
    end
end   

%%% draw results
% project training target points 
for i = 1:opts.num_training % which dataset
    current_index = bag_training_indices(i);

    for j = 1:BagData(current_index).num_tag % which target
        current_corners_SR = [BagData(current_index).lidar_target(j).scan(:).corners];
        current_X_SR = [BagData(current_index).lidar_target(j).scan(:).pc_points];
        if show_baseline_results
            projectBackToImage(training_img_fig_handles(i), NSNR_P, current_corners_SR, 5, 'kd', "training_SR", "not display", "Not-Clean");
        end
        projectBackToImage(training_img_fig_handles(i), SNR_P, current_corners_SR, 5, 'm*', "training_SR", "not display", "Not-Clean");
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
            if show_baseline_results
                projectBackToImage(validation_fig_handles(i), NSNR_P, current_corners, 5, 'cd', ...
                                  "validation_SR", "not display", "Not-Clean");
            end
            projectBackToImage(validation_fig_handles(i), SNR_P, current_corners, 5, 'm*', ...
                              "validation_SR", "not display", "Not-Clean");
            projectBackToImage(validation_fig_handles(i), SNR_P, current_target_pc, 5, 'r.', ...
                              "validation_SR", "not display", "Not-Clean");
            showLinedAprilTag(validation_fig_handles(i), BagData(current_index).camera_target(j), show_validation_results);              
        end
    end
end

% project testing results
% load testing images and testing pc mat
testing_set_pc = loadTestingMatFiles(path.mat_file_path, test_pc_mat_list);
for i = 1: size(bag_testing_list, 2)
    loadBagImg(testing_fig_handles(i), path.bag_file_path, bag_testing_list(i), "not display", "Not clean", camera_topic_name); 
    projectBackToImage(testing_fig_handles(i), SNR_P, testing_set_pc(i).mat_pc, 3, 'g.', "testing", show_testing_results, "Not-Clean");
end
drawnow
disp("********************************************") 
disp("---- Projected using:")
disp(SNR_P)
disp('--- H_LC: ')
disp('-- R:')
disp(SNR_H_LC(1:3, 1:3))
disp('-- RPY (XYZ):')
disp(rad2deg(rotm2eul(SNR_H_LC(1:3, 1:3), "XYZ")))
disp('-- T:')
disp(-inv(SNR_H_LC(1:3, 1:3))*SNR_H_LC(1:3, 4))
disp("********************************************")

if skip == 0
    save(path.save_dir + 'calibration.mat', 'calibration');
elseif skip == 1
    save(path.load_dir + 'calibration.mat', 'calibration');
end
disp("***************************************************************************************")
disp("***************************************************************************************")
% disp("------------------")
disp(" training results")
% disp("------------------")
disp(struct2table(calibration(1).error_struc.training_results))

%%
if validation_flag
%     disp("------------------")
    disp(" validation error")
%     disp("------------------")
    disp(struct2table(calibration(1).error_struc.validation))
%     disp("NSNR_RMSE--validata on its own method")
%     [calibration(1).error_struc.validation.baseline.NSNR_RMSE]
    disp("Baseline_RMSE")
    [calibration(1).error_struc.validation.NSNR_RMSE_validate_its_own]
    disp("L1-inspired_RMSE")
    [calibration(1).error_struc.validation.SNR_RMSE]

    disp("------paper info-------")
    disp("-- training RMSE (BL ; L1)")
    training_RMSE = [calibration(1).error_struc.training_results.NSNR_RMSE;
                    calibration(1).error_struc.training_results.SNR_RMSE]

    disp("-- validating RMSE (BL ; L1)")
    validating_RMSE = [calibration(1).error_struc.validation.NSNR_RMSE_validate_its_own;
                      calibration(1).error_struc.validation.SNR_RMSE]
    disp('summary mean RMSE (BL ; L1)')
    validating_mean = mean(validating_RMSE')'
    validating_std = std(validating_RMSE')'
end
