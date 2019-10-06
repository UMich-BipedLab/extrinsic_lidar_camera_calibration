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
%%% display (0/1): To show numerical results
%%% validation_flag (0/1): validate the calibration result
%%% base_line_method (1/2): 
%                   1: ransac edges seperately and the intersect edges to
%                      estimate corners (will add back in a bit)
%                   2: apply geometry contrain to estimate the corners
%%% correspondance_per_pose (int): how many correspondance on a target
%%% calibration_method: 
%                     "4 points"
%                     "IoU"
%%% load_dir: directory of saved files
%%% bag_file_path: bag files of images 
%%% mat_file_path: mat files of extracted lidar target's point clouds
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
skip = 0; 
display = 1; % show numerical results
validation_flag = 1; % validate results
base_line_method = 2;
correspondance_per_pose = 4; % 4 correspondance on a target
calibration_method = "4 points";
load_dir = "Paper-C71/02-Oct-2019 13:41:26/";
bag_file_path = "bagfiles/";
mat_file_path = "LiDARTag_data/";

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% parameters for optimization of lidar targets
%%% We have tried several methods to recover the uobserable lidar target's 
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
% num_refinement: how many rounds of refinement
% num_lidar_target_pose: how many lidar target poses to optimize H_LC 
% num_scan: accumulate how many scans to optimize a lidar target's corners
% num_training: how many training sets to optimize H_LC
% num_validation: how many validation set to verify the optimized H_LC
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
opt.num_refinement = 4; % 4 rounds of refinement
opt.num_lidar_target_pose = 5; % how many LiDARTag poses to optimize H_LC (5) (2)
opt.num_scan = 3; % how many scans accumulated to optimize one LiDARTag pose (3)
opt.num_training = 1; %%% how many training set to use (2)
opt.num_validation = 7; % use how many different datasets to verify the calibration result


opt_H_TL.rpy_init = [45 2 3];
opt_H_TL.T_init = [2, 0, 0];
opt_H_TL.H_init = eye(4);
opt_H_TL.method = "Constraint Customize"; 
opt_H_TL.UseCentroid = 1;
opt_H_TC.rpy_init = [90 0 90];


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
trained_ids = [11];
skip_indices = [1, 2, 3, 12]; %% skip non-standard 
[BagData, TestData] = getBagData();  %% get 
bag_with_tag_list  = [BagData(:).bagfile];
bag_testing_list = [TestData(:).bagfile];
test_pc_mat_list = [TestData(:).pc_file];
opt.num_validation = min(size(bag_with_tag_list, 2) - ...
                         length(skip_indices) - opt.num_training, ...
                         opt.num_validation);
opt.num_training = min(length(trained_ids), opt.num_training);                    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% create figure handles
training_img_fig_handles = createFigHandle(opt.num_training, "training_img");
training_pc_fig_handles = createFigHandle(opt.num_training, "training_pc");
validation_fig_handles = createFigHandle(opt.num_validation, "validation_img");
validation_pc_fig_handles = createFigHandle(opt.num_validation, "validation_pc");
testing_fig_handles = createFigHandle(size(bag_testing_list, 2), "testing");

          

if random_select
    % get training indices
    bag_training_indices = randi([1, length(bag_with_tag_list)], 1, opt.num_training);

    % make sure they are not the same and not consists of undesire index
    while length(unique(bag_training_indices)) ~=  length(bag_training_indices) || ...
            any(ismember(bag_training_indices, skip_indices)) 
        bag_training_indices = randi([1, length(bag_with_tag_list)], 1, opt.num_training);
    end
    
    % get validation indices
    bag_validation_indices = randi(length(bag_with_tag_list), 1, opt.num_validation);

    % make sure they are not the same and not consists of undesire index
    while length(unique(bag_validation_indices)) ~=  length(bag_validation_indices) || ...`
          any(ismember(bag_validation_indices, skip_indices)) || ...
          any(ismember(bag_validation_indices, bag_training_indices)) 
       bag_validation_indices = randi(length(bag_with_tag_list), 1, opt.num_validation);
    end
else
    % overwrite
    bag_training_indices = trained_ids;
    bag_validation_indices = linspace(1, length(bag_with_tag_list), length(bag_with_tag_list));
    bag_validation_indices([trained_ids skip_indices]) = [];
end
bag_chosen_indices = [bag_training_indices, bag_validation_indices];



% save into results into folder         
save_name = "Paper-C71";
diary Paper-C71-diary % save terminal outputs

ans_error_big_matrix = [];
ans_counting_big_matrix = [];

if skip
    load(load_dir + 'saved_chosen_indices.mat');
    load(load_dir + 'saved_parameters.mat');
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
fprintf("-- number of refinement: %i\n", opt.num_refinement)
fprintf("-- number of LiDARTag's poses: %i\n", opt.num_lidar_target_pose)
fprintf("-- number of scan to optimize a LiDARTag pose: %i\n", opt.num_scan)
c = datestr(datetime); 
save_dir = save_name + "/" + c + "/";


if ~skip
    mkdir(save_dir);
    save(save_dir + 'saved_parameters.mat', 'opt', 'validation_flag');
    save(save_dir + 'saved_chosen_indices.mat', 'skip_indices', 'bag_training_indices', 'bag_validation_indices', 'bag_chosen_indices');
end

% loading training image
for i = 1:opt.num_training
    current_index = bag_training_indices(i);
    loadBagImg(training_img_fig_handles(i), bag_file_path + bag_with_tag_list(current_index), "not display", "not clean");
end

for i = 1:opt.num_validation
    current_index = bag_validation_indices(i);
    loadBagImg(validation_fig_handles(i), bag_file_path + bag_with_tag_list(current_index), "not display", "not clean");
end


if skip == 1 || skip == 2
    % load saved data
    load(load_dir + "X_base_line.mat");
    load(load_dir + "X_train.mat");
    load(load_dir + "Y.mat")
    load(load_dir + "save_validation.mat")
    load(load_dir + "array.mat")
    load(load_dir + "BagData.mat")
else
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
    
    for i = 1:opt.num_lidar_target_pose
        fprintf("--- Working on scan: %i/%i\n", i, opt.num_lidar_target_pose)
        validation_counter = 1;
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
                    image_num = i;
                    pc_iter = opt.num_scan*(i-1) + 1;
                    
                    % optimize lidar targets corners
                    [BagData(current_index), H_LT] = get4CornersReturnHLT(i, j, opt_H_TL, ...
                                                         mat_file_path, BagData(current_index), ...
                                                         pc_iter, opt.num_scan);
                    % draw camera targets 
                    BagData(current_index).camera_target(j).four_corners_line = ...
                                                point2DToLineForDrawing(BagData(current_index).camera_target(j).corners);
                    showLinedLiDARTag(training_pc_fig_handles(k), ...
                                      BagData(current_index).lidar_target(j).scan(i), "display");
                    showLinedAprilTag(training_img_fig_handles(k), ...
                                      BagData(current_index).camera_target(j), "display");
                    drawnow
                    X_training_tmp = [X_training_tmp, BagData(current_index).lidar_target(j).scan(i).corners];
                    Y_training_tmp = [Y_training_tmp, BagData(current_index).camera_target(j).corners];
                    H_LT_tmp = [H_LT_tmp, H_LT];
                    train_num_tag_array = [train_num_tag_array, BagData(current_index).num_tag];
                    train_tag_size_array = [train_tag_size_array, BagData(current_index).lidar_target(j).tag_size];
                end

                % 4 x M*i, M is correspondance per scan, i is scan
                X_train = [X_train, X_training_tmp]; 

                % 3 x M*i, M is correspondance per image, i is image
                Y_train = [Y_train, Y_training_tmp]; 
                fprintf(" Got training set: %s\n", bag_with_tag_list(current_index))
                
                % base line
                pc_iter = opt.num_scan*(i-1) + 1;
                [corners_big, edges]= KaessNewConstraintCorners(mat_file_path, ...
                                        BagData(current_index).lidar_target(1).pc_file, pc_iter);
                X_base_line = [X_base_line, corners_big];
                Y_base_line = [Y_base_line, BagData(current_index).camera_target(1).corners]; %% use big tag
                X_base_line_edge_points = [X_base_line_edge_points, edges];
                H_LT_big = [H_LT_big, H_LT_tmp];

            else
                %%% validation set
                if validation_counter > opt.num_validation
                    break;
                end

                X_validation_tmp = [];
                Y_validation_tmp = [];

                for j = 1:BagData(current_index).num_tag
                    image_num = i;
                    pc_iter = opt.num_scan*(i-1) + 1; 
                    [BagData(current_index), ~] = get4CornersReturnHLT(i, j, opt_H_TL, ...
                                                         mat_file_path, BagData(current_index), ...
                                                         pc_iter, opt.num_scan);

                    BagData(current_index).camera_target(j).four_corners_line = ...
                                                point2DToLineForDrawing(BagData(current_index).camera_target(j).corners);
                    showLinedLiDARTag(validation_pc_fig_handles(validation_counter), ...
                                      BagData(current_index).lidar_target(j).scan(i), "display");
                    showLinedAprilTag(validation_fig_handles(validation_counter), ...
                                      BagData(current_index).camera_target(j), "display");
                    drawnow
                    X_validation_tmp = [X_validation_tmp, BagData(current_index).lidar_target(j).scan(i).corners];
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
        fprintf("--- Finished scan: %i\n", i)
    end
    drawnow
    save(save_dir + 'X_base_line.mat', 'X_base_line');
    save(save_dir + 'X_train.mat', 'X_train', 'H_LT_big', 'X_base_line_edge_points');
    save(save_dir + 'array.mat', 'train_num_tag_array', 'train_tag_size_array', 'validation_num_tag_array', 'validation_tag_size_array');
    save(save_dir + 'Y.mat', 'Y_train', 'Y_base_line');
    save(save_dir + 'BagData.mat', 'BagData');
    save(save_dir + 'save_validation.mat', 'X_validation', 'Y_validation');
end

%
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
            [SNR_H_LC, SNR_P, SNR_opt_total_cost] = optimize4Points(opt_H_TC.rpy_init,...
                                                                    X_square_no_refinement, Y_train, ...
                                                                    intrinsic_matrix, display); 
            % NOT square withOUT refinement
            [NSNR_H_LC, NSNR_P, NSNR_opt_total_cost] = optimize4Points(opt_H_TC.rpy_init, ...
                                                                       X_base_line, Y_base_line, ... 
                                                                       intrinsic_matrix, display); 

            for i = 0: opt.num_refinement-1
                disp('---------------------')
                disp(' Optimizing H_LC ...')
                disp('---------------------')

                % square with refinement
                [SR_H_LC, SR_P, SR_opt_total_cost] = optimize4Points(opt_H_TC.rpy_init, ...
                                                                     X_train, Y_train, ... 
                                                                     intrinsic_matrix, display); 
                % NOT square with refinement
                [NSR_H_LC, NSR_P, NSR_opt_total_cost] = optimize4Points(opt_H_TC.rpy_init, ...
                                                                        X_not_square_refinement, Y_base_line, ...
                                                                        intrinsic_matrix, display); 
                
                if i == opt.num_refinement-1
                    break;
                else
                    disp('------------------')
                    disp(' Refining H_LT ...')
                    disp('------------------')
                    X_train = regulizedFineTuneLiDARTagPose(train_tag_size_array, ...
                                                            X_train, Y_train, H_LT_big, SR_P, ...
                                                            correspondance_per_pose, display);
                    X_not_square_refinement = regulizedFineTuneKaessCorners(X_not_square_refinement, Y_base_line,...
                                                                            X_base_line_edge_points, NSR_P, ...
                                                                            correspondance_per_pose, display);
                end
            end

        case "IoU"
            % one shot calibration (*-NR)
            [SNR_H_LC, SNR_P, SNR_opt_total_cost] = optimize4Points(opt_H_TC.rpy_init, ...
                                                                    X_square_no_refinement, Y_train, ...
                                                                    intrinsic_matrix, display); % square withOUT refinement
            [NSNR_H_LC, NSNR_P, NSNR_opt_total_cost] = optimize4Points(opt_H_TC.rpy_init, ...
                                                                       X_base_line, Y_base_line, ...
                                                                       intrinsic_matrix, display); % NOT square withOUT refinement

            if Alternating
                for i = 1: opt.num_refinement
                    disp('---------------------')
                    disp(' Optimizing H_LC ...')
                    disp('---------------------')

                    [SR_H_LC, SR_P, SR_opt_total_cost] = optimizeIoU(X_train, Y_train, intrinsic_matrix); % square with refinement
                    [NSR_H_LC, NSR_P, NSR_opt_total_cost] = optimizeIoU(X_not_square_refinement, Y_base_line, intrinsic_matrix); % NOT square with refinement
                    if i == opt.num_refinement
                        break;
                    else
                        disp('------------------')
                        disp(' Refining H_LT ...')
                        disp('------------------')

                        X_train = regulizedFineTuneLiDARTagPose(train_tag_size_array, ...
                                                                X_train, Y_train, H_LT_big, SR_P, ...
                                                                correspondance_per_pose, display);
                        X_not_square_refinement = regulizedFineTuneKaessCorners(X_not_square_refinement, ...
                                                                Y_base_line, X_base_line_edge_points, NSR_P, ...
                                                                correspondance_per_pose, display);
                    end
                end
            end
    end
    if skip == 0
        save(save_dir + 'save_validation.mat', 'X_validation', 'Y_validation');
        save(save_dir + 'NSNR.mat', 'NSNR_H_LC', 'NSNR_P', 'NSNR_opt_total_cost');
        save(save_dir + 'SNR.mat', 'SNR_H_LC', 'SNR_P', 'SNR_opt_total_cost');
        save(save_dir + 'NSR.mat', 'NSR_H_LC', 'NSR_P', 'NSR_opt_total_cost');
        save(save_dir + 'SR.mat',  'SR_H_LC', 'SR_P', 'SR_opt_total_cost');
    elseif skip == 1
        save(load_dir + 'save_validation.mat', 'X_validation', 'Y_validation');
        save(load_dir + 'NSNR.mat', 'NSNR_H_LC', 'NSNR_P', 'NSNR_opt_total_cost');
        save(load_dir + 'SNR.mat', 'SNR_H_LC', 'SNR_P', 'SNR_opt_total_cost');
        save(load_dir + 'NSR.mat', 'NSR_H_LC', 'NSR_P', 'NSR_opt_total_cost');
        save(load_dir + 'SR.mat',  'SR_H_LC', 'SR_P', 'SR_opt_total_cost');
    end
end

% load saved data
if skip == 2
    load(load_dir + "NSNR.mat");
    load(load_dir + "SNR.mat");
    load(load_dir + "NSR.mat");
    load(load_dir + "SR.mat");
    load(load_dir + "save_validation.mat")
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
disp(NSNR_opt_total_cost)
disp(' Training Error Per Corner (pixel)')
disp(NSNR_opt_total_cost/size(Y_base_line, 2)) % 2 tags, 4 corners
ans_error_submatrix = [bag_training_indices(1); NSNR_opt_total_cost/size(Y_base_line, 2)];

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
disp(NSR_opt_total_cost)
disp(' Training Error Per Corner (pixel)')
disp(NSR_opt_total_cost/size(Y_base_line, 2)) % 2 tags, 4 corners
ans_error_submatrix = [ans_error_submatrix; NSR_opt_total_cost/size(Y_base_line, 2)];

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
disp(SNR_opt_total_cost)
disp(' Training Error Per Corner (pixel)')
disp(SNR_opt_total_cost/size(Y_train, 2)) % 2 tags, 4 corners
ans_error_submatrix = [ans_error_submatrix; SNR_opt_total_cost/size(Y_base_line, 2)];

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
disp(SR_opt_total_cost)
disp(' Training Error Per Corner (pixel)')
disp(SR_opt_total_cost/size(Y_train, 2)) % 2 tags, 4 corners
ans_error_submatrix = [ans_error_submatrix; SR_opt_total_cost/size(Y_base_line, 2)];
ans_error_big_matrix = [ans_error_submatrix];

if length(bag_training_indices)>1
    for i = 2:length(bag_training_indices)
        index = bag_training_indices(i);
        ans_error_submatrix(1) = index;
        ans_error_big_matrix = [ans_error_big_matrix, ans_error_submatrix];
    end
end

%%% verify corner accuracy
if validation_flag
    SR_validation_cost = verifyCornerAccuracyWRTDataset(bag_validation_indices, validation_num_tag_array, opt, BagData, SR_P);
    NSR_validation_cost = verifyCornerAccuracyWRTDataset(bag_validation_indices, validation_num_tag_array, opt, BagData, NSR_P);
    SNR_validation_cost = verifyCornerAccuracyWRTDataset(bag_validation_indices, validation_num_tag_array, opt, BagData, SNR_P);
    NSNR_validation_cost = verifyCornerAccuracyWRTDataset(bag_validation_indices, validation_num_tag_array, opt, BagData, NSNR_P);

    [t_SNR_count, t_SR_count]   = inAndOutBeforeAndAfter(bag_training_indices, ...
                                                         opt.num_training, opt, BagData, SNR_P, SR_P);
    [t_NSNR_count, t_NSR_count] = inAndOutBeforeAndAfter(bag_training_indices, ...
                                                         opt.num_training, opt, BagData, NSNR_P, NSR_P);
    [SNR_count, SR_count]       = inAndOutBeforeAndAfter(bag_validation_indices, ...
                                                         opt.num_validation, opt, BagData, SNR_P, SR_P);
    [NSNR_count, NSR_count]     = inAndOutBeforeAndAfter(bag_validation_indices, ...
                                                         opt.num_validation, opt, BagData, NSNR_P, NSR_P);
end


if validation_flag
    disp("***************** validation Error*****************")
    for i = 1:opt.num_validation
        disp('------')
        current_index = bag_validation_indices(i);
        fprintf("---dataset: %s\n", bag_with_tag_list(current_index))
        ans_error_submatrix = [bag_validation_indices(i)];
        disp("-- Error Per Corner (pixel)")
        disp(' NSNR validation Error Per Corner (pixel)')
        disp(NSNR_validation_cost(i).total_cost/ size(Y_validation, 2))
        ans_error_submatrix = [ans_error_submatrix; SNR_opt_total_cost/size(Y_base_line, 2)];
        disp(' NSR validation Error Per Corner (pixel)')
        disp(NSR_validation_cost(i).total_cost/ size(Y_validation, 2))
        ans_error_submatrix = [ans_error_submatrix; SNR_opt_total_cost/size(Y_base_line, 2)];
        disp(' SNR validation Error Per Corner (pixel)')
        disp(SNR_validation_cost(i).total_cost/ size(Y_validation, 2))
        ans_error_submatrix = [ans_error_submatrix; SNR_opt_total_cost/size(Y_base_line, 2)];
        disp(' SR validation Error Per Corner (pixel)')
        disp(SR_validation_cost(i).total_cost/ size(Y_validation, 2))
        ans_error_submatrix = [ans_error_submatrix; SNR_opt_total_cost/size(Y_base_line, 2)];
        ans_error_big_matrix = [ans_error_big_matrix, ans_error_submatrix];
    end
    
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


if ~validation_flag
    save(save_dir + 'validation_cost' , 'SR_validation_cost', 'SNR_validation_cost', 'NSR_validation_cost', 'NSNR_validation_cost');
end

%%% draw results
% project training results
for i = 1:opt.num_training % which dataset
    current_index = bag_training_indices(i);

    for j = 1:BagData(current_index).num_tag % which target
        current_corners_SR = [BagData(current_index).lidar_target(j).scan(:).corners];
        current_X_SR = [BagData(current_index).lidar_target(j).scan(:).pc_points];
        prjectBackToImage(training_img_fig_handles(i), SR_P, current_corners_SR, 5, 'g*', "training_SR", "display", "Not-Clean");
        prjectBackToImage(training_img_fig_handles(i), SR_P, current_X_SR, 3, 'r.', "training_SR", "display", "Not-Clean");
    end
 end
drawnow

% project validation results
if validation_flag
    for i = 1:opt.num_validation % which dataset
        current_index = bag_validation_indices(i);
        for j = 1:BagData(current_index).num_tag % which target
            current_corners = [BagData(current_index).lidar_target(j).scan(:).corners];
            current_target_pc = [BagData(current_index).lidar_target(j).scan(:).pc_points];
            
            if size(current_corners, 1) ~= 4
                current_corners = [current_corners; 
                                   ones(1, size(current_corners, 2))];
            end

            if size(current_target_pc, 1) ~= 4
                current_target_pc = [current_target_pc; 
                                     ones(1, size(current_target_pc, 2))];
            end
            prjectBackToImage(validation_fig_handles(i), SR_P, current_corners, 5, 'g*', ...
                              "validation_SR", "display", "Not-Clean");
            prjectBackToImage(validation_fig_handles(i), SR_P, current_target_pc, 5, 'r.', ...
                              "validation_SR", "display", "Not-Clean");
        end
    end
end

% project testing results
% load testing images and testing pc mat
testing_set_pc = loadTestingMatFiles(mat_file_path, test_pc_mat_list);
for i = 1: size(bag_testing_list, 2)
    loadBagImg(testing_fig_handles(i), bag_file_path + bag_testing_list(i), "display", "Not clean"); 
    prjectBackToImage(testing_fig_handles(i), SR_P, testing_set_pc(i).mat_pc, 3, 'g.', "testing", "display", "Not-Clean");
end
drawnow
disp("********************************************")
disp("Projected using:")
disp(SR_P)
disp("********************************************")






         
