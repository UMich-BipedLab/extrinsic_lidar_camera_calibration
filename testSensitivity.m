clc
intrinsic_matrix = [616.3681640625, 0.0,            319.93463134765625;
                    0.0,            616.7451171875, 243.6385955810547;
                    0.0, 0.0, 1.0];
Good_dataset = 11;
Good_transorm_r = 82.0122;
Good_transorm_p = -0.0192 ;
Good_transorm_h = 87.7953;
Good_transform_T = [0.0228
   -0.2070
   -0.0783];
                
Bad_dataset = 4;
Bad_transorm_r = 81.9883;
Bad_transorm_p = -0.5840;
Bad_transorm_h = 87.2922;
Bad_transform_T = [    0.0053
   -0.2040
   -0.0544];                
                
% Good transform, bad training data                
X = [BagData(Bad_dataset).lidar_target(1).scan(1).corners];
Y = [BagData(Bad_dataset).camera_target(1).corners];
disp("Good transform, bad training data")
cost = cost4Points(Good_transorm_r, Good_transorm_p, Good_transorm_h, Good_transform_T, X, Y, intrinsic_matrix)

% bad transform, bad training data
X = [BagData(Bad_dataset).lidar_target(1).scan(1).corners];
Y = [BagData(Bad_dataset).camera_target(1).corners];
disp("bad transform, bad training data")
cost = cost4Points(Bad_transorm_r, Bad_transorm_p, Bad_transorm_h, Bad_transform_T, X, Y, intrinsic_matrix)

% Good transform, good training data
X = [BagData(Good_dataset).lidar_target(1).scan(1).corners];
Y = [BagData(Good_dataset).camera_target(1).corners];
disp("Good transform, good training data")
cost = cost4Points(Good_transorm_r, Good_transorm_p, Good_transorm_h, Good_transform_T, X, Y, intrinsic_matrix)

% bad transform, good training data
X = [BagData(Good_dataset).lidar_target(1).scan(1).corners];
Y = [BagData(Good_dataset).camera_target(1).corners];
disp("bad transform, good training data")
cost = cost4Points(Bad_transorm_r, Bad_transorm_p, Bad_transorm_h, Bad_transform_T, X, Y, intrinsic_matrix)

