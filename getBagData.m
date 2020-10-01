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

function [BagData, TestData] = getBagData()
  
  % Choose alphasense camera:
  cam0 = 1;
  cam1 = 0;

  if cam0


    % Test Data:
    TestData(1).bagfile = "position4_undistorted_cam0_2020-08-04-11-34-58.bag";
    TestData(1).pc_file = "point_cloud-full-pcl-pos4-final--2020-08-03-11-16.mat";


    %%% tag size: largest -> smallest 
    %%% parameter for dataset
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % position1:
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    BagData(1).bagfile = "position1_undistorted_cam0_2020-08-04-11-27-56.bag";
    BagData(1).num_tag = 2;
    BagData(1).lidar_target(1).pc_file = 'point_cloud-small-position1-final--2020-07-29-16-11.mat'; %% payload: 3Tags-OldLiDAR.bag
    BagData(1).lidar_target(1).tag_size = 0.5;
    BagData(1).camera_target(1).corners = [721, 642, 795, 709;
                                250, 317, 326, 394;
                                1, 1, 1, 1];
    BagData(1).lidar_target(2).pc_file = 'point_cloud-large-pos1-final--2020-08-05-09-58.mat'; %% payload: 3Tags-OldLiDAR.bag
    BagData(1).lidar_target(2).tag_size = 0.75;
    BagData(1).camera_target(2).corners = [501, 424, 561, 487;
                                247, 306, 320, 380;
                                1, 1, 1, 1];       
    BagData(1).lidar_full_scan = "point_cloud-full-pcl-position1--2020-07-29-13-37.mat";
                  


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % position2:
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%         
    BagData(2).bagfile = "position2_undistorted_cam0_2020-08-04-11-30-53.bag";
    BagData(2).num_tag = 2;
    BagData(2).lidar_target(1).pc_file = 'point_cloud-small-position2-final--2020-07-30-16-22.mat'; %% payload: lab2-closer.bag
    BagData(2).lidar_target(1).tag_size = 0.5;
    BagData(2).camera_target(1).corners = [459, 387, 514, 449;
                                246, 298, 311, 364;
                                1, 1, 1, 1];
    BagData(2).lidar_target(2).pc_file = 'point_cloud-large-pos2-final--2020-08-05-10-34.mat'; %% payload: lab2-closer.bag
    BagData(2).lidar_target(2).tag_size = 0.75;
    BagData(2).camera_target(2).corners = [669, 600, 725, 656;
                                239, 296, 307, 363;
                                1, 1, 1, 1];  
    BagData(2).lidar_full_scan = "point_cloud-full-pcl-position2--2020-07-30-17-34.mat";
 


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % position3:
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%         
    BagData(3).bagfile = "position3_undistorted_cam0_2020-08-04-11-33-07.bag"; 
    BagData(3).num_tag = 2;
    BagData(3).lidar_target(1).pc_file = 'point_cloud-small-pos3-final--2020-08-03-09-04.mat'; %% somewhat rotated, there are some pole points on the top, might need better filtering!
    BagData(3).lidar_target(1).tag_size = 0.5;
    BagData(3).camera_target(1).corners = [453, 383, 506, 443;
                                    248, 299, 311, 363;
                                    1, 1, 1, 1];
    BagData(3).lidar_target(2).pc_file = 'point_cloud-large-pos3-final--2020-08-03-09-34.mat'; %% Also rotated, but well filtered, just not determined enough
    BagData(3).lidar_target(2).tag_size = 0.75;
    BagData(3).camera_target(2).corners = [784, 718, 846, 772;
                                    251, 304, 311, 364;
                                    1, 1, 1, 1];    
    BagData(3).lidar_full_scan = "point_cloud-full-pcl-pos3-final--2020-08-03-11-17.mat";



    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % position4
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%         
    BagData(4).bagfile = "position4_undistorted_cam0_2020-08-04-11-34-58.bag";
    BagData(4).num_tag = 2;
    BagData(4).lidar_target(1).pc_file = 'point_cloud-small-pos4-final--2020-08-03-10-38.mat'; %% Wrong rotation
    BagData(4).lidar_target(1).tag_size = 0.5;
    BagData(4).camera_target(1).corners = [479, 414, 532, 467;
                                    304, 356, 367, 420;
                                    1, 1, 1, 1];
    BagData(4).lidar_target(2).pc_file = 'point_cloud-large-pos4-final--2020-08-03-10-04.mat'; %% Too flat on the top!
    BagData(4).lidar_target(2).tag_size = 0.75;
    BagData(4).camera_target(2).corners = [719, 629, 800, 702;
                                    261, 334, 351, 423;
                                    1, 1, 1, 1];
    BagData(4).lidar_full_scan = "point_cloud-full-pcl-pos4-final--2020-08-03-11-16.mat";



    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % position5:
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%         
    BagData(5).bagfile = "position5_undistorted_cam0_2020-08-04-11-37-21.bag";
    BagData(5).num_tag = 2;
    BagData(5).lidar_target(1).pc_file = 'point_cloud-small-pos5-final--2020-08-03-11-12.mat'; %% payload: 3Tags-OldLiDAR.bag
    BagData(5).lidar_target(1).tag_size = 0.5;
    BagData(5).camera_target(1).corners = [750, 662, 845, 738;
                                   297, 370, 390, 460;
                                   1, 1, 1, 1];
    BagData(5).lidar_target(2).pc_file = 'point_cloud-large-pos5-final--2020-08-03-10-56.mat'; %% payload
    BagData(5).lidar_target(2).tag_size = 0.75;
    BagData(5).camera_target(2).corners = [450, 334, 529, 427;
                                   247, 325, 346, 428;
                                   1, 1, 1, 1];
    BagData(5).lidar_full_scan = "point_cloud-full-pcl-pos5-final--2020-08-03-11-13.mat";

  end



  %%%%%%%%%%%%%%%%%%%%%%%%%%
  % CAM 1 
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%

  if cam1


    % Test Data:
    TestData(1).bagfile = "position3_undistorted_cam1_2020-08-06-13-26-12.bag";
    TestData(1).pc_file = "point_cloud-full-pcl-pos3-final--2020-08-03-11-17.mat";


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % position1:
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    BagData(1).bagfile = "position1_undistorted_cam1_2020-08-06-13-11-20.bag";
    BagData(1).num_tag = 2;
    BagData(1).lidar_target(1).pc_file = 'point_cloud-small-position1-final--2020-07-29-16-11.mat'; %% payload: 3Tags-OldLiDAR.bag
    BagData(1).lidar_target(1).tag_size = 0.5;
    BagData(1).camera_target(1).corners = [681, 604, 757, 673;
                                252, 321, 328, 397;
                                1, 1, 1, 1];
    BagData(1).lidar_target(2).pc_file = 'point_cloud-large-pos1-final--2020-08-05-09-58.mat'; %% payload: 3Tags-OldLiDAR.bag
    BagData(1).lidar_target(2).tag_size = 0.75;
    BagData(1).camera_target(2).corners = [471, 395, 532, 459;
                                253, 314, 326, 387;
                                1, 1, 1, 1];       
    BagData(1).lidar_full_scan = "point_cloud-full-pcl-position1--2020-07-29-13-37.mat";
                  


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % position2:
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%         
    BagData(2).bagfile = "position2_undistorted_cam1_2020-08-06-13-25-33.bag";
    BagData(2).num_tag = 2;
    BagData(2).lidar_target(1).pc_file = 'point_cloud-small-position2-final--2020-07-30-16-22.mat'; %% payload: lab2-closer.bag
    BagData(2).lidar_target(1).tag_size = 0.5;
    BagData(2).camera_target(1).corners = [423, 351, 481, 415;
                                252, 306, 317, 371;
                                1, 1, 1, 1];
    BagData(2).lidar_target(2).pc_file = 'point_cloud-large-pos2-final--2020-08-05-10-34.mat'; %% payload: lab2-closer.bag
    BagData(2).lidar_target(2).tag_size = 0.75;
    BagData(2).camera_target(2).corners = [636, 568, 695, 625;
                                242, 300, 310, 368;
                                1, 1, 1, 1];  
    BagData(2).lidar_full_scan = "point_cloud-full-pcl-position2--2020-07-30-17-34.mat";
 


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % position3:
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%         
    BagData(3).bagfile = "position3_undistorted_cam1_2020-08-06-13-26-12.bag"; 
    BagData(3).num_tag = 2;
    BagData(3).lidar_target(1).pc_file = 'point_cloud-small-pos3-final--2020-08-03-09-04.mat'; %% payload: lab_angled.bag
    BagData(3).lidar_target(1).tag_size = 0.5;
    BagData(3).camera_target(1).corners = [418, 348, 474, 409;
                                    255, 307, 318, 370;
                                    1, 1, 1, 1];
    BagData(3).lidar_target(2).pc_file = 'point_cloud-large-pos3-final--2020-08-03-09-34.mat'; %% payload: lab_angled.bag
    BagData(3).lidar_target(2).tag_size = 0.75;
    BagData(3).camera_target(2).corners = [755, 690, 819, 745;
                                    251, 307, 312, 366;
                                    1, 1, 1, 1];    
    BagData(3).lidar_full_scan = "point_cloud-full-pcl-pos3-final--2020-08-03-11-17.mat";



    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % position4
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%         
    BagData(4).bagfile = "position4_undistorted_cam1_2020-08-06-13-27-02.bag";
    BagData(4).num_tag = 2;
    BagData(4).lidar_target(1).pc_file = 'point_cloud-small-pos4-final--2020-08-03-10-38.mat'; %% payload: 3Tags-OldLiDAR.bag
    BagData(4).lidar_target(1).tag_size = 0.5;
    BagData(4).camera_target(1).corners = [445, 380, 499, 435;
                                    310, 364, 373, 427;
                                    1, 1, 1, 1];
    BagData(4).lidar_target(2).pc_file = 'point_cloud-large-pos4-final--2020-08-03-10-04.mat'; %% payload
    BagData(4).lidar_target(2).tag_size = 0.75;
    BagData(4).camera_target(2).corners = [685, 597, 768, 671;
                                    263, 338, 352, 426;
                                    1, 1, 1, 1];
    BagData(4).lidar_full_scan = "point_cloud-full-pcl-pos4-final--2020-08-03-11-16.mat";



    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % position5:
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%         
    BagData(5).bagfile = "position5_undistorted_cam1_2020-08-06-13-27-43.bag";
    BagData(5).num_tag = 2;
    BagData(5).lidar_target(1).pc_file = 'point_cloud-small-pos5-final--2020-08-03-11-12.mat'; %% payload: 3Tags-OldLiDAR.bag
    BagData(5).lidar_target(1).tag_size = 0.5;
    BagData(5).camera_target(1).corners = [709, 624, 804, 700;
                                   298, 374, 391, 464;
                                   1, 1, 1, 1];
    BagData(5).lidar_target(2).pc_file = 'point_cloud-large-pos5-final--2020-08-03-10-56.mat'; %% payload
    BagData(5).lidar_target(2).tag_size = 0.75;
    BagData(5).camera_target(2).corners = [414, 298, 496, 394;
                                   254, 334, 352, 436;
                                   1, 1, 1, 1];
    BagData(5).lidar_full_scan = "point_cloud-full-pcl-pos5-final--2020-08-03-11-13.mat";

  end