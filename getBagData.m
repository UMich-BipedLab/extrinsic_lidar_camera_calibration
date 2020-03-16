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

    TestData(1).bagfile = "EECS3.bag";
    TestData(1).pc_file = "velodyne_points-EECS3--2019-09-06-06-19.mat";

    TestData(2).bagfile = "verification2-45.bag";
    TestData(2).pc_file = "velodyne_points-verification2--2019-09-03-23-02.mat";
    
    TestData(3).bagfile = "verification3-45.bag";
    TestData(3).pc_file = "velodyne_points-verification3--2019-09-03-23-03.mat";
    
    TestData(4).bagfile = "grove2.bag";
    TestData(4).pc_file = "velodyne_points-grove2--2019-09-06-06-20.mat";
    
    TestData(5).bagfile = "verification5-45.bag";
    TestData(5).pc_file = "velodyne_points-verification5--2019-09-03-23-03.mat";
    
    TestData(6).bagfile = "outdoor6-notag.bag";
    TestData(6).pc_file = "velodyne_points-outdoor6-NoTag--2019-09-06-10-31.mat";
    
    TestData(7).bagfile = "outdoor4.bag";
    TestData(7).pc_file = "velodyne_points-outdoor4--2019-09-04-18-16.mat";
    
    TestData(8).bagfile = "outdoor5.bag";
    TestData(8).pc_file = "velodyne_points-outdoor5--2019-09-04-18-20.mat";
    

    %%% tag size: largest -> smallest 
    %%% parameter for dataset
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % 3Tags-OldLiDAR.bag dataset
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    BagData(1).bagfile = "3Tags-OldLiDAR.bag";
    BagData(1).num_tag = 3;
    BagData(1).lidar_target(1).pc_file = 'velodyne_points-3Tags-OldLiDAR-largest--2019-09-03-08-26.mat'; %% payload: 3Tags-OldLiDAR.bag
    BagData(1).lidar_target(1).tag_size = 0.8051;
    BagData(1).lidar_target(2).pc_file = 'velodyne_points-3Tags-OldLiDAR-medium--2019-09-02-20-53.mat'; %% payload
    BagData(1).lidar_target(2).tag_size = 0.225;
    BagData(1).lidar_target(3).pc_file = 'velodyne_points-3Tags-OldLiDAR-small--2019-09-02-20-47.mat'; %% payload
    BagData(1).lidar_target(3).tag_size = 0.158;
    BagData(1).lidar_full_scan = "velodyne_points-3Tags-full-pc--2019-09-01-02-25.mat";
    BagData(1).camera_target(1).corners = [526, 447, 569, 490;
                                           261, 297, 341, 379;
                                           1, 1, 1, 1];                         
    BagData(1).camera_target(2).corners = [269, 237, 284, 251;
                                           296, 313, 326, 343;
                                           1, 1, 1, 1];
    BagData(1).camera_target(3).corners = [394, 349, 413, 367;
                                           249, 267, 294, 312;
                                           1, 1, 1, 1]; 



    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % lab2-closer.bag dataset
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%         
    BagData(2).bagfile = "lab2-closer.bag";
    BagData(2).num_tag = 2;
    BagData(2).lidar_target(1).pc_file = 'velodyne_points-lab2-closer-big--2019-09-05-21-51.mat'; %% payload: lab2-closer.bag
    BagData(2).lidar_target(1).tag_size = 0.8051;
    BagData(2).lidar_target(2).pc_file = 'velodyne_points-lab2-closer-small--2019-09-05-21-53.mat'; %% payload
    BagData(2).lidar_target(2).tag_size = 0.158;
    BagData(2).lidar_full_scan = "velodyne_points-lab2-full-pc--2019-09-05-23-20.mat";
    BagData(2).camera_target(1).corners = [340, 263, 406, 316;
                                236, 313, 341, 417;
                                1, 1, 1, 1];    
    BagData(2).camera_target(2).corners = [197, 153, 220, 176;
                                 250, 273, 292, 315;
                                 1, 1, 1, 1];



    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % lab_angled.bag dataset
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%         
    BagData(3).bagfile = "lab_angled.bag"; 
    BagData(3).num_tag = 1;
    BagData(3).lidar_target(1).pc_file = 'velodyne_points-lab_angled-big--2019-09-05-21-25.mat'; %% payload: lab_angled.bag
    BagData(3).lidar_target(1).tag_size = 0.8051;
    BagData(3).lidar_full_scan = "velodyne_points-lab_angled-full-pc--2019-09-06-14-03.mat";
    BagData(3).camera_target(1).corners = [340, 263, 406, 316;
										236, 313, 341, 417;
										1, 1, 1, 1];    



    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % lab3-closer-cleaner.bag dataset
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%         
    BagData(4).bagfile = "lab3-closer-cleaner.bag";
    BagData(4).num_tag = 2;
    BagData(4).lidar_target(1).pc_file = 'velodyne_points-lab3-closer-big--2019-09-06-08-38.mat'; %% payload: 3Tags-OldLiDAR.bag
    BagData(4).lidar_target(1).tag_size = 0.8051;
    BagData(4).lidar_target(2).pc_file = 'velodyne_points-lab3-closer-small--2019-09-06-08-35.mat'; %% payload
    BagData(4).lidar_target(2).tag_size = 0.158;
    BagData(4).lidar_full_scan = "velodyne_points-lab3-closer-full-scan--2019-09-06-08-28.mat";
    BagData(4).camera_target(2).corners = [200, 157, 223, 180;
                                    251, 275, 292, 315;
                                    1, 1, 1, 1];
    BagData(4).camera_target(1).corners = [333, 248, 418, 328;
                                   239, 322, 328, 416;
                                   1, 1, 1, 1];    



    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % lab4-closer-cleaner.bag dataset
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%         
    BagData(5).bagfile = "lab4-closer-cleaner.bag";
    BagData(5).num_tag = 2;
    BagData(5).lidar_target(1).pc_file = 'velodyne_points-lab4-closer-big--2019-09-06-13-49.mat'; %% payload: 3Tags-OldLiDAR.bag
    BagData(5).lidar_target(1).tag_size = 0.8051;
    BagData(5).lidar_target(2).pc_file = 'velodyne_points-lab4-closer-small--2019-09-06-13-38.mat'; %% payload
    BagData(5).lidar_target(2).tag_size = 0.158;
    BagData(5).lidar_full_scan = "velodyne_points-lab4-closer-full-pc--2019-09-06-13-34.mat";
    BagData(5).camera_target(2).corners = [250, 206, 272, 230;
                                   257, 281, 299, 323;
                                   1, 1, 1, 1];
    BagData(5).camera_target(1).corners = [473, 349, 575, 435;
                                   227, 314, 361, 448;
                                   1, 1, 1, 1];    


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % lab5-closer-cleaner.bag dataset
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%         
    BagData(6).bagfile = "lab5-closer-cleaner.bag";
    BagData(6).num_tag = 2;
    BagData(6).lidar_target(1).pc_file = 'velodyne_points-lab5-closer-bag--2019-09-06-14-27.mat'; %% payload: 3Tags-OldLiDAR.bag
    BagData(6).lidar_target(1).tag_size = 0.8051;
    BagData(6).lidar_target(2).pc_file = 'velodyne_points-lab5-closer-small--2019-09-06-14-23.mat'; %% payload
    BagData(6).lidar_target(2).tag_size = 0.158;
    BagData(6).lidar_full_scan = "velodyne_points-lab5-closer-full-pc--2019-09-06-14-15.mat";
    BagData(6).camera_target(2).corners = [145, 105, 165, 127;
                                   263, 284, 299, 321;
                                   1, 1, 1, 1];
    BagData(6).camera_target(1).corners = [398, 281, 490, 367;
                                   233, 318, 349, 440;
                                   1, 1, 1, 1];     



    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % lab6-closer-cleaner.bag dataset
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%         
    BagData(7).bagfile = "lab6-closer-cleaner.bag";
    BagData(7).num_tag = 2;
    BagData(7).lidar_target(1).pc_file = 'velodyne_points-lab6-closer-big--2019-09-06-15-09.mat'; %% payload: 3Tags-OldLiDAR.bag
    BagData(7).lidar_target(1).tag_size = 0.8051;
    BagData(7).lidar_target(2).pc_file = 'velodyne_points-lab6-closer-small--2019-09-06-15-05.mat'; %% payload
    BagData(7).lidar_target(2).tag_size = 0.158;
    BagData(7).lidar_full_scan = "velodyne_points-lab6-closer-full-pc--2019-09-06-14-15.mat";
    BagData(7).camera_target(2).corners = [230, 191, 251, 213;
                                   260, 282, 298, 321;
                                   1, 1, 1, 1];
    BagData(7).camera_target(1).corners = [409, 314, 476, 372;
                                   246, 309, 349, 411;
                                   1, 1, 1, 1];    



    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % lab7-closer-cleaner.bag dataset
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%         
    BagData(8).bagfile = "lab7-closer-cleaner.bag";
    BagData(8).num_tag = 2;
    BagData(8).lidar_target(1).pc_file = 'velodyne_points-lab7-closer-big--2019-09-06-15-14.mat'; %% payload: 3Tags-OldLiDAR.bag
    BagData(8).lidar_target(1).tag_size = 0.8051;
    BagData(8).lidar_target(2).pc_file = 'velodyne_points-lab7-closer-small--2019-09-06-15-12.mat'; %% payload
    BagData(8).lidar_target(2).tag_size = 0.158;
    BagData(8).lidar_full_scan = "velodyne_points-lab7-closer-full-pc--2019-09-06-14-16.mat";
    BagData(8).camera_target(2).corners = [509, 479, 529, 498;
                                   274, 292, 305, 323;
                                   1, 1, 1, 1];
    BagData(8).camera_target(1).corners = [178, 79, 230, 137;
                                   253, 307, 342, 402;
                                   1, 1, 1, 1];    



    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % lab8-closer-cleaner.bag dataset
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%         
    BagData(9).bagfile = "lab8-closer-cleaner.bag";
    BagData(9).num_tag = 2;
    BagData(9).lidar_target(1).pc_file = 'velodyne_points-lab8-closer-big--2019-09-06-15-28.mat'; %% payload: 3Tags-OldLiDAR.bag
    BagData(9).lidar_target(1).tag_size = 0.8051;
    BagData(9).lidar_target(2).pc_file = 'velodyne_points-lab8-closer-small--2019-09-06-15-17.mat'; %% payload
    BagData(9).lidar_target(2).tag_size = 0.158;
    BagData(9).lidar_full_scan = "velodyne_points-lab8-closer-full-pc--2019-09-06-14-16.mat";
    BagData(9).camera_target(2).corners = [264, 226, 284, 246;
                                   258, 279, 296, 316;
                                   1, 1, 1, 1];
    BagData(9).camera_target(1).corners = [465, 372, 545, 445;
                                   222, 293, 318, 389;
                                   1, 1, 1, 1];    



    BagData(10).bagfile = "wavefield3-tag.bag";
    BagData(10).num_tag = 2;
    BagData(10).lidar_target(1).pc_file = 'velodyne_points-wavefield3-big--2019-09-07-19-04.mat'; %% payload: 3Tags-OldLiDAR.bag
    BagData(10).lidar_target(1).tag_size = 0.8051;
    BagData(10).lidar_target(2).pc_file = 'velodyne_points-wavefield3-small--2019-09-07-20-18.mat'; %% payload
    BagData(10).lidar_target(2).tag_size = 0.158;
    BagData(10).lidar_full_scan = "velodyne_points-wavefield3-full-pc--2019-09-07-19-00.mat";
    BagData(10).camera_target(2).corners = [517, 489, 540, 512;
                                    268, 289, 297, 319;
                                    1, 1, 1, 1];
    BagData(10).camera_target(1).corners = [255, 215, 282, 243;
                                    284, 313, 328, 359;
                                    1, 1, 1, 1];    



    BagData(11).bagfile = "wavefield5-tag.bag";
    BagData(11).num_tag = 2;
    BagData(11).lidar_target(1).pc_file = 'velodyne_points-wavefield5-big--2019-09-07-20-24.mat'; %% payload: 3Tags-OldLiDAR.bag
    BagData(11).lidar_target(1).tag_size = 0.8051;
    BagData(11).lidar_target(2).pc_file = 'velodyne_points-wavefield5-small--2019-09-07-19-17.mat'; %% payload
    BagData(11).lidar_target(2).tag_size = 0.158;
    BagData(11).lidar_full_scan = "velodyne_points-wavefield5-full-pc--2019-09-07-19-01.mat";
    BagData(11).camera_target(2).corners = [483, 443, 517, 474;
                                    236, 273, 280, 317;
                                    1, 1, 1, 1];
    BagData(11).camera_target(1).corners = [168, 110, 204, 152;
                                    268, 317, 329, 383;
                                    1, 1, 1, 1];    

                                
    BagData(12).bagfile = "wavefield_3tag_closer.bag";
    BagData(12).num_tag = 3;
    BagData(12).lidar_target(1).pc_file = 'velodyne_points-wavefield_3tag_closer_big--2019-10-02-12-09.mat'; %% payload: 3Tags-OldLiDAR.bag
    BagData(12).lidar_target(1).tag_size = 1.216;
    BagData(12).lidar_target(2).pc_file = 'velodyne_points-wavefield_3tag_closer_medium--2019-10-02-12-06.mat'; %% payload
    BagData(12).lidar_target(2).tag_size = 0.8051;
    BagData(12).lidar_target(3).pc_file = 'velodyne_points-wavefield_3tag_closer_small--2019-10-02-12-02.mat'; %% payload
    BagData(12).lidar_target(3).tag_size = 0.158;
    BagData(12).lidar_full_scan = "velodyne_points-wavefield5-full-pc--2019-09-07-19-01.mat";
    BagData(12).camera_target(1).corners = [486, 443, 507, 464;
                                            312, 328, 356, 372;
                                            1, 1, 1, 1];
    BagData(12).camera_target(2).corners = [226, 194, 232, 198;
                                            309, 326, 349, 365;
                                            1, 1, 1, 1];
    BagData(12).camera_target(3).corners = [371, 348, 380, 358;
                                            355, 364, 378, 388;
                                            1, 1, 1, 1];                            
end
