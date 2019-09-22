function BagData = getBagData()
     %%% parameter for dataset
    BagData(1).bagfile = "3Tags-OldLiDAR.bag";
    BagData(1).name_biggest = 'velodyne_points-3Tags-OldLiDAR-largest--2019-09-03-08-26.mat'; %% payload: 3Tags-OldLiDAR.bag
    BagData(1).name_small = 'velodyne_points-3Tags-OldLiDAR-small--2019-09-02-20-47.mat'; %% payload
    BagData(1).name_medium = 'velodyne_points-3Tags-OldLiDAR-medium--2019-09-02-20-53.mat'; %% payload
    BagData(1).full_scan = "velodyne_points-3Tags-full-pc--2019-09-01-02-25.mat";

    BagData(2).bagfile = "lab2-closer.bag";
%             app.full_pointcloud_name = "velodyne_points-lab2-full-pc--2019-09-05-23-20.mat";
    BagData(2).name_small = 'velodyne_points-lab2-closer-small--2019-09-05-21-53.mat'; %% payload
    BagData(2).name_biggest = 'velodyne_points-lab2-closer-big--2019-09-05-21-51.mat'; %% payload: lab2-closer.bag
    BagData(2).full_scan = "velodyne_points-lab2-full-pc--2019-09-05-23-20.mat";

    BagData(3).bagfile = "lab_angled.bag"; 
    BagData(3).name_biggest = 'velodyne_points-lab_angled-big--2019-09-05-21-25.mat'; %% payload: lab_angled.bag
    BagData(3).full_scan = "velodyne_points-lab_angled-full-pc--2019-09-06-14-03.mat";


    BagData(4).bagfile = "lab3-closer-cleaner.bag";
    BagData(4).name_biggest = 'velodyne_points-lab3-closer-big--2019-09-06-08-38.mat'; %% payload: 3Tags-OldLiDAR.bag
    BagData(4).name_small = 'velodyne_points-lab3-closer-small--2019-09-06-08-35.mat'; %% payload
    BagData(4).full_scan = "velodyne_points-lab3-closer-full-scan--2019-09-06-08-28.mat";

    BagData(5).bagfile = "lab4-closer-cleaner.bag";
    BagData(5).name_biggest = 'velodyne_points-lab4-closer-big--2019-09-06-13-49.mat'; %% payload: 3Tags-OldLiDAR.bag
    BagData(5).name_small = 'velodyne_points-lab4-closer-small--2019-09-06-13-38.mat'; %% payload
    BagData(5).full_scan = "velodyne_points-lab4-closer-full-pc--2019-09-06-13-34.mat";

    BagData(6).bagfile = "lab5-closer-cleaner.bag";
    BagData(6).name_biggest = 'velodyne_points-lab5-closer-bag--2019-09-06-14-27.mat'; %% payload: 3Tags-OldLiDAR.bag
    BagData(6).name_small = 'velodyne_points-lab5-closer-small--2019-09-06-14-23.mat'; %% payload
    BagData(6).full_scan = "velodyne_points-lab5-closer-full-pc--2019-09-06-14-15.mat";

    BagData(7).bagfile = "lab6-closer-cleaner.bag";
    BagData(7).name_biggest = 'velodyne_points-lab6-closer-big--2019-09-06-15-09.mat'; %% payload: 3Tags-OldLiDAR.bag
    BagData(7).name_small = 'velodyne_points-lab6-closer-small--2019-09-06-15-05.mat'; %% payload
    BagData(7).full_scan = "velodyne_points-lab6-closer-full-pc--2019-09-06-14-15.mat";

    BagData(8).bagfile = "lab7-closer-cleaner.bag";
    BagData(8).name_biggest = 'velodyne_points-lab7-closer-big--2019-09-06-15-14.mat'; %% payload: 3Tags-OldLiDAR.bag
    BagData(8).name_small = 'velodyne_points-lab7-closer-small--2019-09-06-15-12.mat'; %% payload
    BagData(8).full_scan = "velodyne_points-lab7-closer-full-pc--2019-09-06-14-16.mat";


    BagData(9).bagfile = "lab8-closer-cleaner.bag";
    BagData(9).name_biggest = 'velodyne_points-lab8-closer-big--2019-09-06-15-28.mat'; %% payload: 3Tags-OldLiDAR.bag
    BagData(9).name_small = 'velodyne_points-lab8-closer-small--2019-09-06-15-17.mat'; %% payload
    BagData(9).full_scan = "velodyne_points-lab8-closer-full-pc--2019-09-06-14-16.mat";


    BagData(10).bagfile = "wavefield3-tag.bag";
    BagData(10).name_biggest = 'velodyne_points-wavefield3-big--2019-09-07-19-04.mat'; %% payload: 3Tags-OldLiDAR.bag
    BagData(10).name_small = 'velodyne_points-wavefield3-small--2019-09-07-20-18.mat'; %% payload
    BagData(10).full_scan = "velodyne_points-wavefield3-full-pc--2019-09-07-19-00.mat";


    BagData(11).bagfile = "wavefield5-tag.bag";
    BagData(11).name_biggest = 'velodyne_points-wavefield5-big--2019-09-07-20-24.mat'; %% payload: 3Tags-OldLiDAR.bag
    BagData(11).name_small = 'velodyne_points-wavefield5-small--2019-09-07-19-17.mat'; %% payload
    BagData(11).full_scan = "velodyne_points-wavefield5-full-pc--2019-09-07-19-01.mat";
end