%% 
clc
addpath('/home/brucebot/test')
path = "/home/brucebot/workspace/griztag/src/griz_tag/bagfiles/matlab/";
% file = "3Tags-OldLiDAR.bag";
% file = "lab_angled.bag";
% file = "lab2-closer.bag";
file = "lab4-closer-cleaner.bag";
% file = "wall1.bag";
% file = "wavefield_3tag_closer.bag";
bagselect = rosbag(path + file);
bagselect2 = select(bagselect,'Time',...
    [bagselect.StartTime bagselect.StartTime + 1],'Topic','/camera/color/image_raw');
allMsgs = readMessages(bagselect2);
[img,~] = readImage(allMsgs{1});
figure(1000)
imshow(img)
title(file)
gray = rgb2gray(img);
BW = edge(gray, 'Canny',[0.05]);
figure(2000)
imshow(BW)
title(file)
