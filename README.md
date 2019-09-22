# extrinsic_lidar_camera_calibration
## Overview
This is a packagt for extrinsic calibration between a 3D LiDAR and a camera, describe in paper: Improvements to Target-Based 3D LiDAR to Camera Calibration. We evaluated our proposed methods and compared them with other approaches in a round-robin validation study, including qualitative results and quantitative results, which we use images corners as ground truth to evaluate our accuracy of projection.
* [note] ALL the code are still on the dev branch. Everything will be cleaned up and well-documented with in a week. To run the testing code, please checkout to the dev branch and the put the test files in folders and the change the two paths in the main.m and then hit run!

* Author: Bruce JK Huang and Jessy W. Grizzle
* Maintainer: [Bruce JK Huang](https://www.brucerobot.com/), brucejkh[at]gmail.com
* Affiliation: [The Biped Lab](https://www.biped.solutions/), the University of Michigan

This package has been tested under MATLAB 2019a and Ubuntu 16.04.
More detailed introduction will be updated in a week. Sorry for the inconvenient!

## Abstract
The homogeneous transformation between a LiDAR and monocular camera is required for sensor fusion tasks, such as SLAM. While determining such a transformation is not considered glamorous in any sense of the word, it is nonetheless crucial for many modern autonomous systems. Indeed, an error of a few degrees in rotation or a few percent in translation can lead to 20 cm translation errors at a distance of 5 m when overlaying a LiDAR image on a camera image. The biggest impediments to determining the transformation accurately are the relative sparsity of LiDAR point clouds and systematic errors in their distance measurements. This paper proposes (1) the use of targets of known dimension and geometry to ameliorate target pose estimation in face of the quantization and systematic errors inherent in a LiDAR image of a target, and (2) a fitting method for the LiDAR to monocular camera transformation that fundamentally assumes the camera image data is the most accurate information in one's possession. 

## Quick View
Using the obtained transformation, LiDAR points are mapped onto a semantically segmented image. Each point is associated with the label of a pixel. The road is marked as white; static objects such buildings as orange; the grass as yellow-green, and dark green indicates trees.

<img src="https://github.com/UMich-BipedLab/extrinsic_lidar_camera_calibration/blob/master/figure/semanticImg.png" width="640">
<img src="https://github.com/UMich-BipedLab/extrinsic_lidar_camera_calibration/blob/master/figure/semanticPC3.png" width="640">

# Why important? 
A calibration result is not usable if it has few degrees of rotation error and a few percent of translation error.
The below shows that a calibration result with little disturbance from the well-aigned image.

<img src="https://github.com/UMich-BipedLab/extrinsic_lidar_camera_calibration/blob/master/figure/disturbance.png" width="320"> 
<img src="https://github.com/UMich-BipedLab/extrinsic_lidar_camera_calibration/blob/master/figure/undisturbance.png" width="320">

# Why this package? (coming strong in a week)

## Overall pipeline (coming strong in a week)


## Presentation and Video (coming strong in a week)
https://www.brucerobot.com/

## Installation (coming strong in a week)
TODO

## Parameters (coming strong in a week)
TODO

## Examples (coming strong in a week)
TODO

# Qualitative results
For the method GL_1-R trained on S_1, the LiDAR point cloud has been projected into the image plane for the other data sets and marked in green. The red circles highlight various poles, door edges, desk legs, monitors, and sidewalk curbs where the quality of the alignment can be best judged. The reader may find other areas of interest. Enlarge in your browser for best viewing. 


<img src="https://github.com/UMich-BipedLab/extrinsic_lidar_camera_calibration/blob/master/figure/test1_3.png" width="640">
<img src="https://github.com/UMich-BipedLab/extrinsic_lidar_camera_calibration/blob/master/figure/test2_3.png" width="640">
<img src="https://github.com/UMich-BipedLab/extrinsic_lidar_camera_calibration/blob/master/figure/test3_3.png" width="640">
<img src="https://github.com/UMich-BipedLab/extrinsic_lidar_camera_calibration/blob/master/figure/test4_3.png" width="640">
<img src="https://github.com/UMich-BipedLab/extrinsic_lidar_camera_calibration/blob/master/figure/test5_3.png" width="640">
<img src="https://github.com/UMich-BipedLab/extrinsic_lidar_camera_calibration/blob/master/figure/test6_3.png" width="640">
<img src="https://github.com/UMich-BipedLab/extrinsic_lidar_camera_calibration/blob/master/figure/test7_3.png" width="640">

# Quantitative results
For the method GL_1-R, five sets of estimated LiDAR vertices for each target have been projected into the image plane and marked in green, while the target's point cloud has been marked in red. Blowing up the image allows the numbers reported in the table to be visualized. The vertices are key.

<img src="https://github.com/UMich-BipedLab/extrinsic_lidar_camera_calibration/blob/master/figure/v1-2.png" width="640">
<img src="https://github.com/UMich-BipedLab/extrinsic_lidar_camera_calibration/blob/master/figure/v2-2.png" width="640">
<img src="https://github.com/UMich-BipedLab/extrinsic_lidar_camera_calibration/blob/master/figure/v3-2.png" width="640">
<img src="https://github.com/UMich-BipedLab/extrinsic_lidar_camera_calibration/blob/master/figure/v4-2.png" width="640">
<img src="https://github.com/UMich-BipedLab/extrinsic_lidar_camera_calibration/blob/master/figure/v5-2.png" width="640">
<img src="https://github.com/UMich-BipedLab/extrinsic_lidar_camera_calibration/blob/master/figure/v6-2.png" width="640">
<img src="https://github.com/UMich-BipedLab/extrinsic_lidar_camera_calibration/blob/master/figure/v7-2.png" width="640">

## Citations (coming strong in a week)
The  Improvements to Target-Based 3D LiDAR to Camera Calibration is described in: 

