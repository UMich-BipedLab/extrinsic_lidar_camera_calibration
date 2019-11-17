# extrinsic_lidar_camera_calibration 
## Overview
This is a package for extrinsic calibration between a 3D LiDAR and a camera, described in paper: **Improvements to Target-Based 3D LiDAR to Camera Calibration** ([PDF](https://arxiv.org/abs/1910.03126)). We evaluated our proposed methods and compared them with other approaches in a round-robin validation study, including qualitative results and quantitative results, where we use image corners as ground truth to evaluate our projection accuracy.

* Authors: Bruce JK Huang and Jessy W. Grizzle
* Maintainer: [Bruce JK Huang](https://www.brucerobot.com/), brucejkh[at]gmail.com
* Affiliation: [The Biped Lab](https://www.biped.solutions/), the University of Michigan

This package has been tested under **MATLAB 2019a** and **Ubuntu 16.04**.

**[Issues]**
If you encounter _any_ issues, I would be happy to help. If you cannot find a related one in the existing issues, please open a new one. I will try my best to help! 

**[Super Super Quick Start]**
Just to see the results, please clone this repo, download the [process/optimized data](https://drive.google.com/drive/folders/1DTyG9pcIvXBqgXUxULWUaBT1zxLYmfz7?usp=sharing) into ALL_LiDAR_vertices and change the [path.load_dir](https://github.com/UMich-BipedLab/extrinsic_lidar_camera_calibration/blob/901a5b4ff4a054b3f19ebb386ef1bfcd4f8c334d/main.m#L49) to ALL_LiDAR_vertices in main.m, and then hit run!

**[Super Quick Start]**
If you would like to see how the LiDAR vertices are optimized, please place the [test datasets](https://github.com/UMich-BipedLab/extrinsic_lidar_camera_calibration/tree/master#dataset) in folders, change the two paths ([path.bag_file_path](https://github.com/UMich-BipedLab/extrinsic_lidar_camera_calibration/blob/901a5b4ff4a054b3f19ebb386ef1bfcd4f8c334d/main.m#L50) and [path.mat_file_path](https://github.com/UMich-BipedLab/extrinsic_lidar_camera_calibration/blob/901a5b4ff4a054b3f19ebb386ef1bfcd4f8c334d/main.m#L51)) in main.m, and then hit run!

**[Developers and Calibrators]**
Please follow more detail instruction as [below](https://github.com/UMich-BipedLab/extrinsic_lidar_camera_calibration/blob/master/README.md#running).


## Abstract
The homogeneous transformation between a LiDAR and monocular camera is required for sensor fusion tasks, such as SLAM. While determining such a transformation is not considered glamorous in any sense of the word, it is nonetheless crucial for many modern autonomous systems. Indeed, an error of a few degrees in rotation or a few percent in translation can lead to 20 cm translation errors at a distance of 5 m when overlaying a LiDAR image on a camera image. The biggest impediments to determining the transformation accurately are the relative sparsity of LiDAR point clouds and systematic errors in their distance measurements. This paper proposes (1) the use of targets of known dimension and geometry to ameliorate target pose estimation in face of the quantization and systematic errors inherent in a LiDAR image of a target, and (2) a fitting method for the LiDAR to monocular camera transformation that fundamentally assumes the camera image data is the most accurate information in one's possession. 

## Quick View
Using the obtained transformation, LiDAR points are mapped onto a semantically segmented image. Each point is associated with the label of a pixel. The road is marked as white; static objects such buildings as orange; the grass as yellow-green, and dark green indicates trees.

<img src="https://github.com/UMich-BipedLab/extrinsic_lidar_camera_calibration/blob/master/figure/semanticImg.png" width="640">
<img src="https://github.com/UMich-BipedLab/extrinsic_lidar_camera_calibration/blob/master/figure/semanticPC3.png" width="640">

# Why important? 
A calibration result is not usable if it has few degrees of rotation error and a few percent of translation error.
The below shows that a calibration result with little disturbance from the well-aigned image.

<img src="https://github.com/UMich-BipedLab/extrinsic_lidar_camera_calibration/blob/master/figure/disturbance.png" width="640"> 
<img src="https://github.com/UMich-BipedLab/extrinsic_lidar_camera_calibration/blob/master/figure/undisturbance.png" width="640">

## Presentation and Video (coming strong in a week)
https://www.brucerobot.com/

## Calibration Targets
Any **square** targets would be fine. The dimensions are assumed known.
note: You can place any number of targets with different size in different datasets.

## Installation 
* Which toolboxes are used in this package: 
  - MATLAB 2019a
  - optimization_toolbox
  - phased_array_system_toolbox
  - robotics_system_toolbox
  - signal_blocks
* Dataset: download from [here](https://github.com/UMich-BipedLab/extrinsic_lidar_camera_calibration/tree/master#dataset).

## Dataset 
Please download optimized LiDAR vertices from [here](https://drive.google.com/drive/folders/1DTyG9pcIvXBqgXUxULWUaBT1zxLYmfz7?usp=sharing) and put them into ALL_LiDAR_vertices folder.

Please download point cloud mat files from [here](https://drive.google.com/drive/folders/1rI3vPvPOJ1ib4i1LMqw66habZEf4SwEr?usp=sharing) and put them into LiDARTag_data folder.

Please download bagfiles from [here](https://drive.google.com/drive/folders/1qawEuUBsC2gQJsHejLEuga2nhERKWRa5?usp=sharing) and put them into bagfiles folder.


## Running
**[Super Super Quick Start]**
Just to see the results, please clone this repo, download the [process/optimized data](https://drive.google.com/drive/folders/1DTyG9pcIvXBqgXUxULWUaBT1zxLYmfz7?usp=sharing) into ALL_LiDAR_vertices and change the [path.load_dir](https://github.com/UMich-BipedLab/extrinsic_lidar_camera_calibration/blob/901a5b4ff4a054b3f19ebb386ef1bfcd4f8c334d/main.m#L49) to ALL_LiDAR_vertices in main.m, and then hit run!

**[Super Quick Start]**
If you would like to see how the LiDAR vertices are optimized, please place the [test datasets](https://github.com/UMich-BipedLab/extrinsic_lidar_camera_calibration/tree/master#dataset) in folders, change the two paths ([path.bag_file_path](https://github.com/UMich-BipedLab/extrinsic_lidar_camera_calibration/blob/901a5b4ff4a054b3f19ebb386ef1bfcd4f8c334d/main.m#L50) and [path.mat_file_path](https://github.com/UMich-BipedLab/extrinsic_lidar_camera_calibration/blob/901a5b4ff4a054b3f19ebb386ef1bfcd4f8c334d/main.m#L51)) in main.m, and then hit run!


**[Calibrators]**
- Please first try the [Super Super Quick Start] section to make sure you can run this code.
- Use _justCalibrate.m_ file
- Find out your camera intrinsic matrix and write them in the _justCalibrate.m_ file. 
- Give initial guess to the LiDAR to camera transformation
- Edit the _trained_ids_ and _skip_indices_ (ids are from _getBagData.m_).
- If you have more validation dataset (containing targets), set the _validation_flag_ to 1 and then use put the related information to _getBagData.m_.
- Place several _square_ boards with known dimensions. When placing boards, make sure the left corner is taller than the right corner.
- Use you favorite methods to extract corners of camera targets and then write them in _getBagData.m_. When writing the corners, Please follow **top-left-right-bottom** order. 
- Given point patches of LiDAR targets, saved them into .mat files and also put them _getBagData.m_. Please make sure you have correctly match your _lidar_target_ with _camera_target_. 
- RUN _justCalibrate.m_! That's it!

note: You can place any number of targets with different size in different datasets.

**[Developers]**
Please download all datasets if you like to play around.

**[Dataset structure]**
Put _ALL information of datasets_ into [getBagData.m](https://github.com/UMich-BipedLab/extrinsic_lidar_camera_calibration/blob/73a614517c7f6077e49368002b8ee563605c9977/getBagData.m#L1). 
This funciton returns two data structure: TestData and BagData.
- TestData contains bagfile and pc_file, where bagfile is the name of the bagfile and pc_file is mat files of _FULL_ scan of point cloud.
- BagData contatins: 
  - bagfile: name of the bagfile
  - num_tag: how many tags in this dataset
  - lidar_target
    - pc_file: the name of the mat file of this target of point cloud
    - tag_size: size of this target
  - camera_target
    - corners: corner coordinates of the camera targets

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

## Citations
The detail is described in: 

Jiunn-Kai Huang and J. Grizzle, "Improvements to Target-Based 3D LiDAR to Camera Calibration" ([PDF](https://github.com/UMich-BipedLab/extrinsic_lidar_camera_calibration/blob/master/LiDAR2CameraCalibration.pdf))([arXiv](https://arxiv.org/abs/1910.03126))
```
@article{huang2019improvements,
  title={Improvements to Target-Based 3D LiDAR to Camera Calibration},
  author={Huang, Jiunn-Kai and Grizzle, Jessy W},
  journal={arXiv preprint arXiv:1910.03126},
  year={2019}
}
```
