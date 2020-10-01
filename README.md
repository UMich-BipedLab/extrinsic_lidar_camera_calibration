# extrinsic_lidar_camera_calibration 
**[Release Note July 2020]**
This work has been accepted by [IEEE Access](https://ieeexplore.ieee.org/document/9145571) and has been uploaded to [arXiv](https://arxiv.org/pdf/1910.03126v3.pdf).

**[Release Note March 2020]**
This is the new master branch from March 2020. The current master branch supports a revised version of the arXiv paper, namely [paper](https://arxiv.org/pdf/1910.03126v2.pdf). The original master branch from Oct 2019 to March 2020 is now moved to v1-2019 branch, and it supports the functions associated with the first version of the Extrinsic Calibration paper that we placed on the arXiv, namely [paper](https://arxiv.org/pdf/1910.03126v1.pdf). Please be aware that there are functions in the older branch that have been removed from the current master branch. 
 

## Overview
This is a package for extrinsic calibration between a 3D LiDAR and a camera, described in paper: **Improvements to Target-Based 3D LiDAR to Camera Calibration** ([PDF](https://arxiv.org/abs/1910.03126)). We evaluated our proposed methods and compared them with other approaches in a round-robin validation study, including qualitative results and quantitative results, where we use image corners as ground truth to evaluate our projection accuracy.

* Authors: Bruce JK Huang and Jessy W. Grizzle
* Maintainer: [Bruce JK Huang](https://www.brucerobot.com/), brucejkh[at]gmail.com
* Affiliation: [The Biped Lab](https://www.biped.solutions/), the University of Michigan

This package has been tested under **MATLAB 2019a** and **Ubuntu 16.04**.

**[Issues]**
If you encounter _any_ issues, I would be happy to help. If you cannot find a related one in the existing issues, please open a new one. I will try my best to help! 

**[Super Super Quick Start]**
Just to see the results, please clone this repo, download the [process/optimized data](https://drive.google.com/drive/folders/1DTyG9pcIvXBqgXUxULWUaBT1zxLYmfz7?usp=sharing) into [load_all_vertices folder](https://github.com/UMich-BipedLab/extrinsic_lidar_camera_calibration/blob/15e2d941053c6d68a18746d4a28c92f7717756d1/justCalibrate.m#L63) and change the [path.load_dir](https://github.com/UMich-BipedLab/extrinsic_lidar_camera_calibration/blob/15e2d941053c6d68a18746d4a28c92f7717756d1/justCalibrate.m#L62) to load_all_vertices folder in justCalibrate.m, and then hit run!

**[Super Quick Start]**
If you would like to see how the LiDAR vertices are optimized, please place the [test datasets](https://github.com/UMich-BipedLab/extrinsic_lidar_camera_calibration/tree/master#dataset) in folders, change the two paths ([path.bag_file_path](https://github.com/UMich-BipedLab/extrinsic_lidar_camera_calibration/blob/15e2d941053c6d68a18746d4a28c92f7717756d1/justCalibrate.m#L64) and [path.mat_file_path](https://github.com/UMich-BipedLab/extrinsic_lidar_camera_calibration/blob/15e2d941053c6d68a18746d4a28c92f7717756d1/justCalibrate.m#L65)) in justCalibrate.m, and then hit run!

**[Developers and Calibrators]**
Please follow more detail instruction as [below](https://github.com/UMich-BipedLab/extrinsic_lidar_camera_calibration/blob/master/README.md#running).


## Abstract
The rigid-body transformation between a LiDAR and monocular camera is required for sensor fusion tasks, such
as SLAM. While determining such a transformation is not considered glamorous in any sense of the word, it is nonetheless crucial for many modern autonomous systems. Indeed, an error of a few degrees in rotation or a few percent in translation can lead to 20 cm reprojection errors at a distance of 5 m when overlaying a LiDAR image on a camera image. The biggest impediments to determining the transformation accurately are the relative sparsity of LiDAR point clouds and systematic errors in their distance measurements. This paper proposes (1) the use of targets of known dimension and geometry to ameliorate target pose estimation in face of the quantization and systematic errors inherent in a LiDAR image of a target, (2) a fitting method for the LiDAR to monocular camera transformation that avoids the tedious task of target edge extraction from the point could, and (3) a “cross-validation study” based on projection of the 3D LiDAR target vertices to the corresponding corners in the camera image. The end result is a **50%** reduction in projection error and a **70%** reduction in its variance.

## Performance
This is a short summary from the paper; see [PDF](https://arxiv.org/abs/1910.03126) for more detail.
This table compares mean and standard deviation for [baseline](https://www.cs.cmu.edu/~kaess/pub/Zhou18iros.pdf) and our approach as a function of
the number of targets used in training. Units are pixel per corner.
|      |# Tag|    2   |    4   |    6   |   8   |
|:----:|:---:| :-----:| :-----:| :-----:|:-----:|
| Baseline (previous state-of-the-art)  | mean| 10.3773| 4.9645 | 4.3789 | 3.9940|
| Proposed method - PnP | mean| **3.8523** | **1.8939** | **1.6817** | **1.7547**|
| Proposed method - IoU | mean| 4.9019 | 2.2442 | 1.7631 | 1.7837 |
| Baseline (previous state-of-the-art)  | std | 7.0887 | 1.9532 | 1.7771 | 2.0467|
| Proposed method  - PnP | std | **2.4155** | **0.5609** | 0.5516 | 0.5419|
| Proposed method - IoU | std | 2.5060| 0.7162 | **0.5070** | **0.4566**|

## Application Videos 
The 3D-LiDAR map shown in the videos used this package to calibrate the LiDAR to camera (to get the transformatoin between the LiDAR and camera). Briefly speaking, we project point coulds from the LiDAR back to the semantic labeled images using the obtained transformation and then associate labels with the point to build the 3D LiDAR semantic map.

[Halloween Edition: Cassie Autonomy](https://www.youtube.com/watch?v=4OUr2DspYoo) 

[Autonomous Navigation and 3D Semantic Mapping on Bipedal Robot Cassie Blue (Shorter Version)](https://www.youtube.com/watch?v=uFyT8zCg1Kk)

[Autonomous Navigation and 3D Semantic Mapping on Bipedal Robot Cassie Blue (Longer Version)](https://youtu.be/N8THn5YGxPw)


[<img src="https://github.com/UMich-BipedLab/extrinsic_lidar_camera_calibration/blob/master/figure/Halloween.png" width="640">](https://www.youtube.com/watch?v=4OUr2DspYoo)
[<img src="https://github.com/UMich-BipedLab/extrinsic_lidar_camera_calibration/blob/master/figure/3D-LiDAR-Semantic-maps.png" width="640">](https://www.youtube.com/watch?v=uFyT8zCg1Kk)
[<img src="https://github.com/UMich-BipedLab/extrinsic_lidar_camera_calibration/blob/master/figure/3D-LiDAR-Semantic-maps2.png" width="640">](https://www.youtube.com/watch?v=uFyT8zCg1Kk)

## Quick View
Using the obtained transformation, LiDAR points are mapped onto a semantically segmented image. Each point is associated with the label of a pixel. The road is marked as white; static objects such buildings as orange; the grass as yellow-green, and dark green indicates trees.

<img src="https://github.com/UMich-BipedLab/extrinsic_lidar_camera_calibration/blob/master/figure/semanticImg.png" width="640">
<img src="https://github.com/UMich-BipedLab/extrinsic_lidar_camera_calibration/blob/master/figure/semanticPC3.png" width="640">

# Why important? 
A calibration result is not usable if it has few degrees of rotation error and a few percent of translation error.
The below shows that a calibration result with little disturbance from the well-aigned image.

<img src="https://github.com/UMich-BipedLab/extrinsic_lidar_camera_calibration/blob/master/figure/disturbance.png" width="640"> 
<img src="https://github.com/UMich-BipedLab/extrinsic_lidar_camera_calibration/blob/master/figure/undisturbance.png" width="640">

## Presentation and Video
https://www.brucerobot.com/calibration

## Calibration Targets
Any **square** targets would be fine. The dimensions are assumed known. We use fiducial tags that can be detected both from LiDARs and cameras. Physically, they are the same tags. However, if the tag is detected from LiDARs, we call it [LiDARTag](https://arxiv.org/abs/1908.10349) and on the other hand, if is is detected from cameras, it is called AprilTag. Please check out this [link](https://drive.google.com/open?id=1Twx7y6yxr-s2qAoCa4XliCxNuINAlXNn) to download the target images. If you use these targets as you LiDAR targets, please cite 
```
@article{huang2019lidartag,
  title={LiDARTag: A Real-Time Fiducial Tag using Point Clouds},
  author={Huang, Jiunn-Kai and Ghaffari, Maani and Hartley, Ross and Gan, Lu and Eustice, Ryan M and Grizzle, Jessy W},
  journal={arXiv preprint arXiv:1908.10349},
  year={2019}
}
```
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
Please download optimized LiDAR vertices from [here] (these are precalculated vertices and are used if only extrinsics optimization should be performed) (https://drive.google.com/drive/folders/1DTyG9pcIvXBqgXUxULWUaBT1zxLYmfz7?usp=sharing) and put them into ALL_LiDAR_vertices folder.

Please download point cloud mat files from [here] (full lidar pointclouds converted to mat format and extracted pointclouds for each target (LIDARTag)) (https://drive.google.com/drive/folders/1rI3vPvPOJ1ib4i1LMqw66habZEf4SwEr?usp=sharing) and put them into LiDARTag_data folder.

Please download bagfiles from [here] (https://drive.google.com/drive/folders/1qawEuUBsC2gQJsHejLEuga2nhERKWRa5?usp=sharing) and put them into bagfiles folder.


## Running
**[Super Super Quick Start]**
Just to see the results, please clone this repo, download the [process/optimized data](https://drive.google.com/drive/folders/1DTyG9pcIvXBqgXUxULWUaBT1zxLYmfz7?usp=sharing) into [load_all_vertices folder](https://github.com/UMich-BipedLab/extrinsic_lidar_camera_calibration/blob/15e2d941053c6d68a18746d4a28c92f7717756d1/justCalibrate.m#L63) and change the [path.load_dir](https://github.com/UMich-BipedLab/extrinsic_lidar_camera_calibration/blob/15e2d941053c6d68a18746d4a28c92f7717756d1/justCalibrate.m#L62) to load_all_vertices folder in justCalibrate.m, and then hit run!

**[Super Quick Start]**
If you would like to see how the LiDAR vertices are optimized, please place the [test datasets](https://github.com/UMich-BipedLab/extrinsic_lidar_camera_calibration/tree/master#dataset) in folders, change the two paths ([path.bag_file_path](https://github.com/UMich-BipedLab/extrinsic_lidar_camera_calibration/blob/15e2d941053c6d68a18746d4a28c92f7717756d1/justCalibrate.m#L64) and [path.mat_file_path](https://github.com/UMich-BipedLab/extrinsic_lidar_camera_calibration/blob/15e2d941053c6d68a18746d4a28c92f7717756d1/justCalibrate.m#L65)) in justCalibrate.m, and then hit run!


**[Calibrators]**
- Please first try the [Super Super Quick Start] section to ensure you can run this code.
- Use _justCalibrate.m_ file
- Find out your camera intrinsic matrix and write them in the _justCalibrate.m_ file. 
- Give initial guess to the LiDAR to camera transformation
- Edit the _trained_ids_ and _skip_indices_ (ids are from _getBagData.m_).
- If you have more validation dataset (containing targets), set the _validation_flag_ to 1 and then use put the related information to _getBagData.m_.
- Place several _square_ boards with known dimensions. When placing boards, make sure the left corner is taller than the right corner. We use fiducial tags that can be detected both from LiDARs and cameras. Physically, they are the same tags. However, if the tag is detected from LiDARs, we call it [LiDARTag](https://arxiv.org/abs/1908.10349) and on the other hand, if is is detected from cameras, it is called AprilTag. Please check out this [link](https://drive.google.com/open?id=1Twx7y6yxr-s2qAoCa4XliCxNuINAlXNn) to download the target images. If you use these targets as you LiDAR targets, please cite 
```
@article{huang2019lidartag,
  title={LiDARTag: A Real-Time Fiducial Tag using Point Clouds},
  author={Huang, Jiunn-Kai and Ghaffari, Maani and Hartley, Ross and Gan, Lu and Eustice, Ryan M and Grizzle, Jessy W},
  journal={arXiv preprint arXiv:1908.10349},
  year={2019}
}
```
- Use you favorite methods to extract corners of camera targets and then write them in _getBagData.m_. When writing the corners, Please follow **top-left-right-bottom** order. 
- Given point patches of LiDAR targets, saved them into .mat files and also put them _getBagData.m_. Please make sure you have correctly match your _lidar_target_ with _camera_target_. 
- If you have trouble extracting patches of LiDAR targets, or converting bagfiles to mat-files, I have also provided another python script to conver a bagfile to a mat-file and extract patches. Please check out bag2mat.py.
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

**[Additional Tools for Data preparation]**
- [bag2mat.py] There is the script for converting point cloud data to mat format and extract the LIDAR tags (i.e. the subset of points in the pointcloud that correspond to one of the targets in the scene).
  - usage: '$ python bag2mat.py <bagfile_path> <event_name>' (event name will be part of the output file name)
  - if parameter 'extracted' is 0 then the whole input pointcloud is converted to mat
  - if extracted == 1 then cuboids can be defined in function 'readPointsandSave' to extract points that belong to one of the tags
  - Make sure to set appropriate axis limits for the visualalization

- [bag2opencvCorners.py] There is a script for extracting the camera_target, i.e. the corner coordinates of the camera targets. 
  - usage '$ python bag2opencvCorners.py <bagfile_path> <undistorted_image_topic_name>'
  - It will open the first image of the bagfile (assuming the sensor will not be moved) with opencv.
  - Hovering the mouse cursor on top of the image will display its coordinates in pixel values (for entering them in getBagData).
  - Note that these are initial guesses and will be optimized (with default settings using canny edge detection and ransac).
  - Make sure to use the undistorted camera images, there is no undistortion performed.

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
@article{huang2020improvements,
  author={J. {Huang} and J. W. {Grizzle}},
  journal={IEEE Access}, 
  title={Improvements to Target-Based 3D LiDAR to Camera Calibration}, 
  year={2020},
  volume={8},
  number={},
  pages={134101-134110},}
```
If you use [LiDARTag](https://arxiv.org/abs/1908.10349) as you LiDAR targets, please cite 
```
@article{huang2019lidartag,
  title={LiDARTag: A Real-Time Fiducial Tag using Point Clouds},
  author={Huang, Jiunn-Kai and Ghaffari, Maani and Hartley, Ross and Gan, Lu and Eustice, Ryan M and Grizzle, Jessy W},
  journal={arXiv preprint arXiv:1908.10349},
  year={2019}
}
```
