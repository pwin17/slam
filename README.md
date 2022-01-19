# Simultaneous Localization and Mapping (SLAM) with graph-based optimization

Input: detected april tag corners in the frames of a video sequence, real tag size and calibration matrix
Task: calculate the *3D locations* of the april tags and the *pose (position and orientation) of the camera* at each frame in the world coordinates
Optimization: Pose Graph Optimization using [GTSAM toolbox](https://www.borg.cc.gatech.edu/download.html "GTSAM toolbox")

Sample output(The rectangle blocks are the tags in world coordinates. The red path indicates the pose of camera at each frame):

<img src="result_images/mapping%20gtsam%20side.jpg" width="425"/> <img src="result_images/mapping%20gtsam%20top.jpg" width="425"/> 

For full report: [project 4 report.pdf](https://github.com/pwin17/slam/blob/master/project%204%20report.pdf "project 4 report.pdf")

To run: 
1. Open the folder in Matlab and `select gtsam_toolbox > Add to Path > Selected Folders`
2. Run Wrapper.m
