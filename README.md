# Simultaneous Localization and Mapping (SLAM) with graph-based optimization

Input: detected april tag corners in the frames of a video sequence, real tag size and calibration matrix
Task: calculate the *3D locations* of the april tags and the *pose (position and orientation) of the camera* at each frame in the world coordinates
Optimization: Pose Graph Optimization using [GTSAM toolbox][1]
[1]: https://www.borg.cc.gatech.edu/download.html

Sample output:
The rectangle blocks are the tags in world coordinates. The red path indicates the pose of camera at each frame.
![Alt][https://drive.google.com/file/d/1IqSBtnICOz3gmZHJAi8rRNtnbdZJj4_G/view?usp=sharing]

For full report: [project 4 report.pdf][link]

To run: 
1. Open the folder in Matlab and `select gtsam_toolbox > Add to Path > Selected Folders`
2. Run Wrapper.m