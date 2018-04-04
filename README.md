# PL-SLAM ROS Wrapper #

This code contains a simple ROS wrapper for it's use along with our stereo visual SLAM by using both point and line segment features project:

[https://github.com/rubengooj/pl-slam](https://github.com/rubengooj/pl-slam)

Notice that for representation purposes we are using our MRPT-based visualizer, but it can be used with any other representation class by modifying the project.

### Usage:

./plslam  <params_file>  <config_file>
   <params_file> - file with the camera configuration
   <config_file> - file with the VO configuration (optional)


### PerceptIn Sensor

We also provide a modified version of the following repository: 

[https://github.com/Shuailing-Li/PerceptIn_ROS](https://github.com/Shuailing-Li/PerceptIn_ROS) 

to stream the PerceptIn stereo sensor. The repository can be found in:

[https://github.com/rubengooj/PerceptIn_ROS](https://github.com/rubengooj/PerceptIn_ROS)  















































