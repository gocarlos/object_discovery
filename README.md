Object Discovery in 3D scenes via Shape Analysis
Original authors: Andrej Karpathy, Stephen Miller, Fei-Fei Li

Heavily refactored by Carlos Gomes.
This version of the software "object discovery" was modified be able to interact with ROS and to follow better OO programming style. Tested on Ubuntu 14.04, 16.04, ROS Indigo & Kinetic.

This code can be used to produce results published in the above paper (paper can be found in the doc folder).
The bulk of the code is written in C++ and the evaluation is in Matlab.

Official website of the original authors:
http://cs.stanford.edu/people/karpathy/discovery/

------

**Installation**:

*dependencies*:
* Get catkin_simple (build tool):
https://github.com/ethz-asl/catkin_simple

* Get pcl_catkin (PCL library):
https://github.com/ethz-asl/pcl_catkin

* Optional: Install matlab (parts of visualisation):


*build*:
* catkin build object_discovery

*change ROS parameters*:
* change names of the topics and segmentation paramaters in the config file under [object_discovery_parameters](https://github.com/gocarlos/object_discovery/blob/master/config/object_discovery_parameters.yaml)


------
**Optional**

*scenes dataset*:
Files to test the software:
http://cs.stanford.edu/people/karpathy/discovery/kinfu_scenes.zip

------
**Usage**
 * ROS Usage (provides services, subscribers and publishers)
   * rosrun object_discovery object_discovery_node or
   * roslaunch object_discovery default_segmentation.launch
 * visualisation (See results with PCL Viewer)
   * rosrun object_discovery segment *arg* (ply file which should be segmented).
