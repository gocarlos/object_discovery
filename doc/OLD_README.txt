Object Discovery in 3D scenes via Shape Analysis
Andrej Karpathy, Stephen Miller, Fei-Fei Li

This code can be used to produce results published in the above paper.
The bulk of the code is written in C++ and the evaluation is in Matlab.
We tested the on Ubuntu 11.04 running on 8-core 2.67GHz Intel Core i7 CPU with 4GB RAM.

------

Installation:

Install PCL: (if not yet installed)
(instructions http://pointclouds.org/downloads/linux.html)
sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
sudo apt-get update
sudo apt-get install libpcl-all

!!!!
Go into io_utils.h and make sure to set ROOT_PATH to location of the folder (with slash at end)
!!!!

------
Optional: Explore the dataset!

run:
[navigate to main project folder]
$ mkdir build
$ cd build
$ cmake ..
$ make        <- this is where you cross your fingers
$ ./browse 0

that will show the 0'th .ply scene (from /scenes folder). Change the argument to browse for different scene

------
Optional: Segment a single scene and see some intermediate results
$ ./segment 0

------
Run the full pipeline:
(from the build folder)

1. segment all scenes:
$ for i in {0..70}; do ./segment $i 1 0; done
(takes about 2 hours)

2. Next, compute recurrence
$ ./cooccurrence
(takes only a minute or two)

3. annotate the bag with ground truth
$ ./annotate 0   <--- [0 = to start labeling from scratch. (default)]
(OR $ ./annotate 1   <--- [1 = to load existing labels and resume labeling from a previous session])
(brings up the annotation interface. Use 'wsad' to select segments you'd like to label. Use 'x' to toggle the label of a segment (initially all are labeled as Not objects). Hit 'q' when done and you want to save the labels and exit. The annotation interface will sort objects according their average objectness, so most objects should be on top of the list and close together. With our data, we marked about 300 segments as objects during the annotation phase.

We found it helpful while annotating to sometimes reference the original scene that the segment is from. The annotation
interface is open in one window, and when a segment is in doubt, we would use ./browse (id) , where id can be read from console
and inspect more closely if the particular segment is a full object or not.

4. run produceResults.m in Matlab to create and show the PR curves
(takes about 5 minutes as extensive cross-validation is performed)
