Collision detection using data from recorded PCAP file (or Velodyne Puck Lite -UNTESTED)

Point Cloud Library ( https://github.com/PointCloudLibrary/pcl ) may enable handling of acquired data e.g. downsampling, filtering, etc

Octomap ( https://github.com/OctoMap/octomap.git ) provides 3D mapping

In this application, Flexible Collision Library ( https://github.com/flexible-collision-library/fcl ) provides collision detection and distance computation

Before compiling this repo, please make sure Eigen, libccd, PCL, Octomap and FCL libraries are installed. 

https://github.com/danfis/libccd
https://github.com/PointCloudLibrary/pcl
https://github.com/OctoMap/octomap.git
https://github.com/flexible-collision-library/fcl

In order to compile this repo, in command line run:

mkdir build
cd build
cmake ..
make

Download a sample PCAP from https://midas3.kitware.com/midas/folder/12979 

Sample executable call:

./collision_detection -pcap ../sample.pcap -r 0.1 -box 1.5,0.5,5 -tf 4.75,0,-0.4

for a pcap file in the main dir, octomap resolution of 0.1, box with sides x=1.5 y=0.5 z=5 and box translated by x=4.75, y=0 and z==-0.4