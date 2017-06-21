Collision detection using data from recorded PCAP file (or Velodyne Puck Lite -UNTESTED)

Point Cloud Library ( https://github.com/PointCloudLibrary/pcl ) enables handling acquired data e.g. downsampling, filtering, etc

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

./collision_detection_viewer -pcap ../sample.pcap -r 0.4 -box 2.5,1.5,3.2 -tf 3.25,-2,-0.5 -v 0.4 -sor 1 -s 10 -pcd 0

for a pcap file in the main dir, octomap resolution of 0.4, box with sides x=2.5 y=1.5 z=3.2 and box translated by x=3.25, y=-2 and z=-0.5, voxel leaf size of 0.4m, using Statistical Outlier Remover (SOR) with 10 neighbors and not savind pcd files