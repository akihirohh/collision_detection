/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011-2014, Willow Garage, Inc.
 *  Copyright (c) 2014-2016, Open Source Robotics Foundation
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Open Source Robotics Foundation nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * vlp_grabber code from http://unanancyowen.com/en/pcl-with-velodyne/
 * octomap usage from https://github.com/gaoxiang12/octomap_tutor
 * modified from fcl example teste_fcl_octomap_distance
 */
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>

#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <fstream>
#include <string>

int main( int argc, char *argv[] )
{
    pcl::PCDWriter w;
    std::string basename = "lidar";
    unsigned int cnt = 0;
    std::ifstream data_file("cloud.txt");
    double a, b, c;

    pcl::PointCloud<pcl::PointXYZ> cloud;
    while (data_file >> a >> b >> c)
    {
        cloud.push_back (pcl::PointXYZ (a,b,c) );
    }
    w.writeBinaryCompressed(basename + std::to_string(cnt) + ".pcd", cloud);
    std::cout << "Width: " << cloud.width << " Height:" << cloud.height << std::endl;
    cloud.clear(); 
    std::cout << "Width: " << cloud.width << " Height:" << cloud.height << std::endl;

    std::cout << "Use pcl/build/bin/pcl_viewer to visualize PCD image";
    return 0;
}