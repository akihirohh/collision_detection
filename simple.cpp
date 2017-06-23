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
#include <pcl/io/vlp_grabber.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/Dense>

#include <fstream>
#include <string>
#include <cmath>
#include <vector>


int main( int argc, char *argv[] )
{
    std::string ipaddress( "192.168.1.70" );
    std::string port( "2368" );
    std::string pcap;
    int save_pcd = 0;

    int cnt_collisions = 0;
    pcl::PCDWriter w;
    std::string basename = "lidar";
    unsigned int cnt = 0;

    // Command-Line Argument Parsing
    if( pcl::console::find_switch( argc, argv, "-help" ) || argc < 3 ){
        std::cout << "usage: " << argv[0]
            << "\n-ipaddress # \t\t= Puck's IP "
            << "\n-port # \t\t= Puck's port"
            << "\n-pcap *.pcap \t\t= PCAP recorded file path"
            << "\n-pcd # \t\t\t= save PCD files (1) or not (0)"
            << " [-help]"
            << std::endl;
    }

    pcl::console::parse_argument( argc, argv, "-ipaddress", ipaddress );
    pcl::console::parse_argument( argc, argv, "-port", port );
    pcl::console::parse_argument( argc, argv, "-pcap", pcap );
    pcl::console::parse_argument( argc, argv, "-pcd", save_pcd);

    // Point Cloud
    pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud;

    // Retrieved Point Cloud Callback Function
    boost::mutex mutex;
    boost::function<void( const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& )> function =
        [ &cloud, &mutex ]( const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& ptr ){
            boost::mutex::scoped_lock lock( mutex );
            cloud = ptr;
        };

    // VLP Grabber
    boost::shared_ptr<pcl::VLPGrabber> grabber;
    if( !pcap.empty() ){
        std::cout << "Capture from PCAP..." << std::endl;
        grabber = boost::shared_ptr<pcl::VLPGrabber>( new pcl::VLPGrabber( pcap ) );
    }
    else if( !ipaddress.empty() && !port.empty() ){
        std::cout << "Capture from Sensor..." << std::endl;
        grabber = boost::shared_ptr<pcl::VLPGrabber>( new pcl::VLPGrabber( boost::asio::ip::address::from_string( ipaddress ), boost::lexical_cast<unsigned short>( port ) ) );
    }

    // Register Callback Function
    boost::signals2::connection connection = grabber->registerCallback( function );

    // Start Grabber
    grabber->start();

    while( true ){
        std::cout << "\n################################\n";
        boost::mutex::scoped_try_lock lock( mutex );
        if( lock.owns_lock() && cloud )
        {            
            if(save_pcd)
            {
                cnt++;   
                w.writeBinaryCompressed(basename + std::to_string(cnt) + ".pcd", *cloud);
            }
        }
    }

    // Stop Grabber
    grabber->stop();

    // Disconnect Callback Function
    if( connection.connected() ){
        connection.disconnect();
    }
    return 0;
}