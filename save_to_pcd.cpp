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

#include "collision_detection.h"
#include <pcl/io/pcd_io.h>

using namespace fcl;

int main( int argc, char *argv[] )
{

    std::string ipaddress( "192.168.1.70" );
    std::string port( "2368" );
    std::string pcap;
    double resolution = 0.1, voxel_leaf_size=0.1;
    int prev, actual = 0, use_sor = 1, sor_num_neighbors = 10, save_pcd = 0;
    double tf[3] = {0,0,0}, box_size[3] = {0,0,0}, rot[3] = {0,0,0};;

    int cnt_collisions = 0;
    test::Timer timer;

    pcl::PCDWriter w;
    std::string basename = "lidar";
    unsigned int cnt = 0;

    // Command-Line Argument Parsing
    if( pcl::console::find_switch( argc, argv, "-help" ) || argc < 9 ){
        cd::printHelp(argv);
        return 0;
    }

    pcl::console::parse_argument( argc, argv, "-ipaddress", ipaddress );
    pcl::console::parse_argument( argc, argv, "-port", port );
    pcl::console::parse_argument( argc, argv, "-pcap", pcap );
    pcl::console::parse_argument( argc, argv, "-r", resolution );
    pcl::console::parse_3x_arguments( argc, argv, "-box", box_size[0], box_size[1], box_size[2]);
    pcl::console::parse_3x_arguments( argc, argv, "-tf", tf[0], tf[1], tf[2]);
    pcl::console::parse_3x_arguments( argc, argv, "-rot", rot[0], rot[1], rot[2]);
    pcl::console::parse_argument( argc, argv, "-v", voxel_leaf_size );
    pcl::console::parse_argument( argc, argv, "-s", sor_num_neighbors);
    pcl::console::parse_argument( argc, argv, "-sor", use_sor);
    pcl::console::parse_argument( argc, argv, "-pcd", save_pcd);

    for (int i= 0; i < 3; i++) rot[i] *= M_PI/180;

    // Point Cloud
    pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud;
    // Collision Checking Box(es)
    std::vector<CollisionObject<double>*> boxes;
    cd::createBox(boxes, box_size, tf, rot);

   // Point Cloud to plot collision checking box 
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_box = cd::createPointCloudFromBox(boxes, 0, box_size, 10);
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> box_color_handler(cloud_box, "z");

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
            timer.start(); 
            if(prev == 1 && actual)
            {
                cnt_collisions++;
                usleep(1000000);
                actual = 0;
            }    
            prev = actual;
            // Check collisions between box and map
            actual = cd::octomap_distance_test( resolution, voxel_leaf_size, boxes, cloud, use_sor, sor_num_neighbors);
            timer.stop();
            if(save_pcd)
            {
                cnt++;  
                timer.start();   
                w.writeBinaryCompressed(basename + std::to_string(cnt) + ".pcd", *cloud);
                timer.stop(); 
                std::cout << "Write PCD: " << timer.getElapsedTime() << "ms" << std::endl;
            }
            std::cout << "Looptime: " << timer.getElapsedTime() << "ms cnt_collisions: " << cnt_collisions;
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