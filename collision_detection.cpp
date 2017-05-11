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

#include <fstream>
#include <string>
#include <cmath>
#include <vector>

#include <fcl/config.h>
#include <fcl/geometry/octree/octree.h>
#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include "test_fcl_utility.h"

using namespace fcl;

// Generate Tree 
inline octomap::OcTree* generateOcTree(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud, double resolution)
{
    octomap::OcTree* tree = new octomap::OcTree(resolution);
    for (auto p:cloud->points) tree->updateNode( octomap::point3d(p.x, p.y, p.z), true );

    tree->updateInnerOccupancy();
    tree->writeBinary( "t3.bt");
    return tree;
}

template <typename S>
int octomap_distance_test(double resolution, double box_size[3], double tf[3], pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud )
{
  std::cout << "\nOCTOMAP_DISTANCE_TEST FCN:\n";
  test::TStruct t1;
  test::Timer timer1;
  timer1.start();

  std::vector<CollisionObject<S>*> env;
  Box<double>* box = new Box<double>(box_size[0], box_size[1], box_size[2]);
  fcl::Transform3<S> tf2(fcl::Translation3<S>(fcl::Vector3<S> (tf[0], tf[1], tf[2])));
  env.push_back(new CollisionObject<S>(std::shared_ptr<CollisionGeometry<S>>(box), tf2));
  timer1.stop();

  std::cout << "Box generation: " << timer1.getElapsedTime() << "ms" << std::endl;

  timer1.start();
  OcTree<S>* tree = new OcTree<S>(std::shared_ptr<const octomap::OcTree>(generateOcTree(cloud,resolution)));
  timer1.stop();

  std::cout << "Octree generation: " << timer1.getElapsedTime() << "ms" << std::endl;

  timer1.start();
  CollisionObject<S> tree_obj((std::shared_ptr<CollisionGeometry<S>>(tree)));

  DynamicAABBTreeCollisionManager<S>* manager = new DynamicAABBTreeCollisionManager<S>();
  manager->registerObjects(env);
  manager->setup();

  test::CollisionData<S> cdataCollision;
  test::DistanceData<S> cdata;

  manager->octree_as_geometry_collide = false;
  manager->octree_as_geometry_distance = false;
  manager->distance(&tree_obj, &cdata, test::defaultDistanceFunction);

  manager->collide(&tree_obj, &cdataCollision, test::defaultCollisionFunction);

  timer1.stop();
  std::cout << "collision fcn: " << timer1.getElapsedTime() << "ms" << std::endl;
  delete manager;

  std::cout << "cdata.result.min_distance: " << cdata.result.min_distance << std::endl;
  std::cout << "cdata.result.numContacts: " << cdataCollision.result.numContacts() << std::endl;
  return cdataCollision.result.numContacts();  
}

int main( int argc, char *argv[] )
{
    // Command-Line Argument Parsing
    if( pcl::console::find_switch( argc, argv, "-help" ) || argc < 9 ){
        std::cout << "usage: " << argv[0]
            << "\n-ipaddress # \t\t= Puck's IP "
            << "\n-port # \t\t= Puck's port"
            << "\n-pcap *.pcap \t\t= PCAP recorded file path"
            << "\n-r # \t\t\t= octomap resolution"
            << "\n-box x,y,z \t\t= x,y,z box side values" 
            << "\n-tf x,y,z \t\t= x,y,z box translation"
            << " [-help]"
            << std::endl;
        return 0;
    }

    std::string ipaddress( "192.168.1.70" );
    std::string port( "2368" );
    std::string pcap;

    double resolution = 0.1, voxel_leaf_size=0.1;
    int prev, actual = 0;
    double tf[3] = {0,0,0}, box_size[3] = {0,0,0};

    test::Timer timer;

    pcl::console::parse_argument( argc, argv, "-ipaddress", ipaddress );
    pcl::console::parse_argument( argc, argv, "-port", port );
    pcl::console::parse_argument( argc, argv, "-pcap", pcap );
    pcl::console::parse_argument( argc, argv, "-r", resolution );
    pcl::console::parse_3x_arguments( argc, argv, "-box", box_size[0], box_size[1], box_size[2]);
    pcl::console::parse_3x_arguments( argc, argv, "-tf", tf[0], tf[1], tf[2]);
    pcl::console::parse_argument( argc, argv, "-v", voxel_leaf_size );

    std::cout << "pcap: " <<  pcap << std::endl;
    std::cout << "resolution: " <<  resolution << std::endl;
    for (int i = 0; i < 3; i++)
    std::cout << "b[" << i << "]: " << box_size[i] << std::endl;
    for (int i = 0; i < 3; i++)
    std::cout << "tf[" << i << "]: " << tf[i] << std::endl;

    // Point Cloud
    pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud;
    pcl::PointXYZI p;
    std::vector<double> x,y,z;
    x.push_back(tf[0]);
    y.push_back(tf[1]);
    z.push_back(tf[2]);

    x.push_back(tf[0]);
    y.push_back(tf[1] + box_size[1]);
    z.push_back(tf[2] + box_size[2]);

    x.push_back(tf[0]);
    y.push_back(tf[1]);
    z.push_back(tf[2] + box_size[2]);
    
    x.push_back(tf[0] + box_size[0]);
    y.push_back(tf[1] + box_size[1]);
    z.push_back(tf[2] + box_size[2]);

    x.push_back(tf[0] + box_size[0]);
    y.push_back(tf[1]);
    z.push_back(tf[2] + box_size[2]);

    x.push_back(tf[0] + box_size[0]);
    y.push_back(tf[1] + box_size[1]);
    z.push_back(tf[2]);


    // PCL Visualizer
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer( new pcl::visualization::PCLVisualizer( "Velodyne Viewer" ) );
    viewer->addCoordinateSystem( 3.0, "coordinate" );
    viewer->setBackgroundColor( 0.0, 0.0, 0.0, 0 );
    viewer->initCameraParameters();
    viewer->setCameraPosition( 0.0, 0.0, 30.0, 0.0, 1.0, 0.0, 0 );

    // Point Cloud Color Hndler
    pcl::visualization::PointCloudColorHandler<pcl::PointXYZI>::Ptr handler;
    const std::type_info& type = typeid( pcl::PointXYZI );
    if( type == typeid( pcl::PointXYZ ) ){
        std::vector<double> color = { 255.0, 255.0, 255.0 };
        boost::shared_ptr<pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI>> color_handler( new pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI>( color[0], color[1], color[2] ) );
        handler = color_handler;
    }
    else if( type == typeid( pcl::PointXYZI ) ){
        boost::shared_ptr<pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>> color_handler( new pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI>( "intensity" ) );
        handler = color_handler;
    }
    else if( type == typeid( pcl::PointXYZRGBA ) ){
        boost::shared_ptr<pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZI>> color_handler( new pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZI>() );
        handler = color_handler;
    }
    else{
        throw std::runtime_error( "This PointType is unsupported." );
    }

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

    //Voxel
    pcl::VoxelGrid< pcl::PointXYZI  > sor;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);

    // Register Callback Function
    boost::signals2::connection connection = grabber->registerCallback( function );
    // Start Grabber
    grabber->start();
    while( !viewer->wasStopped() ){
        timer.start();
        std::cout << "\n################################\n";
        // Update Viewer
        viewer->spinOnce();

        boost::mutex::scoped_try_lock lock( mutex );
        if( lock.owns_lock() && cloud ){
            /*
            // Update Point Cloud
            for (size_t i = 0; i < x.size(); ++i)
            {
                p.x = x[i];
                p.y = y[i];
                p.z = z[i];
                p.z = 100;
                cloud->points.push_back(p);
            }*/
            handler->setInputCloud( cloud );
            if( !viewer->updatePointCloud( cloud, *handler, "cloud" ) ){
                viewer->addPointCloud( cloud, *handler, "cloud" );
            }
            sor.setInputCloud (cloud);
            sor.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
            sor.filter(*cloud_filtered);
            std::cout << "Number of points : " << cloud->points.size() << " ===> " << cloud_filtered->points.size() << std::endl;
            
            prev = actual;
            actual = octomap_distance_test<double>( resolution, box_size, tf, cloud_filtered);
            if(prev == 0 && actual)
            {
                usleep(500000);
            }
            timer.stop();
            std::cout << "Looptime: " << timer.getElapsedTime() << "ms";
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