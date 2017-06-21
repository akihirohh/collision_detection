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
 * modified from fcl example a_fcl_octomap_distance
 */

#include "collision_detection.h"

using namespace fcl;

namespace cd
{
    void createBox(std::vector<CollisionObject<double>*> &env, double box_size[3], double tf[3], double rot[3])
    {
        Box<double>* box = new Box<double>(box_size[0], box_size[1], box_size[2]);
        Matrix3<double> rotation(
                AngleAxis<double>(rot[0], Vector3<double>::UnitX())
            * AngleAxis<double>(rot[1], Vector3<double>::UnitY())
            * AngleAxis<double>(rot[2], Vector3<double>::UnitZ()));
        fcl::Transform3<double> tf2 = fcl::Transform3<double>::Identity();
        tf2.translation() = fcl::Vector3<double>(tf[0], tf[1], tf[2]);
        tf2.linear() = rotation;

        Box<double> box2(box_size[0], box_size[1], box_size[2]);
        BVHModel<OBBRSS<double>>* model = new BVHModel<OBBRSS<double>>();
        fcl::generateBVHModel(*model, box2,  Transform3<double>::Identity());

        //env.push_back(new CollisionObject<double>(std::shared_ptr<CollisionGeometry<double>>(box), tf2));
        env.push_back(new CollisionObject<double>(std::shared_ptr<CollisionGeometry<double>>(model), tf2));
    }

    // Create PointCloud from user input box
    pcl::PointCloud<pcl::PointXYZ>::Ptr createPointCloudFromBox(std::vector<CollisionObject<double>*> env, int n_box, double box_size[3], int pnts_side)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_box (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointXYZ p;
        Vector3<double> translation = env[n_box]->getTranslation();
        Matrix3<double> rotation = env[n_box]->getRotation();
        int total_length = 12*pnts_side - 4;
        Eigen::MatrixXd translated(3,total_length);
        Eigen::MatrixXd rotated(3,total_length);
        Eigen::MatrixXd a(3,total_length);
        int k = -1;
        for (double i = - box_size[0]/2; i <= box_size[0]/2; i+= box_size[0]/pnts_side)
        {
            a.col(++k) << i, -box_size[1]/2, -box_size[2]/2;
            a.col(++k) << i,  box_size[1]/2, -box_size[2]/2;
            a.col(++k) << i, -box_size[1]/2,  box_size[2]/2;
            a.col(++k) << i,  box_size[1]/2,  box_size[2]/2;
           
        }       
        for (double i = - box_size[1]/2 + box_size[1]/pnts_side; i < box_size[1]/2; i += box_size[1]/pnts_side)
        {
            a.col(++k) << -box_size[0]/2, i, -box_size[2]/2;
            a.col(++k) <<  box_size[0]/2, i, -box_size[2]/2;
            a.col(++k) << -box_size[0]/2, i,  box_size[2]/2;
            a.col(++k) <<  box_size[0]/2, i,  box_size[2]/2;
        }
        for (double i = - box_size[2]/2 + box_size[2]/pnts_side; i < box_size[2]/2; i += box_size[2]/pnts_side)
        {
            a.col(++k) << -box_size[0]/2, -box_size[1]/2, i;
            a.col(++k) <<  box_size[0]/2, -box_size[1]/2, i;
            a.col(++k) << -box_size[0]/2,  box_size[1]/2, i;
            a.col(++k) <<  box_size[0]/2,  box_size[1]/2, i;
        }

        rotated = rotation * a;
        for (int i = 0; i < total_length; i++)
        {
            translated(0,i) = rotated(0,i) + translation[0]; 
            translated(1,i) = rotated(1,i) + translation[1]; 
            translated(2,i) = rotated(2,i) + translation[2]; 
        }
        // Update Point Cloud
        for (size_t i = 0; i < total_length; ++i)
        {
            p.x = translated(0,i);
            p.y = translated(1,i);
            p.z = translated(2,i);
            cloud_box->points.push_back(p);
        }
        return cloud_box;
    }

    // Generate Tree 
    octomap::OcTree* generateOcTree(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud, double resolution)
    {
        octomap::OcTree* tree = new octomap::OcTree(resolution);
        for (auto p:cloud->points) tree->updateNode( octomap::point3d(p.x, p.y, p.z), true );
        tree->updateInnerOccupancy();
        return tree;
    }

    void printHelp(char *argv[])
    {
        std::cout << "usage: " << argv[0]
            << "\n-ipaddress # \t\t= Puck's IP "
            << "\n-port # \t\t= Puck's port"
            << "\n-pcap *.pcap \t\t= PCAP recorded file path"
            << "\n-r # \t\t\t= octomap resolution"
            << "\n-box x,y,z \t\t= x,y,z box side values" 
            << "\n-tf x,y,z \t\t= x,y,z box translation"
            << "\n-s # \t\t\t= number of neighbors for StatisticalOutlierRemoval"
            << "\n-sor # \t\t\t= use SOR (1) or not (0)"
            << "\n-pcd # \t\t\t= save PCD files (1) or not (0)"
            << " [-help]"
            << std::endl;
    }

    int octomap_distance_test(double resolution, double voxel_leaf_size, std::vector<CollisionObject<double>*> env , pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud, int use_sor, int sor_number_neighbors )
    {
    std::cout << "\nOCTOMAP_DISTANCE_TEST FCN:\n";
    test::TStruct t1;
    test::Timer timer1;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);
    timer1.start();
    //Voxel
    pcl::VoxelGrid< pcl::PointXYZI  > vg;
    vg.setInputCloud (cloud);
    vg.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
    vg.filter(*cloud_filtered);
    std::cout << "Number of points : " << cloud->points.size() << " ===> " << cloud_filtered->points.size() << std::endl;
    timer1.stop();    
    std::cout << "Voxel: " << timer1.getElapsedTime() << "ms" << std::endl;

    if (use_sor)
    {
        timer1.start();
        pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
        sor.setInputCloud (cloud_filtered);
        sor.setMeanK (sor_number_neighbors);
        sor.setStddevMulThresh (1.0);
        sor.filter (*cloud_filtered);
        std::cout << "Number of points after SOR: " << cloud_filtered->points.size() << std::endl;
        timer1.stop();
        std::cout << "StatisticalOutlierRemoval: " << timer1.getElapsedTime() << "ms" << std::endl;
    }

    timer1.start();
    OcTree<double>* tree = new OcTree<double>(std::shared_ptr<const octomap::OcTree>(generateOcTree(cloud_filtered,resolution)));
    timer1.stop();
    std::cout << "Octree generation: " << timer1.getElapsedTime() << "ms" << std::endl;

    timer1.start();
    CollisionObject<double> tree_obj((std::shared_ptr<CollisionGeometry<double>>(tree)));
    DynamicAABBTreeCollisionManager<double>* manager = new DynamicAABBTreeCollisionManager<double>();
    manager->registerObjects(env);
    manager->setup();

    test::CollisionData<double> cdataCollision;
    test::DistanceData<double> cdata;

    manager->octree_as_geometry_collide = true;
    manager->octree_as_geometry_distance = true;
    //manager->distance(&tree_obj, &cdata, test::defaultDistanceFunction);

    manager->collide(&tree_obj, &cdataCollision, test::defaultCollisionFunction);

    timer1.stop();
    std::cout << "collision fcn: " << timer1.getElapsedTime() << "ms" << std::endl;
    delete manager;

    //std::cout << "cdata.result.min_distance: " << cdata.result.min_distance << std::endl;
    std::cout << "cdata.result.numContacts: " << cdataCollision.result.numContacts() << std::endl;
    return cdataCollision.result.numContacts();  
    }

    void saveOcTree(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud2, double resolution, int n_file)
    {
        octomap::OcTree* tree = new octomap::OcTree(resolution);
        for (auto p:cloud->points) tree->updateNode( octomap::point3d(p.x, p.y, p.z), true );
        tree->updateInnerOccupancy();
        for (auto p:cloud2->points) tree->updateNode( octomap::point3d(p.x, p.y, p.z), true );
        tree->updateInnerOccupancy();
        std::string filename = 't' + std::to_string(n_file) + ".bt";
        tree->writeBinary( filename.c_str() );
        std::cout << "\noctree saved in file: " << filename << std::endl;;
    }
}

