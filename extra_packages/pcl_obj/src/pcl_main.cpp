#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <vector>
#include <ctime>
#include <tf/transform_broadcaster.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include "cloud_helpers.cpp"
#include <tf/transform_broadcaster.h>

typedef pcl::PointXYZ PointT;

class cloudHandler
{
public:
    cloudHandler()
    {
        pcl_sub = nh.subscribe("/camera/depth_registered/points", 10, &cloudHandler::cloudCB, this);
        pcl_object_pub = nh.advertise<sensor_msgs::PointCloud2>("object_out", 1);
        //ind_pub = nh.advertise<pcl_msgs::PointIndices>("point_indices", 1);
        //coef_pub = nh.advertise<pcl_msgs::ModelCoefficients>("planar_coef", 1);
    }

    void cloudCB(const sensor_msgs::PointCloud2 &input)
    {


        // objects
        pcl::VoxelGrid<PointT> voxelSampler;
        pcl::StatisticalOutlierRemoval<PointT> statFilter;
        pcl::PassThrough<PointT> pass;
        pcl::NormalEstimation<PointT, pcl::Normal> ne;
        pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
        pcl::ExtractIndices<pcl::Normal> extract_normals;
        pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
        sensor_msgs::PointCloud2 output;
        pcl::PCDWriter writer;

        // datasets
        pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr cloud_f (new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
        pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);
        pcl::PointCloud<PointT>::Ptr cloud_cylinder_big (new pcl::PointCloud<PointT> ());

        //read data to pcl
        pcl::fromROSMsg(input, *cloud);
        //std::cerr << "PointCloud has: " << cloud->points.size () << " data points." << std::endl;

        // Build a passthrough filter to remove points further away than 1.5 meters
        pass.setInputCloud (cloud);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (0, 1.5);
        pass.filter (*cloud_filtered);
       //std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;

        //Downsampling
        voxelSampler.setInputCloud(cloud_filtered);
        voxelSampler.setLeafSize(0.003f, 0.003f, 0.003f);
        voxelSampler.filter(*cloud_filtered);

        //Statistical Outlier removal
        statFilter.setInputCloud(cloud_filtered);
        statFilter.setMeanK(10);
        statFilter.setStddevMulThresh(0.2);
        statFilter.filter(*cloud_filtered);
        // Estimate point normals
        ne.setSearchMethod (tree);
        ne.setInputCloud (cloud_filtered);
        ne.setKSearch (50);//50
        ne.compute (*cloud_normals);

        // Create the segmentation object for the planar model and set all the parameters
        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
        seg.setNormalDistanceWeight (0.1);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setMaxIterations (100);
        seg.setDistanceThreshold (0.03);


        // Segment the planar component from the remaining cloud
        seg.setInputCloud (cloud_filtered);
        seg.setInputNormals (cloud_normals);
        seg.segment (*inliers_plane, *coefficients_plane);
        if (inliers_plane->indices.size () == 0)
        {
          std::cout << "Could not estimate a planar model for the given dataset." << std::endl;

        }
        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud (cloud_filtered);
        extract.setIndices (inliers_plane);
        extract.setNegative (false);

        // Get the points associated with the planar surface
        extract.filter (*cloud_plane);
        std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

        // Remove the planar inliers, extract the rest
        extract.setNegative (true);
        extract.filter (*cloud_f);
        *cloud_filtered = *cloud_f;

        // Creating the KdTree object for the search method of the extraction

        tree->setInputCloud (cloud_filtered);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance (0.005); // 2cm
        ec.setMinClusterSize (100);
        ec.setMaxClusterSize (2000);
        ec.setSearchMethod (tree);
        ec.setInputCloud (cloud_filtered);
        ec.extract (cluster_indices);

        int j = 0;
        int cylinder_size=0;
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
          pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
          for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
          cloud_cluster->width = cloud_cluster->points.size ();
          cloud_cluster->height = 1;
          cloud_cluster->is_dense = true;

          // Estimate point normals
          ne.setSearchMethod (tree);
          ne.setInputCloud (cloud_cluster);
          ne.setKSearch (50);//50
          ne.compute (*cloud_normals2);

          // Create the segmentation object for cylinder segmentation and set all the parameters
          seg.setOptimizeCoefficients (true);
          seg.setModelType(pcl::SACMODEL_NORMAL_SPHERE);
          seg.setModelType (pcl::SACMODEL_CYLINDER);
          seg.setMethodType (pcl::SAC_RANSAC);
          seg.setNormalDistanceWeight (0.1);
          seg.setMaxIterations (10000);
          seg.setDistanceThreshold (0.05);
          seg.setRadiusLimits (0, 0.07);
          seg.setInputCloud (cloud_cluster);
          seg.setInputNormals (cloud_normals2);

          // Obtain the cylinder inliers and coefficients
          seg.segment (*inliers_cylinder, *coefficients_cylinder);
          //std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

          // Write the cylinder inliers to disk
          extract.setInputCloud (cloud_cluster);
          extract.setIndices (inliers_cylinder);
          extract.setNegative (false);
          pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ());
          extract.filter (*cloud_cylinder);
          if (cloud_cylinder->points.empty ())
              std::cerr << "Can't find the cylindrical component." << std::endl;
          else
          {
            if (cloud_cylinder->points.size () > cylinder_size)
            {
              std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size () << " data points." << std::endl;
              std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;
              cylinder_size = cloud_cylinder->size();
              std::cerr << "Cylinder coefficients: " << cylinder_size << std::endl;
              *cloud_cylinder_big = *cloud_cylinder;
            }
            
          }
          j++;
        }
        //calculate average
        pcl::PointXYZ avg;
        avg.x = 0; avg.y = 0; avg.z = 0;
        for(size_t i = 0; i < cloud_cylinder_big->points.size(); i++)
        {
            if(!isnan(cloud_cylinder_big->points[i].x) && !isnan(cloud_cylinder_big->points[i].y) && !isnan(cloud_cylinder_big->points[i].z)) {
                avg.x += cloud_cylinder_big->points[i].x;
                avg.y += cloud_cylinder_big->points[i].y;
                avg.z += cloud_cylinder_big->points[i].z;

            }
        }

        avg.x /= cloud_cylinder_big->points.size();
        avg.y /= cloud_cylinder_big->points.size();
        avg.z /= cloud_cylinder_big->points.size();

        // get header
        std_msgs::Header header = pcl_conversions::fromPCL(cloud_filtered->header);

        // broadcast transform
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(avg.x, avg.y, avg.z) );
        transform.setRotation( tf::Quaternion(0, 0, 0) );
        broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), header.frame_id, "target"));

        pcl::toROSMsg(*cloud_cylinder_big, output);
        output.header.frame_id = "camera_rgb_optical_frame";
        pcl_object_pub.publish(output);







    }
protected:
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub;
    ros::Publisher pcl_object_pub;
    tf::TransformBroadcaster broadcaster;
};

main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_main");

    cloudHandler handler;

    ros::spin();

    return 0;
}
