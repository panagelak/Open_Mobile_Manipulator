
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
// PCL specific includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <pcl/PCLPointCloud2.h>
#include <pcl/point_types_conversion.h>
#include <pcl/filters/filter.h>
#include <pcl/PointIndices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>

namespace cloud_helpers {

    pcl::PointXYZHSV
    getCloudAverage(pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud)
    {
        // std::vector<int> indices;
        // pcl::removeNaNFromPointCloud(*cloud_filtered, *cloud_filtered, indices);

        pcl::PointXYZHSV avg;
        avg.x = 0; avg.y = 0; avg.z = 0; avg.h = 0; avg.s = 0; avg.v = 0;

        for(size_t i = 0; i < cloud->points.size(); i++)
        {
            if(!isnan(cloud->points[i].x) && !isnan(cloud->points[i].y) && !isnan(cloud->points[i].z)) {
                avg.x += cloud->points[i].x;
                avg.y += cloud->points[i].y;
                avg.z += cloud->points[i].z;
                avg.h += cloud->points[i].h;
                avg.s += cloud->points[i].s;
                avg.v += cloud->points[i].v;
            }
        }

        avg.x /= cloud->points.size();
        avg.y /= cloud->points.size();
        avg.z /= cloud->points.size();
        avg.h /= cloud->points.size();
        avg.s /= cloud->points.size();
        avg.v /= cloud->points.size();

        return avg;
    }

    //template <typename PointT>
    std::vector<pcl::PointIndices::Ptr> 
    segmentByDistance(pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud)
    {
        // remove NaNs
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

        // Creating the KdTree object and perform Euclidean distance search
        pcl::search::KdTree<pcl::PointXYZHSV>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZHSV>);
        tree->setInputCloud (cloud);

        // perform extraction
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZHSV> ec;
        ec.setClusterTolerance (0.03); // 3cm
        ec.setMinClusterSize (400);
        ec.setMaxClusterSize (25000);//(25000); // for reference, Kinect returns 307200 points
        ec.setSearchMethod (tree);
        ec.setInputCloud (cloud);
        ec.extract (cluster_indices);

        // convert indicies to pointers
        std::vector<pcl::PointIndices::Ptr> cluster_indice_ptrs;
        cluster_indice_ptrs.resize(cluster_indices.size());
        for(size_t i = 0; i < cluster_indices.size(); i++) {
            pcl::PointIndices::Ptr cluster_indices_ptr (new pcl::PointIndices(cluster_indices[i]));
            cluster_indice_ptrs[i] = cluster_indices_ptr;
        }

        return cluster_indice_ptrs;
    }

    std::vector<pcl::PointCloud<pcl::PointXYZHSV>::Ptr>
    extractClusters(pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud, std::vector<pcl::PointIndices::Ptr> cluster_indices)
    {
        // set up extractor
        pcl::ExtractIndices<pcl::PointXYZHSV> extract;
        extract.setInputCloud(cloud);
        extract.setNegative(false);

        // set up cloud vector
        std::vector<pcl::PointCloud<pcl::PointXYZHSV>::Ptr> clusters;
        clusters.resize(cluster_indices.size());

        // extract clusters
        for(size_t i = 0; i < cluster_indices.size(); i++) {
            pcl::PointCloud<pcl::PointXYZHSV>::Ptr cluster (new pcl::PointCloud<pcl::PointXYZHSV>);
            extract.setIndices(cluster_indices[i]);
            extract.filter(*cluster);
            clusters[i] = cluster;
        }
        // for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
        // {
        //     pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZHSV>);
        //     for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
        //         cloud_cluster->points.push_back (cloud->points[*pit]); //
        //     cloud_cluster->width = cloud_cluster->points.size ();
        //     cloud_cluster->height = 1;
        //     cloud_cluster->is_dense = false;
        //     cloud_cluster->header = cloud->header;

        //     clusters.push_back(cloud_cluster);
        // }

        return clusters;
    }

    // from https://github.com/ros-interactive-manipulation/pr2_object_manipulation/blob/groovy-devel/perception/tabletop_object_detector/include/tabletop_object_detector/marker_generator.h
    template <class PointCloudTypePtr>
    visualization_msgs::Marker
    static getCloudMarker(const PointCloudTypePtr& cloud)
    {
        static bool first_time = true;
        if(first_time) {
            srand ( time(NULL) );
            first_time = false;
        }

        // create the marker
        visualization_msgs::Marker marker;
        marker.action = visualization_msgs::Marker::ADD;
        marker.lifetime = ros::Duration();

        marker.type = visualization_msgs::Marker::POINTS;
        marker.scale.x = 0.01;
        marker.scale.y = 0.01;
        marker.scale.z = 1.0;

        for(size_t i=0; i<cloud->points.size(); i++) {
            geometry_msgs::Point p;
            p.x = cloud->points[i].x;
            p.y = cloud->points[i].y;
            p.z = cloud->points[i].z;
            marker.points.push_back(p);
        }

        // the caller must decide the header; we are done here
        return marker;
    }

    std_msgs::ColorRGBA
    static getRandomColor()
    {
        std_msgs::ColorRGBA color;

        color.r = ((double)rand())/RAND_MAX;
        color.g = ((double)rand())/RAND_MAX;
        color.b = ((double)rand())/RAND_MAX;
        color.a = 1.0;

        return color;
    }

    // template <class PointCloudType>
    // static markersFromClusters(const PointCloudType& cloud, std::vector<pcl::PointIndices> cluster_indices)
    // {

    // }

    pcl::PointIndices::Ptr
    filterByHue(pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud, int hue, int hue_threshold)
    {
        float hue_min = (hue - hue_threshold) % 360;
        float hue_max = (hue + hue_threshold) % 360;

        pcl::PointIndices::Ptr indices (new pcl::PointIndices ());
        for(size_t i = 0; i < cloud->points.size(); i++)
        {
            // check to see if we are in range
            if(cloud->points[i].h < hue_max || cloud->points[i].h > hue_min)
                indices->indices.push_back(i);
        }

        return indices;
    }

    float
    calculateHue(pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud)
    {
        float hue_sum;
        for(size_t i = 0; i < cloud->points.size(); i++)
        {
            hue_sum += cloud->points[i].h;
        }

        return hue_sum/cloud->points.size();
    }

    // come on guys, why isn't this already in PCL?
    void PointCloudXYZRGBtoXYZHSV(pcl::PointCloud<pcl::PointXYZRGB>& in, pcl::PointCloud<pcl::PointXYZHSV>& out)
    {
        out.width = in.width;
        out.height = in.height;
        out.header = in.header;
        for (size_t i = 0; i < in.points.size (); i++) {
            pcl::PointXYZHSV p;
            pcl::PointXYZRGBtoXYZHSV (in.points[i], p);
            p.x = in.points[i].x;
            p.y = in.points[i].y;
            p.z = in.points[i].z;
            out.points.push_back (p);
        }
    }

    // come on guys, why isn't this already in PCL?
    void PointCloudXYZHSVtoXYZRGB(pcl::PointCloud<pcl::PointXYZHSV>& in, pcl::PointCloud<pcl::PointXYZRGB>& out)
    {
        out.width = in.width;
        out.height = in.height;
        out.header = in.header;
        for (size_t i = 0; i < in.points.size (); i++) {
            pcl::PointXYZRGB p;
            pcl::PointXYZHSVtoXYZRGB (in.points[i], p);
            p.x = in.points[i].x;
            p.y = in.points[i].y;
            p.z = in.points[i].z;
            out.points.push_back (p);
        }
    }
}