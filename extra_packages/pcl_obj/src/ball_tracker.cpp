
#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>

#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include "cloud_helpers.cpp"
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types_conversion.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
// PCL-ROS auto type conversions
#include <pcl_ros/point_cloud.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
class BallTracker
{
    public:
        BallTracker();
        void spin();

    private:
        void cloudCallback(const pcl::PCLPointCloud2ConstPtr& cloud_in);

        ros::NodeHandle nh;
        ros::Subscriber cloud_sub;
        ros::Publisher cloud_pub;
        ros::Publisher marker_pub;

        tf::TransformBroadcaster broadcaster;

        std::vector<std_msgs::ColorRGBA> marker_colors;
};


BallTracker::BallTracker()
{
    // load parameters
    ros::NodeHandle nh_priv("~");
    // nh_priv.param("linear_y_scale", linear_y_scale, 0.8);
    
    // lets show em what we got
    // ROS_INFO_STREAM("param linear_y_scale: " << linear_y_scale);

    // connects subs and pubs
    cloud_sub = nh.subscribe("/camera/depth_registered/points", 1, &BallTracker::cloudCallback, this);
    cloud_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("cloud_out", 1);
    marker_pub = nh.advertise<visualization_msgs::Marker>("marker", 1);


}

void BallTracker::spin()
{
    while(ros::ok())
    {
        // call all waiting callbacks
        ros::Duration(0.1).sleep();
        ros::spinOnce();
    }
}

void BallTracker::cloudCallback(const pcl::PCLPointCloud2ConstPtr& cloud_in)
{
    // convert to RGB from input
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in2 (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(*cloud_in, *cloud_in2);

    // convert to HSV for processing
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZHSV>);
    cloud_helpers::PointCloudXYZRGBtoXYZHSV(*cloud_in2, *cloud_filtered);

    // pick out red points
    pcl::PointIndices::Ptr redPoints = cloud_helpers::filterByHue(cloud_filtered, 240, 5);//350 20
    pcl::ExtractIndices<pcl::PointXYZHSV> extract;
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(redPoints);
    extract.setNegative(false);
    extract.filter(*cloud_filtered);

    // filter saturation
    pcl::PassThrough<pcl::PointXYZHSV> pass;
    pass.setInputCloud (cloud_filtered);
    pass.setFilterFieldName ("s");
    pass.setFilterLimits (0.8, 1.0);//0.8 1,0
    pass.setFilterLimitsNegative (false);
    pass.filter (*cloud_filtered);

    // filter value
    pass.setInputCloud (cloud_filtered);
    pass.setFilterFieldName ("v");
    pass.setFilterLimits (0.1, 0.95); //0.1 0.95
    pass.setFilterLimitsNegative (false);
    pass.filter (*cloud_filtered);

    // statistical outlier filter (Useful for carpet)
    pcl::StatisticalOutlierRemoval<pcl::PointXYZHSV> sor;
    sor.setInputCloud(cloud_filtered);
    sor.setMeanK(50);
    sor.setStddevMulThresh(0.01); // smaller is more restrictive
    sor.filter(*cloud_filtered);

    //plane segmentation
    pcl::ModelCoefficients coefficients;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        //create segmentation object 
    pcl::SACSegmentation<pcl::PointXYZHSV> segmentation;
    segmentation.setModelType(pcl::SACMODEL_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setMaxIterations(1000);
    segmentation.setDistanceThreshold(0.01);
    segmentation.setInputCloud(cloud_filtered);
    segmentation.segment(*inliers, coefficients);

    // Create the filtering object
    //pcl::ExtractIndices<pcl::PointXYZHSV> extract;
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_filtered);
    // segment by distance
    std::vector<pcl::PointIndices::Ptr> cluster_indices;
    cluster_indices = cloud_helpers::segmentByDistance(cloud_filtered);
    ROS_INFO_STREAM("clusters: " << cluster_indices.size());
    std::vector<pcl::PointCloud<pcl::PointXYZHSV>::Ptr> clusters;
    clusters = cloud_helpers::extractClusters(cloud_filtered, cluster_indices);



    // update transform
    if(clusters.size() > 0) {
        // find cloud average
        pcl::PointXYZHSV avg = cloud_helpers::getCloudAverage(clusters[0]);
        avg.x += 0.03; // m; adjust x position because my kinect isn't calibrated
        avg.z += 0.04; // m; adjust z because we are only seeing the front portion

        // get header
        std_msgs::Header header = pcl_conversions::fromPCL(cloud_filtered->header);

        // broadcast transform
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(avg.x, avg.y, avg.z) );
        transform.setRotation( tf::Quaternion(0, 0, 0) );
        broadcaster.sendTransform(tf::StampedTransform(transform, header.stamp, header.frame_id, "target"));
    }

    // publish filtered cloud for debugging
    if(cloud_pub.getNumSubscribers() > 0) {
        // convert to RGB for output
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZRGB>);
        cloud_helpers::PointCloudXYZHSVtoXYZRGB(*cloud_filtered, *cloud_out);
        cloud_pub.publish(*cloud_out);
    }

    // publish markers
    if(marker_pub.getNumSubscribers() > 0) {
        for(size_t i = 0; i < clusters.size(); i++) {
            // cloud
            visualization_msgs::Marker marker;
            marker = cloud_helpers::getCloudMarker(clusters[i]);
            marker.header = pcl_conversions::fromPCL(cloud_filtered->header);
            marker.pose.orientation.w = 1;
            marker.ns = "targets";
            marker.id = i + 1;
            if(marker_colors.size() < i+1)
                marker_colors.push_back(cloud_helpers::getRandomColor());
            marker.color = marker_colors[i];
            marker_pub.publish(marker);

            // label
            pcl::PointXYZHSV avg = cloud_helpers::getCloudAverage(clusters[i]);
            visualization_msgs::Marker marker_label;
            marker_label.ns = "target_labels";
            marker_label.id = marker.id;
            marker_label.action = visualization_msgs::Marker::ADD;
            //else marker.action = visualization_msgs::Marker::DELETE;
            marker_label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker_label.lifetime = ros::Duration();
            marker_label.text = "#" + boost::to_string(marker_label.id);
            marker_label.header = pcl_conversions::fromPCL(cloud_filtered->header);
            marker_label.pose.position.x = avg.x;
            marker_label.pose.position.y = avg.y - 0.15;
            marker_label.pose.position.z = avg.z;
            //marker_label.color = marker.color;
            marker_label.color.r = marker_label.color.g = marker_label.color.b = marker_label.color.a = 1.0;
            marker_label.scale.z = 0.1;
            marker_pub.publish(marker_label);

            // stats
            ROS_INFO_STREAM("cluster" << marker.id << ": " << clusters[i]->points.size() << " points with " << cloud_helpers::calculateHue(clusters[i]) << " average hue");
        }
    } // end publish markers

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ball_tracker");
    BallTracker ball_tracker;
    ball_tracker.spin();
}
