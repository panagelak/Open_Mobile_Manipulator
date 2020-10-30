#!/usr/bin/env python

import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder

import pickle
import tf

from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker

from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper_mine import *


def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy(
        '/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Callback function for your Point Cloud Subscriber


def pcl_callback(pcl_msg):

    # Convert ROS msg to PCL data

    PCL_data = ros_to_pcl(pcl_msg)
    cloud_filtered = PCL_data

    # Voxel Grid Downsampling filter
    # Create a VoxelGrid filter object for our input point cloud
    vox = PCL_data.make_voxel_grid_filter()

    # Choose a voxel (also known as leaf) size
    LEAF_SIZE = 0.005
    # Set the voxel (or leaf) size
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    # Call the filter function to obtain the resultant downsampled point cloud
    cloud_filtered = vox.filter()

    # PassThrough filter
    ################################
    #cloud_crop = cloud_filtered
    
    # Z axis
    # Create a PassThrough filter object.
    passthrough = cloud_filtered.make_passthrough_filter()
    # Assign axis and range to the passthrough filter object.
    filter_axis = 'z'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = 0.0
    axis_max = 1.5
    passthrough.set_filter_limits(axis_min, axis_max)
    # Finally use the filter function to obtain the resultant point cloud.
    cloud_filtered = passthrough.filter()

    # X axis

    filter_axis = 'x'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = -0.4
    axis_max = 0.4
    passthrough.set_filter_limits(axis_min, axis_max)
    # Finally use the filter function to obtain the resultant point cloud.
    cloud_filtered = passthrough.filter()

    # Y axis

    filter_axis = 'y'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = 0.0
    axis_max = 0.5
    passthrough.set_filter_limits(axis_min, axis_max)
    # Finally use the filter function to obtain the resultant point cloud.
    cloud_filtered = passthrough.filter()

    cloud_crop = cloud_filtered

    # RANSAC plane segmentation
    ################################

    # Create the segmentation object
    seg = cloud_filtered.make_segmenter()
    # Set the model you wish to fit
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    # Max distance for a point to be considered fitting the model
    max_distance = 0.01
    seg.set_distance_threshold(max_distance)
    # Call the segment function to obtain set of inlier indices and model coefficients
    inliers, coefficients = seg.segment()

    # Extract inliers pcd for table
    cloud_ground = cloud_filtered.extract(inliers, negative=False)
    cloud_filtered = cloud_filtered.extract(inliers, negative=True)

    cloud_objects = cloud_filtered

    # Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(cloud_objects)
    tree = white_cloud.make_kdtree()

    # Create a cluster extraction object
    ################################
    ec = white_cloud.make_EuclideanClusterExtraction()
    # Set tolerances for distance threshold
    # as well as minimum and maximum cluster size (in points)
    # NOTE: These are poor choices of clustering parameters
    # Your task is to experiment and find values that work for segmenting objects.
    ec.set_ClusterTolerance(0.012)
    ec.set_MinClusterSize(300)
    ec.set_MaxClusterSize(30000)

    # Search the k-d tree for clusters
    ec.set_SearchMethod(tree)

    # Extract indices for each of the discovered clusters
    cluster_indices = ec.Extract()

    # Create Cluster-Mask Point Cloud to visualize each cluster separately
    ################################
    # Assign a color corresponding to each segmented object in scene
    cluster_color = get_color_list(len(cluster_indices))

    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                             white_cloud[indice][1],
                                             white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])

    # Create new cloud containing all clusters, each with unique color
    ################################
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    # Convert PCL data to ROS messages
    ################################
    ros_crop_cloud = pcl_to_ros(cloud_crop)
    ros_cloud_ground = pcl_to_ros(cloud_ground)
    ros_cloud_objects = pcl_to_ros(cloud_objects)
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)

    # Publish ROS messages
    ################################
    pcl_crop_pub.publish(ros_crop_cloud)
    pcl_ground_pub.publish(ros_cloud_ground)
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_cluster_pub.publish(ros_cluster_cloud)

    # Exercise-3:
    # Classify the clusters!
    ################################
    detected_objects_labels = []
    detected_objects = []

    for index, pts_list in enumerate(cluster_indices):

        # Grab the points for the cluster
        pcl_cluster = cloud_objects.extract(pts_list)

        ros_cluster = pcl_to_ros(pcl_cluster)
        # Compute the associated feature vector

        # Extract histogram features
        # Generate Color hist
        c_hists = compute_color_histograms(ros_cluster, using_hsv=True)

        # Generate normals and notmal histograms
        normals = get_normals(ros_cluster)
        n_hists = compute_normal_histograms(normals)

        # Generate feature by concatenate of color and normals.
        feature = np.concatenate((c_hists, n_hists))

        # Make the prediction
        prediction = clf.predict(scaler.transform(feature.reshape(1, -1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        #label_pos[1] -= .4
        label_pos[2] += .1
        object_markers_pub.publish(make_label(label, label_pos, index))

        # Add the detected object to the list of detected objects.

        det_obj = DetectedObject()
        det_obj.label = label
        det_obj.cloud = ros_cluster
        detected_objects.append(det_obj)

    rospy.loginfo('Detected {} objects: {}'.format(
        len(detected_objects_labels), detected_objects_labels))

    # Publish the list of detected objects
    # This is the output you'll need to complete the upcoming project!
    ################################
    detected_objects_pub.publish(detected_objects)

    # Publish Transforms of the center of the points for later grasping
    if len(detected_objects) > 0:
        try:
            tf_center_pub(detected_objects)
        except rospy.ROSInterruptException:
            pass


def tf_center_pub(object_list):
    br = tf.TransformBroadcaster()
    #centroids = []
    labels = []
    objects_for_tf = []
    for object in object_list:
        # check for object with the same name if so pass (no double tf)
        label = object.label
        if label not in labels:
            labels.append(label)
        else:
            continue
        # Calculate center position of the object
        points_arr = ros_to_pcl(object.cloud).to_array()
        center = np.mean(points_arr, axis=0)[:3]
        objects_for_tf.append((center, label))

    for object in objects_for_tf:
        x = object[0][0]
        y = object[0][1]
        z = object[0][2]
        name = object[1]
        br.sendTransform((x, y, z),
                         tf.transformations.quaternion_from_euler(0, 0.0, 0),
                         rospy.Time.now(),
                         name,
                         "camera_rgb_optical_frame"
                         )


if __name__ == '__main__':

    # ROS node initialization
    ################################
    rospy.init_node('object_recognition', anonymous=True)

    # Create Subscribers
    ################################
    pcl_sub = rospy.Subscriber(
        "/sensor_stick/point_cloud", pc2.PointCloud2, pcl_callback, queue_size=1)

    # Create Publishers
    ################################
    #pcl_test_pub      = rospy.Publisher("/pcl_test"     , PointCloud2,          queue_size=1)
    pcl_objects_pub = rospy.Publisher(
        "/pcl_objects", PointCloud2,          queue_size=1)
    pcl_crop_pub = rospy.Publisher(
        "/pcl_crop", PointCloud2,          queue_size=1)
    pcl_ground_pub = rospy.Publisher(
        "/pcl_ground", PointCloud2,          queue_size=1)
    pcl_cluster_pub = rospy.Publisher(
        "/pcl_cluster", PointCloud2,          queue_size=1)
    object_markers_pub = rospy.Publisher(
        "/object_markers", Marker,               queue_size=1)
    detected_objects_pub = rospy.Publisher(
        "/detected_objects", DetectedObjectsArray, queue_size=1)

    # Initialize color_list
    get_color_list.color_list = []

    # Load Model From disk
    model = pickle.load(open(
        '/home/makemelive/workspaces/catkin_ws/src/Open_Mobile_Manipulator/sensor_stick/model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Spin while node is not shutdown
    ################################
    while not rospy.is_shutdown():
        rospy.spin()
