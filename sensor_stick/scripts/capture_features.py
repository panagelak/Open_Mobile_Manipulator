#!/usr/bin/env python
import numpy as np
import pickle
import rospy

from sensor_stick.pcl_helper import *
from sensor_stick.training_helper import spawn_model
from sensor_stick.training_helper import delete_model
from sensor_stick.training_helper import initial_setup
from sensor_stick.training_helper import capture_sample
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from sensor_stick.srv import GetNormals
from geometry_msgs.msg import Pose
from sensor_msgs.msg import PointCloud2


def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy(
        '/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster


if __name__ == '__main__':
    rospy.init_node('capture_node')

    models = [\
        # 'arm_part',
        'box_red',
        'monster'
        # 'book',
        # 'bowl',
        # 'create',
        # 'disk_part',
        # 'eraser',
        # 'glue',
        # 'hammer',
        # 'plastic_cup',
        # 'snacks',
        # 'soap',
        # 'soap2',
        # 'soda_can',
        # 'sticky_notes'
    ]

    # Disable gravity and delete the ground plane
    initial_setup()
    labeled_features = []

    # Spawn models one by one.
    for model_name in models:
        spawn_model(model_name)

        # capture ten samples for each model
        ####################################
        for i in range(100):

            # make five attempts to get a valid point cloud then give up
            sample_was_good = False
            try_count = 0
            while not sample_was_good and try_count < 5:
                sample_cloud = capture_sample()
                sample_cloud_arr = ros_to_pcl(sample_cloud).to_array()

                # Check for invalid clouds.
                if sample_cloud_arr.shape[0] == 0:
                    print('Invalid cloud detected')
                    try_count += 1
                else:
                    sample_was_good = True

            # Extract Features
            ##################
            # Generate Color Histogram for the spawned model
            # Enable using_hsv for better results
            c_hists = compute_color_histograms(sample_cloud, using_hsv=True)

            # Generate normals and notmal histograms for the spawned model
            normals = get_normals(sample_cloud)
            n_hists = compute_normal_histograms(normals)

            # Generate feature by concatenate of color and normals.
            feature = np.concatenate((c_hists, n_hists))

            # add feature to list
            labeled_features.append([feature, model_name])

        # remove the spawnd model
        delete_model()

    pickle.dump(labeled_features, open('training_set.sav', 'wb'))
