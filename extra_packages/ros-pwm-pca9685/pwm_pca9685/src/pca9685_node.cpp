/* pca9685_i2c_node.cpp
 * Author: Dheera Venkatraman <dheera@dheera.net>
 *
 * Instantiates a PCA9685 Activity class, as well as
 * a Watchdog that causes this node to die if things aren't
 * working.
 */

#include <pwm_pca9685/pca9685_activity.h>
#include <csignal>

int main(int argc, char *argv[]) {
    ros::NodeHandle* nh = NULL;
    ros::NodeHandle* nh_priv = NULL;

    pwm_pca9685::PCA9685Activity* activity = NULL;

    ros::init(argc, argv, "pca9685_node");

    nh = new ros::NodeHandle();
    if(!nh) {
        ROS_FATAL("Failed to initialize NodeHandle");
        ros::shutdown();
        return -1;
    }

    nh_priv = new ros::NodeHandle("~");
    if(!nh_priv) {
        ROS_FATAL("Failed to initialize private NodeHandle");
        delete nh;
        ros::shutdown();
        return -2;
    }

    activity = new pwm_pca9685::PCA9685Activity(*nh, *nh_priv);

    if(!activity) {
        ROS_FATAL("Failed to initialize driver");
        delete nh_priv;
        delete nh;
        ros::shutdown();
        return -3;
    }

    if(!activity->start()) {
        ROS_ERROR("Failed to start activity");
        delete nh_priv;
        delete nh;
        ros::shutdown();
        return -4;
    }

    ros::Rate rate(100);
    while(ros::ok()) {
        rate.sleep();
        activity->spinOnce();
    }

    activity->stop();

    delete activity;
    delete nh_priv;
    delete nh;

    return 0;
}
