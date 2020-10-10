/* pca9685_activity.cpp
 * Author: Dheera Venkatraman <dheera@dheera.net>
 *
 * Defines a PCA9685 Activity class, constructed with node handles
 * and which handles all ROS duties.
 */

#include "pwm_pca9685/pca9685_activity.h"

namespace pwm_pca9685 {

// ******** constructors ******** //

PCA9685Activity::PCA9685Activity(ros::NodeHandle &_nh, ros::NodeHandle &_nh_priv) :
  nh(_nh), nh_priv(_nh_priv) {
    ROS_INFO("initializing");
    nh_priv.param("device", param_device, (std::string)"/dev/i2c-1");
    nh_priv.param("address", param_address, (int)PCA9685_ADDRESS);
    nh_priv.param("frequency", param_frequency, (int)50);
    nh_priv.param("frame_id", param_frame_id, (std::string)"imu");
    
    // timeouts in milliseconds per channel
    nh_priv.param("timeout", param_timeout, std::vector<int>{
        5000, 5000, 5000, 5000, 5000, 5000, 5000, 5000,
        5000, 5000, 5000, 5000, 5000, 5000, 5000, 5000
    });

    // minimum pwm value per channel
    nh_priv.param("pwm_min", param_pwm_min, std::vector<int>{
        1600, 2200, 2400, 1800, 2000, 2000, 4000, 1000,
        1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000
    });

    // maximum pwm value per channel
    nh_priv.param("pwm_max", param_pwm_max, std::vector<int>{
        8200, 8200, 7800, 8200, 8300, 8300, 7500, 10000,
        10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000
    });

    // default pwm value per channel after timeout is reached
    nh_priv.param("timeout_value", param_timeout_value, std::vector<int>{
        0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0
    });

    if(param_timeout.size() != 16) {
        ROS_ERROR("size of param timeout must be 16");
        ros::shutdown();
    }

    if(param_timeout_value.size() != 16) {
        ROS_ERROR("size of param timeout_value must be 16");
        ros::shutdown();
    }

    if(param_pwm_min.size() != 16) {
        ROS_ERROR("size of param pwm_min must be 16");
        ros::shutdown();
    }

    if(param_pwm_max.size() != 16) {
        ROS_ERROR("size of param pwm_min must be 16");
        ros::shutdown();
    }

    if(param_address < 0 || param_address > 127) {
        ROS_ERROR("param address must be between 0 and 127 inclusive");
        ros::shutdown();
    }

    if(param_frequency <= 0) {
        ROS_ERROR("param frequency must be positive");
        ros::shutdown();
    }

    ros::Time time = ros::Time::now();
    uint64_t t = 1000 * (uint64_t)time.sec + (uint64_t)time.nsec / 1e6;

    for(int channel = 0; channel < 16; channel++) {
      last_set_times[channel] = t;
      last_change_times[channel] = t;
      last_data[channel] = 0;
    }
}

// ******** private methods ******** //

bool PCA9685Activity::reset() {
    int i = 0;

    // reset
    _i2c_smbus_write_byte_data(file, PCA9685_MODE1_REG, 0b10000000);
    ros::Duration(0.500).sleep();

    // set frequency
    uint8_t prescale = (uint8_t)(25000000.0 / 4096.0 / param_frequency + 0.5);

    _i2c_smbus_write_byte_data(file, PCA9685_MODE1_REG, 0b00010000); // sleep
    ros::Duration(0.025).sleep();

    _i2c_smbus_write_byte_data(file, PCA9685_PRESCALE_REG, prescale); // set prescale
    ros::Duration(0.025).sleep();

    _i2c_smbus_write_byte_data(file, PCA9685_MODE2_REG, 0x04); // outdrv
    ros::Duration(0.025).sleep();

    _i2c_smbus_write_byte_data(file, PCA9685_MODE1_REG, 0xA1); // un-sleep
    ros::Duration(0.025).sleep();

    _i2c_smbus_write_byte_data(file, PCA9685_CHANNEL_ALL_REG, 0);
    _i2c_smbus_write_byte_data(file, PCA9685_CHANNEL_ALL_REG + 1, 0);
    _i2c_smbus_write_byte_data(file, PCA9685_CHANNEL_ALL_REG + 2, 0);
    _i2c_smbus_write_byte_data(file, PCA9685_CHANNEL_ALL_REG + 3, 0);

    return true;
}

bool PCA9685Activity::set(uint8_t channel, uint16_t value) {
    uint16_t value_12bit = value >> 4;
    uint8_t values[4];
    if(value_12bit == 0x0FFF) { // always on
    	values[0] = 0x10;
    	values[1] = 0x00;
    	values[2] = 0x00;
    	values[3] = 0x00;
    } else if(value_12bit == 0x0000) { // always off
    	values[0] = 0x00;
    	values[1] = 0x00;
    	values[2] = 0x10;
    	values[3] = 0x00;
    } else { // PWM
    	values[0] = 0x00;
    	values[1] = 0x00;
    	values[2] = (value_12bit + 1) & 0xFF;
    	values[3] = (value_12bit + 1) >> 8;
    }

    _i2c_smbus_write_i2c_block_data(file, PCA9685_CHANNEL_0_REG + (channel * 4), 4, values);
}

// ******** public methods ******** //

bool PCA9685Activity::start() {
    ROS_INFO("starting");

    if(!sub_command) sub_command = nh.subscribe("command", 1, &PCA9685Activity::onCommand, this);

    file = open(param_device.c_str(), O_RDWR);
    if(ioctl(file, I2C_SLAVE, param_address) < 0) {
        ROS_ERROR("i2c device open failed");
        return false;
    }

    if(!reset()) {
        ROS_ERROR("chip reset and setup failed");
        return false;
    }

    return true;
}

bool PCA9685Activity::spinOnce() {
    ros::spinOnce();
    ros::Time time = ros::Time::now();
    uint64_t t = 1000 * (uint64_t)time.sec + (uint64_t)time.nsec / 1e6;

    if(seq++ % 10 == 0) {
      for(int channel = 0; channel < 16; channel++) {
        // positive timeout: timeout when no cammand is received
        if(param_timeout[channel] > 0 && t - last_set_times[channel] > std::abs(param_timeout[channel])) {
          set(channel, param_timeout_value[channel]);
        }
        // negative timeout: timeout when value doesn't change
	else if(param_timeout[channel] < 0 && t - last_change_times[channel] > std::abs(param_timeout[channel])) {
          set(channel, param_timeout_value[channel]);
	  ROS_WARN_STREAM("timeout " << channel);
        }
	// zero timeout: no timeout
      }
    }

    return true;    
}

bool PCA9685Activity::stop() {
    ROS_INFO("stopping");

    if(sub_command) sub_command.shutdown();

    return true;
}

void PCA9685Activity::onCommand(const std_msgs::Int32MultiArrayPtr &msg) {
    ros::Time time = ros::Time::now();
    uint64_t t = 1000 * (uint64_t)time.sec + (uint64_t)time.nsec / 1e6;

    if(msg->data.size() != 16) {
        ROS_ERROR("array is not have a size of 16");
        return;
    }

    for(int channel = 0; channel < 16; channel++) {
      if(msg->data[channel] < 0) continue;

      if(msg->data[channel] != last_data[channel]) {
          last_change_times[channel] = t;
      }

      if(msg->data[channel] == last_data[channel] && param_timeout[channel]) continue;

      if(msg->data[channel] > param_pwm_max[channel]) {
	  set(channel, param_pwm_max[channel]);
      } else if(msg->data[channel] < param_pwm_min[channel]) {
          set(channel, param_pwm_min[channel]);
      } else {
          set(channel, msg->data[channel]);
      }
      last_set_times[channel] = t;
      last_data[channel] = msg->data[channel];
    }
}


}
