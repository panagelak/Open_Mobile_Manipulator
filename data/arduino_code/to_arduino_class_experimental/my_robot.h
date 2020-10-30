#ifndef MY_ROBOT_H
#define MY_ROBOT_H

#define USE_USBCON
#include "Arduino.h"
#include "ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "quadrature.h"
#include "geometry_msgs/Twist.h"
#include "PID_v1.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/Int64MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int16.h"
#include "HCPCA9685.h"
#define I2CAdd 0x40
//HCPCA9685 HCPCA9685(I2CAdd);

class DifferentialDriveRobot{

public:

    DifferentialDriveRobot();
    
    void activate_arm_cb(const std_msgs::Int16& act_arm);
    void servo_cb(const std_msgs::Int16MultiArray& cmd_msg);
    void gripper_cb(const std_msgs::Int16& cmd_msg);
    void onPid_cb(const std_msgs::Int16MultiArray& cmd_msg);
    void onTwist(const std_msgs::Float32MultiArray& msg);

    void Move_motor(int speed_pwm,const uint8_t pwm,const uint8_t forw,const uint8_t back);
    void setpins();
    void fix_encoder_ori_on_start();
    void reset_pid_Ki();
    void spin();

    
protected:

    ros::NodeHandle nh;
    ros::Subscriber<std_msgs::Float32MultiArray, DifferentialDriveRobot> cmd_sub;
    ros::Subscriber<std_msgs::Int16MultiArray, DifferentialDriveRobot> servo_sub;
    ros::Subscriber<std_msgs::Int16, DifferentialDriveRobot> gripper_sub;
    ros::Subscriber<std_msgs::Int16, DifferentialDriveRobot> activate_arm_sub;
    ros::Subscriber<std_msgs::Int16MultiArray, DifferentialDriveRobot> pid_sub;
    ros::Publisher enc_ticks_pub;
    ros::Publisher vel_pub;
    std_msgs::Int64MultiArray enc_ticks;
    std_msgs::Float64MultiArray vel_wheels;

    

    double Setpoint_fl, Input_fl, Output_fl;
    double Setpoint_fr, Input_fr, Output_fr;
    double Setpoint_bl, Input_bl, Output_bl;
    double Setpoint_br, Input_br, Output_br;
    double aggKp=650, aggKi=1200, aggKd=0.25;

    const uint8_t RF_PWM = 11;
    const uint8_t RF_BACK = 27;
    const uint8_t RF_FORW = 26;
    const uint8_t LF_BACK = 25;
    const uint8_t LF_FORW = 24;
    const uint8_t LF_PWM = 9;
    const uint8_t RB_PWM = 12;
    const uint8_t RB_BACK = 30;
    const uint8_t RB_FORW = 31;
    const uint8_t LB_BACK = 28;
    const uint8_t LB_FORW = 29;
    const uint8_t LB_PWM = 10;
    bool wtf;
    unsigned long prev = 0, now;
    int old_ct1=0, old_ct2=0, old_ct3=0, old_ct4=0;
    int ct1, ct2, ct3, ct4;
    PID myPID_fl;//(&Input_fl, &Output_fl, &Setpoint_fl, aggKp, aggKi, aggKd, DIRECT);
    PID myPID_fr;//(&Input_fr, &Output_fr, &Setpoint_fr, aggKp, aggKi, aggKd, DIRECT);
    PID myPID_bl;//(&Input_bl, &Output_bl, &Setpoint_bl, aggKp, aggKi, aggKd, DIRECT);
    PID myPID_br;//(&Input_br, &Output_br, &Setpoint_br, aggKp, aggKi, aggKd, DIRECT); 
    HCPCA9685 ser;
    Quadrature_encoder<46,47> encoder_fright;
    Quadrature_encoder<49,48> encoder_fleft;
    Quadrature_encoder<42,43> encoder_bright;
    Quadrature_encoder<44,45> encoder_bleft;
};

#endif
