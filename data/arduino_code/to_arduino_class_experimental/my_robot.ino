#include "my_robot.h"



DifferentialDriveRobot::DifferentialDriveRobot()
: cmd_sub("/set_vel", &DifferentialDriveRobot::onTwist, this),
  servo_sub("/servo_cmd", &DifferentialDriveRobot::servo_cb, this),
  gripper_sub("/gripper_cmd", &DifferentialDriveRobot::gripper_cb, this),
  activate_arm_sub("/activate_arm", &DifferentialDriveRobot::activate_arm_cb, this),
  pid_sub("pid_set", &DifferentialDriveRobot::onPid_cb, this),
  enc_ticks_pub("encoder_ticks", &enc_ticks),
  vel_pub("velocity_wheels", &vel_wheels),
  myPID_fl(&Input_fl, &Output_fl, &Setpoint_fl, aggKp, aggKi, aggKd, DIRECT),
  myPID_fr(&Input_fr, &Output_fr, &Setpoint_fr, aggKp, aggKi, aggKd, DIRECT),
  myPID_bl(&Input_bl, &Output_bl, &Setpoint_bl, aggKp, aggKi, aggKd, DIRECT),
  myPID_br(&Input_br, &Output_br, &Setpoint_br, aggKp, aggKi, aggKd, DIRECT),
  encoder_fleft(Board::due),
  encoder_fright(Board::due),
  encoder_bleft(Board::due),
  encoder_bright(Board::due),
  ser(I2CAdd)
  
{
  
    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.subscribe(cmd_sub);
    nh.subscribe(servo_sub);
    nh.subscribe(gripper_sub);
    nh.subscribe(activate_arm_sub);
    nh.subscribe(pid_sub);
    nh.advertise(enc_ticks_pub);
    nh.advertise(vel_pub);

    
    

     
    myPID_fl.SetOutputLimits(-255, 255);
    myPID_fr.SetOutputLimits(-255, 255);
    myPID_bl.SetOutputLimits(-255, 255);
    myPID_br.SetOutputLimits(-255, 255);
  
    myPID_fl.SetMode(AUTOMATIC);
    myPID_fr.SetMode(AUTOMATIC);
    myPID_bl.SetMode(AUTOMATIC);
    myPID_br.SetMode(AUTOMATIC);
  
    myPID_fl.SetSampleTime(20);
    myPID_fr.SetSampleTime(20);
    myPID_bl.SetSampleTime(20);
    myPID_br.SetSampleTime(20);
    

  
    encoder_fright.begin();
    encoder_fleft.begin();
    encoder_bright.begin();
    encoder_bleft.begin();
    this -> setpins();
    this -> fix_encoder_ori_on_start();
    ser.Init(SERVO_MODE);

    char dim0_label[] = "encoder_ticks";
    enc_ticks.layout.dim = (std_msgs::MultiArrayDimension *) malloc(sizeof(std_msgs::MultiArrayDimension) * 2);
    enc_ticks.layout.dim[0].label = dim0_label;
    enc_ticks.layout.dim[0].size = 4;
    enc_ticks.layout.dim[0].stride = 1*4;
    enc_ticks.data = (long long int *)malloc(sizeof(long long int)*4);
    enc_ticks.layout.dim_length = 0;
    enc_ticks.data_length = 4;
  
    char dim1_label[] = "velocity_wheels";
    vel_wheels.layout.dim = (std_msgs::MultiArrayDimension *) malloc(sizeof(std_msgs::MultiArrayDimension) * 2);
    vel_wheels.layout.dim[0].label = dim1_label;
    vel_wheels.layout.dim[0].size = 4;
    vel_wheels.layout.dim[0].stride = 1*4;
    vel_wheels.data = (float *)malloc(sizeof(float)*4);
    vel_wheels.layout.dim_length = 0;
    vel_wheels.data_length = 4;
    
}

void DifferentialDriveRobot::activate_arm_cb(const std_msgs::Int16& act_arm)
{
  int arm = act_arm.data;
  if(arm == 1){

    ser.Sleep(false);
    int start_link_0 = 195;
    int start_link_1 = 27;
    int start_link_2 = 345;
    int start_link_3 = 225;
    int start_link_4 = 18;
    int start_link_5 = 210;
    int start_gripper = 300;
    ser.Servo(0, start_link_1);
    ser.Servo(1, start_link_2);
    ser.Servo(7, start_link_4);
    ser.Servo(2, start_link_0);
    ser.Servo(3, start_link_3);
    ser.Servo(9, start_link_5);
    ser.Servo(10, start_gripper);
    }
  else{
    ser.Sleep(true);
    }     
  
}

void DifferentialDriveRobot::servo_cb(const std_msgs::Int16MultiArray& cmd_msg)
{
  
  ser.Servo(2, cmd_msg.data[0]);
  ser.Servo(0, cmd_msg.data[1]);
  ser.Servo(1, cmd_msg.data[2]);
  ser.Servo(3, cmd_msg.data[3]);
  ser.Servo(7, cmd_msg.data[4]);
  ser.Servo(9, cmd_msg.data[5]);

}

void DifferentialDriveRobot::gripper_cb(const std_msgs::Int16& cmd_msg)
{
  
  ser.Servo(10, cmd_msg.data);

}

void DifferentialDriveRobot::onTwist(const std_msgs::Float32MultiArray& msg)
{

  float left_speed = msg.data[0];
  float right_speed = msg.data[1];
  Setpoint_fl = left_speed;
  Setpoint_fr = right_speed;
  Setpoint_bl = left_speed;
  Setpoint_br = right_speed;
  if(Setpoint_fl==0 && Setpoint_fr==0 && Setpoint_bl==0 &&Setpoint_br==0){
    wtf=true;
    }
  else{
    wtf = false;
    }

}

void DifferentialDriveRobot::onPid_cb(const std_msgs::Int16MultiArray& cmd_msg)
{
    int p = cmd_msg.data[0];
    int i = cmd_msg.data[1];
    int d = cmd_msg.data[2];
    myPID_fl.SetTunings(p, i, d);
    myPID_fr.SetTunings(p, i, d);
    myPID_bl.SetTunings(p, i, d);
    myPID_br.SetTunings(p, i, d);
}


void DifferentialDriveRobot::Move_motor(int speed_pwm,const uint8_t pwm,const uint8_t forw,const uint8_t back)
{

  if(speed_pwm >= 0)
  {
    digitalWrite(forw, HIGH);
    digitalWrite(back, LOW);
    analogWrite(pwm, abs(speed_pwm));
  }
  else if(speed_pwm < 0)
  {
    digitalWrite(forw, LOW);
    digitalWrite(back, HIGH);
    analogWrite(pwm, abs(speed_pwm));
  }
  
}

void DifferentialDriveRobot::setpins()
{

  pinMode(LF_FORW,OUTPUT);
  pinMode(LF_BACK,OUTPUT);
  pinMode(RF_FORW,OUTPUT);
  pinMode(RF_BACK,OUTPUT);
  pinMode(LF_PWM,OUTPUT);
  pinMode(RF_PWM,OUTPUT);
  pinMode(LB_FORW,OUTPUT);
  pinMode(LB_BACK,OUTPUT);
  pinMode(RB_FORW,OUTPUT);
  pinMode(RB_BACK,OUTPUT);
  pinMode(LB_PWM,OUTPUT);
  pinMode(RB_PWM,OUTPUT);
  digitalWrite(LF_FORW, HIGH);
  digitalWrite(LF_BACK, LOW);
  digitalWrite(RF_FORW, HIGH);
  digitalWrite(RF_BACK, LOW);
  digitalWrite(LB_FORW, HIGH);
  digitalWrite(LB_BACK, LOW);
  digitalWrite(RB_FORW, HIGH);
  digitalWrite(RB_BACK, LOW);

}


void DifferentialDriveRobot::fix_encoder_ori_on_start()
{

  int x=180;
  analogWrite(RF_PWM, x);
  analogWrite(LF_PWM, x);
  analogWrite(RB_PWM, x);  
  analogWrite(LB_PWM, x); 
  
  delay(300);

  analogWrite(RF_PWM, 0);
  analogWrite(LF_PWM, 0);
  analogWrite(RB_PWM, 0);
  analogWrite(LB_PWM, 0);

  int ct1 = encoder_fleft.count();
  int ct2 = encoder_fright.count();
  int ct3 = encoder_bleft.count();
  int ct4 = encoder_bright.count();
  if(ct1 < 0) {encoder_fleft.reverse();}
  if(ct2 < 0) {encoder_fright.reverse();}
  if(ct3 < 0) {encoder_bleft.reverse();}
  if(ct4 < 0) {encoder_bright.reverse();}
  
}

void DifferentialDriveRobot::reset_pid_Ki()
{
  
  myPID_fl.SetMode(MANUAL);
  myPID_fr.SetMode(MANUAL);
  myPID_bl.SetMode(MANUAL);
  myPID_br.SetMode(MANUAL);
  Output_fl=0;
  Output_fr=0;
  Output_bl=0;
  Output_br=0;

  myPID_fl.SetMode(AUTOMATIC);
  myPID_fr.SetMode(AUTOMATIC);
  myPID_bl.SetMode(AUTOMATIC);
  myPID_br.SetMode(AUTOMATIC);

}
void DifferentialDriveRobot::spin()
{

  // count encoder ticks
  ct1 = encoder_fleft.count();
  ct2 = encoder_fright.count();
  ct3 = encoder_bleft.count();
  ct4 = encoder_bright.count();
  // for some reason if i omit this it does not work properly
  if (ct1!=-1){
    enc_ticks.data[0] = ct1;}
  if (ct2!=-1){
    enc_ticks.data[1] = ct2;}
  if (ct3!=-1){
    enc_ticks.data[2] = ct3;}
  if (ct4!=-1){
    enc_ticks.data[3] = ct4;}
    
  enc_ticks.data[0]=ct1;
  enc_ticks.data[1]=ct2;
  enc_ticks.data[2]=ct3;
  enc_ticks.data[3]=ct4;
  // Publish encoder ticks to calculate odom on Jetson Nano side
  enc_ticks_pub.publish(&enc_ticks);


 // calculate time and current velocity
  
  now = millis();
  Input_fl = (float(ct1 - old_ct1) / 33000.1) / ((now - prev) / 1000.0);
  Input_fr = (float(ct2 - old_ct2) / 33000.1) / ((now - prev) / 1000.0);
  Input_bl = (float(ct3 - old_ct3) / 33000.1) / ((now - prev) / 1000.0);
  Input_br = (float(ct4 - old_ct4) / 33000.1) / ((now - prev) / 1000.0);
  vel_wheels.data[0] = Input_fl;
  vel_wheels.data[1] = Input_fr;
  vel_wheels.data[2] = Input_bl;
  vel_wheels.data[3] = Input_br;
  vel_pub.publish(&vel_wheels);
  // Compute  Pid
  myPID_fl.Compute();
  myPID_fr.Compute();
  myPID_bl.Compute();
  myPID_br.Compute();


  if(wtf){
    reset_pid_Ki();  
  }

  // Move the motors with the output of the pid
  
  this -> Move_motor(Output_fl,LF_PWM,LF_FORW,LF_BACK);
  this -> Move_motor(Output_fr,RF_PWM,RF_FORW,RF_BACK);
  this -> Move_motor(Output_bl,LB_PWM,LB_FORW,LB_BACK);
  this -> Move_motor(Output_br,RB_PWM,RB_FORW,RB_BACK);

  // spin the ros node
  
  nh.spinOnce();
  // take the old encoder ticks and time for calculating velocity
  old_ct1 = encoder_fleft.count();
  old_ct2 = encoder_fright.count();
  old_ct3 = encoder_bleft.count();
  old_ct4 = encoder_bright.count();

  prev = now;
  
  delay(25);
  
}






























/*
class DifferentialDriveRobot
{
          
    public:
        ros::NodeHandle nh;
        ros::Subscriber<geometry_msgs::Twist, DifferentialDriveRobot> sub;
        ros::Subscriber<geometry_msgs::Twist, DifferentialDriveRobot> sub1;
        std_msgs::Float32 test;
        ros::Publisher pub;
        
        void ddr_callback(const geometry_msgs::Twist& msg)  {}
        void ddr_callback1(const geometry_msgs::Twist& msg)  {}
        DifferentialDriveRobot()
        : sub("/cmd_vel_mux/input/teleop", &DifferentialDriveRobot::ddr_callback, this),
          sub1("/cmd_vel_mux/input/teleop", &DifferentialDriveRobot::ddr_callback1, this),
          pub("encoder_ticks", &test)
        {  // Constructor
           // sub = nh.subscribe("/cmd_vel_mux/input/teleop", &DifferentialDriveRobot::ddr_callback, this);
            nh.initNode();
            nh.subscribe(sub);
            nh.subscribe(sub1);
            nh.advertise(pub);
         }
};
*/
