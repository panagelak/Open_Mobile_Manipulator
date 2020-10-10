#include "ros/ros.h"
#include "std_msgs/Int32.h"

class Simple_talker{
public:
	Simple_talker(){
		pub = nh.advertise<std_msgs::Int32>("counter", 10);
		count.data = 0;
	}
	void spin(){
		ros::Rate loop_rate(rate);
		while (ros::ok()){
			update();
			loop_rate.sleep();
		}	
	}
protected:
	ros::NodeHandle nh;
	ros::Publisher pub;
	double rate = 0.5;
	std_msgs::Int32 count;

	void update(){
		pub.publish(count);
		ROS_INFO("The counter is %d", count.data);
		ros::spinOnce();
		++count.data;
	}
};

int main(int argc, char** argv){
	ros::init(argc,argv, "topic_publisher");
	Simple_talker obj;
	obj.spin();
	return 0;
}