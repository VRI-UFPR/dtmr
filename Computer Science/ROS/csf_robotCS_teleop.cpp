//DIGITAL TWIN MOBILE ROBOT
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
//#include "turtlesim/Pose.h"
#include "kbhit.h" 

using namespace std;

int main(int argc, char **argv){



	ros::init(argc, argv, "csf_robotCS_teleop");

	ros::NodeHandle n;

	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
 
	ros::Rate loop_rate(10);

	system("rosservice call reset");

	float vel_linear = 0.7;
	float vel_angular = 0.7;
	char tecla = '0';
	
	while (ros::ok()&&!(tecla == 'p')){

		geometry_msgs::Twist msg;

		if (kbhit())
			tecla = getchar();

		ros::spinOnce();//This command will read and update all ROS topics.

		switch(tecla){ 
			case 'w':	msg.linear.x = vel_linear;
								msg.angular.z = 0;
								ROS_INFO("Robot Front");
								break;
			case 's':	msg.linear.x = -(vel_linear);
								msg.angular.z = 0;
								ROS_INFO("Robot Rear");
								break;
			case 'a':	msg.linear.x = 0;
								msg.angular.z = vel_angular;
								ROS_INFO("Robot Left");
								break;
			case 'd':	msg.linear.x = 0;
								msg.angular.z = -(vel_angular);
								ROS_INFO("Robot Right");
								break;			
			default:	msg.linear.x = 0; //qualquer tecla
								msg.angular.z = 0;
								ROS_INFO("Robot Stop");
					
			}
			pub.publish(msg);
			ros::spinOnce();
			loop_rate.sleep();
	}
 	ROS_WARN("Teleoperacao de robos terrestres finalizada...");
	return 0;
}


