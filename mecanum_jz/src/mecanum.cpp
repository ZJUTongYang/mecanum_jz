#include "mecanum.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

mecanum::mecanum()
{
	ros::NodeHandle n;
	wheel_sub_ = n.subscribe("/jzhw/robot_cmd", 1, &mecanum::robotCmdCallBack, this);
	key_sub_ = n.subscribe("/keyboard/keydown", 1, &mecanum::keyDownCallBack, this);
	wheel_pub_ = n.advertise<geometry_msgs::Twist>("/jzhw/cmd_vel", 1);
}
void mecanum::robotCmdCallBack(const geometry_msgs::Twist::ConstPtr& msg)
{

	vx_m_s_ = msg->linear.x;
	vy_m_s_ = msg->linear.y;
	w_rad_s_ = msg->angular.z;
	std::cout << "YT: receive command: [" << vx_m_s_ << ", " << vy_m_s_ << ", " << w_rad_s_ << "] " << std::endl;
	plan(vx_m_s_, vy_m_s_, w_rad_s_);
}

void mecanum::keyDownCallBack(const keyboard::Key::ConstPtr& msg)
{
  ROS_INFO("key code=%d", msg->code);
  /*switch(msg->code)
  {
  	case 273:
  	{
  		plan(0.01, 0, 0);
  		break;
  	}
  	case 274:
  	{
  		plan(-0.01, 0, 0);
  		break;
  	}
  	case 275:
  	{
  		plan(0, -0.01, 0);
  		break;
  	}
  	case 276:
  	{
  		plan(0, 0.01, 0);
  		break;
  	}
  	case 119:
  	{
  		plan(0.01, 0, 0);
  		break;
  	}
  	case 115:
  	{
  		plan(-0.01, 0, 0);
  		break;
  	}
  	case 97:
  	{
  		plan(0, 0.01, 0);
  		break;
  	}
  	case 100:
  	{
  		plan(0, -0.01, 0);
  		break;
  	}
  	case 113:
  	{
  		plan(0, 0, 0.01);
  		break;
  	}
  	case 101:
  	{
  		plan(0, 0, -0.01);
  		break;
  	}
  		
  }*/
}

void mecanum::publishwheelvel(double vel_0, double vel_1, double vel_2, double vel_3)
{
	geometry_msgs::Twist temp;
	temp.linear.x = vel_0;
	temp.linear.y = vel_1;
	//temp.linear.z = vel_2;
	//temp.angular.x = vel_3;
	temp.linear.z = -vel_2;
	temp.angular.x = -vel_3;	
	std::cout << "YT: publish command: [" << temp.linear.x << ", " << temp.linear.y << ", " << temp.linear.z << ", " << temp.angular.x << "]" <<std::endl;
	wheel_pub_.publish(temp);
}

void mecanum::plan(double vx_m_s, double vy_m_s, double w_rad_s)
{
	double wheel_vel_[4];
	for(unsigned int i = 0; i < 4; i++)
	{
		wheel_vel_[i] = sin(alpha[i]+beta[i]+gam[i])*vx_m_s -
						cos(alpha[i]+beta[i]+gam[i])*vy_m_s - 
						sqrt((lr/2)*(lr/2)+ (fb/2)*(fb/2))*cos(beta[i]+gam[i])*w_rad_s;
		wheel_vel_[i] /= r * cos(gam[i]);
	}
	publishwheelvel(wheel_vel_[0], wheel_vel_[1], wheel_vel_[2], wheel_vel_[3]);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mecanum_YT");
	mecanum n;
	ros::spin();
	return 0;
}
