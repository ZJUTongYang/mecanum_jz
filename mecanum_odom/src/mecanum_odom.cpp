#include "mecanum_odom.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/ModelState.h>
#include "tf/transform_datatypes.h"

mecanum_odom::mecanum_odom()
{
	ros::NodeHandle n;
    encoder_sub_ = n.subscribe("/jzhw/odom", 1, &mecanum_odom::encoderCallBack, this);
    encoder_test_sub_ = n.subscribe("/jzhw/cmd_vel", 1, &mecanum_odom::testCmdvelCallBack, this);
	key_sub_ = n.subscribe("/keyboard/keydown", 1, &mecanum_odom::keyDownCallBack, this);
    odom_pub_ = n.advertise<geometry_msgs::Twist>("/jzhw/cmd_vel", 1);
    modelstate_pub_ = n.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1);
    x_m_ = 0;
    y_m_ = 0;
    w_rad_ = 0;
    last_vel_[0] = 0;
    last_vel_[1] = 0;
    last_vel_[2] = 0;
    last_walltime_ = ros::WallTime::now();
    now_walltime_ = ros::WallTime::now();
}

void mecanum_odom::testCmdvelCallBack(const geometry_msgs::Twist::ConstPtr& msg)
{
    last_walltime_ = now_walltime_;
    now_walltime_ = ros::WallTime::now();

    encoder_m_s_[0] = msg->linear.x;
    encoder_m_s_[1] = msg->linear.y;
    encoder_m_s_[2] = msg->linear.z;
    encoder_m_s_[3] = msg->angular.x;
    double temp[4];
    for(unsigned int i = 0; i < 4; i++)
    {
        // temp[i] = encoder_m_s_[i]/r;
        temp[i] = encoder_m_s_[i];
    }
    std::cout << "YT: receive velocity: [" << encoder_m_s_[0] << ", " << encoder_m_s_[1] << ", " << encoder_m_s_[2] << ", " << encoder_m_s_[3] 
              << "](m/s)" << std::endl << "                     =[" << temp[0] << ", " << temp[1] << ", " << temp[2] << ", " << temp[3] << "](rad/s)" << std::endl;
    
    //YT change the coordinate from jz to YT
    temp[2] *= -1;
    temp[3] *= -1;
    decode(temp[0], temp[1], temp[2], temp[3]);
}


void mecanum_odom::encoderCallBack(const geometry_msgs::Twist::ConstPtr& msg)
{
    last_walltime_ = now_walltime_;
    now_walltime_ = ros::WallTime::now();

    encoder_m_s_[0] = msg->linear.x;
    encoder_m_s_[1] = msg->linear.y;
    encoder_m_s_[2] = msg->linear.z;
    encoder_m_s_[3] = msg->angular.x;
    double temp[4];
    for(unsigned int i = 0; i < 4; i++)
    {
        temp[i] = encoder_m_s_[i]/wheelr;
	}
    std::cout << "YT: receive velocity: [" << encoder_m_s_[0] << ", " << encoder_m_s_[1] << ", " << encoder_m_s_[2] << ", " << encoder_m_s_[3] 
              << "](m/s)-->[" << temp[0] << ", " << temp[1] << ", " << temp[2] << ", " << temp[3] << "](rad/s)" << std::endl;
    
    //YT change the coordinate from jz to YT
    temp[2] *= -1;
    temp[3] *= -1;
    decode(temp[0], temp[1], temp[2], temp[3]);
}

void mecanum_odom::keyDownCallBack(const keyboard::Key::ConstPtr& msg)
{
  ROS_INFO("key code=%d", msg->code);
  switch(msg->code)
  {
    case 8:
    {
      ROS_INFO("YT: reset pose odom");
      x_m_ = 0;
      y_m_ = 0;
      w_rad_ = 0;
      break;
    }
  }
}

void mecanum_odom::publishOdom()
{
    gazebo_msgs::ModelState ms;
    ms.model_name = "jackal";
    ms.pose.position.x = x_m_;
    ms.pose.position.y = y_m_;
    tf::Quaternion q = tf::createQuaternionFromYaw(w_rad_);
    tf::quaternionTFToMsg(q, ms.pose.orientation);

    modelstate_pub_.publish(ms);
 //    ros::WallDuration d = now_walltime_ - last_walltime_;

    // x_m_ += temp.linear.x * d.toSec();
    // y_m_ += temp.linear.y * d.toSec();
    // w_rad_ += temp.angular.z * d.toSec();


    // std::cout << "YT: pose before now: [" << x_m_ << ", " << y_m_ << ", " << w_rad_ << "]" << std::endl;
}

void mecanum_odom::decode(double v0_rad_s, double v1_rad_s, double v2_rad_s, double v3_rad_s)
{
    //YT we use the new data for calculating present velocity, 
    //YT but the path before now has to be calculated based on the velocity of last time(temporarily written in publishOdom function)
	double wheel_vel_[4] = {v0_rad_s, v1_rad_s, v2_rad_s, v3_rad_s};
    double robot_current[3] = {0, 0, 0};
    for(unsigned int i = 0; i < 3; i++)
    {
        for(unsigned int j = 0; j < 4; j++)
        {
            robot_current[i] += moore_penrose_inv[i][j] * wheel_vel_[j];
        }
    }
//////////////////visualizing velocity of robot center//////////////////////////////////////////
    geometry_msgs::Twist temp;
    temp.linear.x = robot_current[0];
    temp.linear.y = robot_current[1];
    temp.angular.z = robot_current[2];

    if(fabs(temp.linear.x) < 0.0005)
        temp.linear.x = 0;
    if(fabs(temp.linear.y) < 0.0005)
        temp.linear.y = 0;
    if(fabs(temp.angular.z) < 0.0005)
        temp.angular.z = 0;


    std::cout << "YT: veolcity now: [" << temp.linear.x << ", " << temp.linear.y << ", " << temp.angular.z << "]" <<std::endl;
////////////////////////////////////////////////////////////////////
/////////////////////now for odometry///////////////////////////////////////////////
    // ros::WallDuration d = now_walltime_ - last_walltime_;
    ros::WallDuration d(1, 0);//assume 1 second between two command

    if(last_vel_[2] == 0)//no rotation
    {
        x_m_ += (last_vel_[0] * cos(w_rad_) - last_vel_[1] * sin(w_rad_))* d.toSec();
        y_m_ += (last_vel_[0] * sin(w_rad_) + last_vel_[1] * cos(w_rad_))* d.toSec(); 
    }
    else 
    {   
        std::cout << "MOVE WITH ROTATION" << std::endl;
        double min_radius = fabs(sqrt(last_vel_[0] * last_vel_[0] + last_vel_[1] * last_vel_[1])/last_vel_[2]);
        double angle_temp = fabs(last_vel_[2] * d.toSec());
        double oldpose[3] = {x_m_, y_m_, w_rad_ + atan2(last_vel_[1] , last_vel_[0])};
        double newpose[3];
        if(last_vel_[2] >= 0.0005)//a small rotation
        {

        // angle_temp = length_[i-1] / min_radius;
        // if(type_[i-1] == DUBINS_LEFT)
        // {
        //     //YT CREATE TEMP[i]
        //     temp[i].x = temp[i-1].x + min_radius * (sin(temp[i-1].t) * (cos(angle_temp) - 1) +  cos(temp[i-1].t) * sin(angle_temp) );
        //     temp[i].y = temp[i-1].y + min_radius * (-cos(temp[i-1].t) * (cos(angle_temp) - 1) +  sin(temp[i-1].t) * sin(angle_temp) );
        //     temp[i].t = temp[i-1].t + angle_temp;
        // }

            //YT CREATE TEMP[i]
            newpose[0] = oldpose[0] + min_radius * (sin(oldpose[2]) * (cos(angle_temp) - 1) +  cos(oldpose[2]) * sin(angle_temp) );
            newpose[1] = oldpose[1] + min_radius * (-cos(oldpose[2]) * (cos(angle_temp) - 1) +  sin(oldpose[2]) * sin(angle_temp) );
            newpose[2] = oldpose[2] + angle_temp - atan2(last_vel_[1], last_vel_[0]);
        }
        else if(last_vel_[2] <= 0.0005)
        {
        // else if(type_[i-1] == DUBINS_RIGHT)
        // {
        //     //YT CREATE TEMP[i]
        //     temp[i].x = temp[i-1].x + min_radius * (sin(temp[i-1].t) * ( 1- cos(angle_temp)) +  cos(temp[i-1].t) * sin(angle_temp) );
        //     temp[i].y = temp[i-1].y + min_radius * (-cos(temp[i-1].t) * ( 1 - cos(angle_temp)) +  sin(temp[i-1].t) * sin(angle_temp) );
        //     temp[i].t = temp[i-1].t - angle_temp;
        // }
            //YT CREATE TEMP[i]
            newpose[0] = oldpose[0] + min_radius * (sin(oldpose[2]) * ( 1- cos(angle_temp)) +  cos(oldpose[2]) * sin(angle_temp) );
            newpose[1] = oldpose[1] + min_radius * (-cos(oldpose[2]) * ( 1 - cos(angle_temp)) +  sin(oldpose[2]) * sin(angle_temp) );
            newpose[2] = oldpose[2] - angle_temp - atan2(last_vel_[1], last_vel_[0]);
        }   
        x_m_ = newpose[0];
        y_m_ = newpose[1];
        w_rad_ = newpose[2];
    }


    last_vel_[0] = robot_current[0];
    last_vel_[1] = robot_current[1];
    last_vel_[2] = robot_current[2];


    std::cout << "YT: pose before now: [" << x_m_ << ", " << y_m_ << ", " << w_rad_ << "]" << std::endl;

    //temp is measured in m/s and rad/s
    publishOdom();
    
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mecanum_odom_YT");
	mecanum_odom n;
	ros::spin();
	return 0;
}
