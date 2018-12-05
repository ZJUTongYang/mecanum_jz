#include "mecanum_odom.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/ModelState.h>
#include "tf/transform_datatypes.h"
#include <nav_msgs/Odometry.h>
#include <fstream>
#include <iostream>

mecanum_odom::mecanum_odom()
{
	ros::NodeHandle n;
    encoder_sub_ = n.subscribe("/jzhw/odom", 1, &mecanum_odom::encoderCallBack, this);
	key_sub_ = n.subscribe("/keyboard/keydown", 1, &mecanum_odom::keyDownCallBack, this);
    odom_pub_ = n.advertise<geometry_msgs::PoseStamped>("/jzhw/odom_from_encoder", 1);

    x_m_ = 0;
    y_m_ = 0;
    w_rad_ = 0;
    last_vel_[0] = 0;
    last_vel_[1] = 0;
    last_vel_[2] = 0;
    last_walltime_ = ros::WallTime::now();
    now_walltime_ = ros::WallTime::now();
    last_time_in_msg_ = ros::Time::now();
    now_time_in_msg_ = ros::Time::now();
}

// void mecanum_odom::testCmdvelCallBack(const geometry_msgs::Twist::ConstPtr& msg)
// {
//     last_walltime_ = now_walltime_;
//     now_walltime_ = ros::WallTime::now();

//     encoder_m_s_[0] = msg->linear.x;
//     encoder_m_s_[1] = msg->linear.y;
//     encoder_m_s_[2] = msg->linear.z;
//     encoder_m_s_[3] = msg->angular.x;
//     double temp[4];
//     for(unsigned int i = 0; i < 4; i++)
//     {
//         // temp[i] = encoder_m_s_[i]/r;
//         temp[i] = encoder_m_s_[i];
//     }
//     std::cout << "YT: receive velocity: [" << encoder_m_s_[0] << ", " << encoder_m_s_[1] << ", " << encoder_m_s_[2] << ", " << encoder_m_s_[3] 
//               << "](m/s)" << std::endl << "                     =[" << temp[0] << ", " << temp[1] << ", " << temp[2] << ", " << temp[3] << "](rad/s)" << std::endl;
    
//     //YT change the coordinate from jz to YT
//     temp[2] *= -1;
//     temp[3] *= -1;
//     decode(temp[0], temp[1], temp[2], temp[3]);
// }


void mecanum_odom::encoderCallBack(const nav_msgs::Odometry::ConstPtr& msg)
{
    last_walltime_ = now_walltime_;
    now_walltime_ = ros::WallTime::now();

    last_time_in_msg_ = now_time_in_msg_;
    now_time_in_msg_ = msg->header.stamp;

    encoder_m_s_[0] = msg->twist.twist.linear.x;
    encoder_m_s_[1] = msg->twist.twist.linear.y;
    encoder_m_s_[2] = msg->twist.twist.linear.z;
    encoder_m_s_[3] = msg->twist.twist.angular.x;
    double temp[4];
    for(unsigned int i = 0; i < 4; i++)
    {
        // temp[i] = encoder_m_s_[i]/wheelr;
        temp[i] = encoder_m_s_[i];
	}
    std::cout << "YT: receive velocity: [" << encoder_m_s_[0] << ", " << encoder_m_s_[1] << ", " << encoder_m_s_[2] << ", " << encoder_m_s_[3] 
              << "](m/s)" << std::endl;

    std::ofstream f;
    f.open("/home/jz/yt_odom_feedback.txt", std::ios::app);

    f << encoder_m_s_[0] << ", " << encoder_m_s_[1] << ", " 
      << encoder_m_s_[2] << ", " << encoder_m_s_[3] << ", " 
      << ros::WallDuration(now_walltime_ - last_walltime_).toSec() << ", "
      << ros::Duration(now_time_in_msg_ - last_time_in_msg_).toSec() << "; ";// << std::endl;

    f.close();

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
    geometry_msgs::PoseStamped p;
    p.pose.position.x = x_m_;
    p.pose.position.y = y_m_;

    tf::Quaternion q = tf::createQuaternionFromYaw(w_rad_);
    tf::quaternionTFToMsg(q, p.pose.orientation);
    odom_pub_.publish(p);
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


    std::cout << "YT: center veolcity: [" << temp.linear.x << ", " << temp.linear.y << ", " << temp.angular.z << "]" <<std::endl;
////////////////////////////////////////////////////////////////////
    std::ofstream f;
    f.open("/home/jz/yt_odom_feedback.txt", std::ios::app);

    f << temp.linear.x << ", " << temp.linear.y << ", " 
      << temp.angular.z << "; " << std::endl;

    f.close();
/////////////////////now for odometry///////////////////////////////////////////////
    ros::WallDuration d = now_walltime_ - last_walltime_;
    // ros::WallDuration d(1, 0);//assume 1 second between two command

    if(last_vel_[2] == 0)//no rotation
    {
        x_m_ += (last_vel_[0] * cos(w_rad_) - last_vel_[1] * sin(w_rad_))* d.toSec();
        y_m_ += (last_vel_[0] * sin(w_rad_) + last_vel_[1] * cos(w_rad_))* d.toSec(); 
    }
    else 
    {   
        // std::cout << "MOVE WITH ROTATION" << std::endl;
        double min_radius = fabs(sqrt(last_vel_[0] * last_vel_[0] + last_vel_[1] * last_vel_[1])/last_vel_[2]);
        double angle_temp = fabs(last_vel_[2] * d.toSec());
        double oldpose[3] = {x_m_, y_m_, w_rad_ + atan2(last_vel_[1] , last_vel_[0])};
        double newpose[3];
        if(last_vel_[2] >= 0.0005)//a small rotation
        {
            //YT CREATE TEMP[i]
            newpose[0] = oldpose[0] + min_radius * (sin(oldpose[2]) * (cos(angle_temp) - 1) +  cos(oldpose[2]) * sin(angle_temp) );
            newpose[1] = oldpose[1] + min_radius * (-cos(oldpose[2]) * (cos(angle_temp) - 1) +  sin(oldpose[2]) * sin(angle_temp) );
            newpose[2] = oldpose[2] + angle_temp - atan2(last_vel_[1], last_vel_[0]);
        }
        else if(last_vel_[2] <= 0.0005)
        {
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

    if(w_rad_ > M_PI )
        w_rad_ -= M_PI * 2;
    if(w_rad_ < -M_PI)
        w_rad_ += M_PI * 2;

    std::cout << "YT: center pose: [" << x_m_ << ", " << y_m_ << ", " << w_rad_ << "]" << std::endl;

    //temp is measured in m/s and rad/s
    publishOdom();
    
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mecanum_odom_YT");
	mecanum_odom n;

//     double data[41][5] = {
// 0.521072, 0.521025, 0.520948, 0.520783, 0.0399745,
// 0.520792, 0.520915, 0.520762, 0.521079, 0.0400073,
// 0.52092, 0.521031, 0.520403, 0.52095, 0.0400359,
// 0.520892, 0.521211, 0.520513, 0.520908, 0.039957,
// 0.520933, 0.5209, 0.520689, 0.520984, 0.0400034,
// 0.520954, 0.520832, 0.520621, 0.520916, 0.0400275,
// 0.520643, 0.520863, 0.520759, 0.520914, 0.0400009,
// 0.5208, 0.520855, 0.520644, 0.520997, 0.0399714,
// 0.520941, 0.520567, 0.520715, 0.520955, 0.040762,
// 0.520823, 0.520499, 0.520741, 0.520874, 0.039299,
// 0.520936, 0.521075, 0.520856, 0.520969, 0.0401556,
// 0.520973, 0.52074, 0.52093, 0.52111, 0.0398349,
// 0.520947, 0.520957, 0.520932, 0.520979, 0.0399943,
// 0.520852, 0.520918, 0.520846, 0.520935, 0.039998,
// 0.520923, 0.520557, 0.520825, 0.520649, 0.0400028,
// 0.520719, 0.521015, 0.52072, 0.520835, 0.0195961,
// 0.52074, 0.521185, 0.520642, 0.520757, 0.0142386,
// 0.520947, 0.52051, 0.520662, 0.521045, 0.0381545,
// 0.520844, 0.520465, 0.520786, 0.520785, 0.0402391,
// 0.520889, 0.520253, 0.52078, 0.520979, 0.0398374,
// 0.52097, 0.520387, 0.520964, 0.520887, 0.00460595,
// 0.521004, 0.520395, 0.520943, 0.521042, 0.0393078,
// 0.520902, 0.520557, 0.520613, 0.52095, 0.0400229,
// 0.520601, 0.520662, 0.520906, 0.52077, 0.0399857,
// 0.520894, 0.520538, 0.520992, 0.520652, 0.0400581,
// 0.520894, 0.520583, 0.520916, 0.520853, 0.00149854,
// 0.52092, 0.52062, 0.52095, 0.520942, 0.0388771,
// 0.520939, 0.520593, 0.521003, 0.520838, 0.0395474,
// 0.520837, 0.520591, 0.520937, 0.520966, 0.0400013,
// 0.520939, 0.520541, 0.521026, 0.520898, 0.0399914,
// 0.520941, 0.520481, 0.521102, 0.52116, 0.0400457,
// 0.520868, 0.520753, 0.520835, 0.520788, 0.0110379,
// 0.521041, 0.520832, 0.520914, 0.52101, 0.0369454,
// 0.520933, 0.52068, 0.520906, 0.520955, 0.0400152,
// 0.520826, 0.520879, 0.520964, 0.52095, 0.0400259,
// 0.520755, 0.520806, 0.520875, 0.520908, 0.0400007,
// 0.520666, 0.520855, 0.520843, 0.521016, 0.00308999,
// 0.520999, 0.520881, 0.520814, 0.520654, 0.0370555,
// 0.520844, 0.520947, 0.520935, 0.520895, 0.039843,
// 0.520944, 0.521023, 0.52093, 0.520633, 0.00122169,
// 0.520983, 0.520536, 0.520937, 0.521013, 0.0387835
//     };

//     for(unsigned int index= 0; index < 40; index++)
//     {
//     n.encoder_m_s_[0] = data[index][0];
//     n.encoder_m_s_[1] = data[index][1];
//     n.encoder_m_s_[2] = data[index][2];
//     n.encoder_m_s_[3] = data[index][3];
//     double temp[4];
//     for(unsigned int i = 0; i < 4; i++)
//     {
//         temp[i] = n.encoder_m_s_[i]/wheelr;
//     }
//     std::cout << "YT: receive velocity: [" << n.encoder_m_s_[0] << ", " << n.encoder_m_s_[1] << ", " << n.encoder_m_s_[2] << ", " << n.encoder_m_s_[3] 
//               << "](m/s)" << std::endl;

//     //YT change the coordinate from jz to YT
//     temp[2] *= -1;
//     temp[3] *= -1;
//     n.decode(temp[0], temp[1], temp[2], temp[3]);

//     }

	ros::spin();
	return 0;
}
