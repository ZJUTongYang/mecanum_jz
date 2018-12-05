#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <keyboard/Key.h>
#include <math.h>
#include <nav_msgs/Odometry.h>

//mecanum wheel config
//(1)leftfront(2)leftback(3)rightback(4)rightfront
#define lr 0.652
#define fb 0.650
#define wheelr 0.192
const double alpha[4] = {atan2(lr, fb),atan2(lr, -fb), atan2(-lr, -fb), atan2(-lr, fb)};
const double beta[4] = {M_PI/2 - alpha[0], M_PI/2 - alpha[1], 3*M_PI/2 - alpha[2], 3*M_PI/2 - alpha[3]};
const double gam[4] = {-M_PI/4, M_PI/4, -M_PI/4, M_PI/4};

const double moore_penrose_inv[3][4] = {0.048000000000, 0.048000000000, -0.048000000000, -0.048000000000, 
                                        -0.048000000000, 0.048000000000, 0.048000000000, -0.048000000000, 
                                        -0.073732718894009, -0.073732718894009, -0.073732718894009, -0.073732718894009};

class mecanum_odom
{
public:

mecanum_odom();
~mecanum_odom(){}
void encoderCallBack(const nav_msgs::Odometry::ConstPtr& msg);
void keyDownCallBack(const keyboard::Key::ConstPtr& msg);
void decode(double, double, double, double);
void publishOdom();
void plan(double vx_m_s, double vy_m_s, double w_rad_s);
double last_vel_[3];//YT store robot center velocity
	double encoder_m_s_[4];
private:
	ros::WallTime last_walltime_, now_walltime_;
	ros::Time last_time_in_msg_, now_time_in_msg_;
	ros::Subscriber encoder_sub_, encoder_test_sub_;
	ros::Subscriber key_sub_;
	ros::Publisher odom_pub_, modelstate_pub_;

	double x_m_, y_m_, w_rad_;
};
