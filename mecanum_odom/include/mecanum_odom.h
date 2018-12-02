#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <keyboard/Key.h>
#include <math.h>

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
// void robotCmdCallBack(const geometry_msgs::Twist::ConstPtr& msg);
void encoderCallBack(const geometry_msgs::Twist::ConstPtr& msg);
void testCmdvelCallBack(const geometry_msgs::Twist::ConstPtr& msg);
void keyDownCallBack(const keyboard::Key::ConstPtr& msg);
void decode(double, double, double, double);
// void publishwheelvel(double vel_0, double vel_1, double vel_2, double vel_3);
// void publishOdom(double vx_m_s, double vy_m_s, double w_rad_s);
void publishOdom();
void plan(double vx_m_s, double vy_m_s, double w_rad_s);
double last_vel_[3];//YT store robot center velocity
private:
	ros::WallTime last_walltime_, now_walltime_;
	ros::Subscriber encoder_sub_, encoder_test_sub_;
// ros::Subscriber wheel_sub_;
	ros::Subscriber key_sub_;
// ros::Publisher wheel_pub_;
	ros::Publisher odom_pub_, modelstate_pub_;
// double wheel_vel_[4];
	double encoder_m_s_[4];
	double x_m_, y_m_, w_rad_;
};
