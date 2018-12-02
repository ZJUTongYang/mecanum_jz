#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <keyboard/Key.h>
#include <math.h>

//mecanum wheel config
//(1)leftfront(2)leftback(3)rightback(4)rightfront
#define lr 0.652
#define fb 0.650
#define r 0.192
const double alpha[4] = {atan2(lr, fb),atan2(lr, -fb), atan2(-lr, -fb), atan2(-lr, fb)};
const double beta[4] = {M_PI/2 - alpha[0], M_PI/2 - alpha[1], 3*M_PI/2 - alpha[2], 3*M_PI/2 - alpha[3]};
const double gam[4] = {-M_PI/4, M_PI/4, -M_PI/4, M_PI/4};

class mecanum
{
public:

mecanum();
~mecanum(){}
void robotCmdCallBack(const geometry_msgs::Twist::ConstPtr& msg);
void keyDownCallBack(const keyboard::Key::ConstPtr& msg);
void publishwheelvel(double vel_0, double vel_1, double vel_2, double vel_3);
void plan(double vx_m_s, double vy_m_s, double w_rad_s);
// inline void getwheelv(double *wheel4)
// {
// 	for(unsigned i = 0; i < 4; i++)
// 		wheel4[i] = wheel_vel_[i];
// }

private:
ros::Subscriber wheel_sub_;
ros::Subscriber key_sub_;
ros::Publisher wheel_pub_;
double wheel_vel_[4];
double vx_m_s_, vy_m_s_, w_rad_s_;
};
