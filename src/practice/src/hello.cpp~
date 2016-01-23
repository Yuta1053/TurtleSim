#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
using namespace std;

#define PAI atan2(0.0, -1.0)
#define deg(r) ((r)/PAI*180.0)
#define rad(d) ((d)/180.0*PAI)
#define ALLOWANCE 0.0001
#define XMIN 0
#define YMIN 0
#define XMAX 11
#define YMAX 11

class turtleSim{
public:
    turtleSim();
    ~turtleSim();
    void poseCallback(const turtlesim::PoseConstPtr& pose);
    void move(double speed, double distance, bool isForward);
    void l_rotate(double ang_speed, double angle, bool cw);
    double g_rotate(double desired_angle);
    void movePosition(double position_x, double position_y);
    void timerCallback(const ros::TimerEvent&);

private:
    ros::Publisher twist_pub;
    ros::Subscriber pose_sub;
    ros::Timer timer;
    ros::NodeHandle nh;
 
    double po_x, po_y, po_theta;
    int point_num;
    int POINT_NUM_MAX;
};

turtleSim::turtleSim(){
    twist_pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);
    pose_sub = nh.subscribe<turtlesim::Pose>("turtle1/pose", 1, &turtleSim::poseCallback, this);
    timer = nh.createTimer(ros::Duration(0.1), &turtleSim::timerCallback, this);
 
    geometry_msgs::Twist twist;
    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 0.0;
    twist_pub.publish(twist);

 point_num = 0;

 po_x = po_y = 5.5000000;
 po_theta = 0.0;

 ROS_INFO("start");
}
 
turtleSim::~turtleSim(){
}
 
void turtleSim::poseCallback(const turtlesim::PoseConstPtr& pose){
 po_x = pose->x;
 po_y = pose->y;
 po_theta = pose->theta;
}

void turtleSim::timerCallback(const ros::TimerEvent&){
 float point[][2] = {
  {5.5, 5.5},
  {5.5, 10.5},
  {7.5, 7.0},
  {10.5, 7.0},
  {8.0, 4.2},
  {9.3, 0.5},
  {5.5, 2.3},
  {1.7, 0.5},
  {3.0, 4.2},
  {0.5, 7.0},
  {3.5, 7.0},
  {5.5, 10.5},
  {5.5, 5.5}
 };
 POINT_NUM_MAX = sizeof point / sizeof point[0];

 movePosition(point[point_num][0], point[point_num][1]);
}
 
void turtleSim::move(double speed, double distance, bool isForward){
 geometry_msgs::Twist twist;

 if(isForward)
  twist.linear.x = fabs(speed);
 else
  twist.linear.x = -fabs(speed);

 double t0 = ros::Time::now().toSec();
 double current_dis = 0.0;
 
 ros::Rate loop_rate(100);
 do{
  twist_pub.publish(twist);
  double t1 = ros::Time::now().toSec();
  current_dis = speed * (t1 - t0);
  ros::spinOnce();
  loop_rate.sleep();
 }while(current_dis < distance);
 
 twist.linear.x = 0;
 twist_pub.publish(twist); 
}

         
void turtleSim::l_rotate(double ang_speed, double angle, bool cw){
 geometry_msgs::Twist twist;
 if(cw)
  twist.angular.z = -fabs(ang_speed);
 else
  twist.angular.z = fabs(ang_speed);
 
 double t0 = ros::Time::now().toSec();
 double current_ang = 0.0;

 ros::Rate loop_rate(100);
 do{
  twist_pub.publish(twist);
  double t1 = ros::Time::now().toSec();
  current_ang = ang_speed * (t1 - t0);
  ros::spinOnce();
  loop_rate.sleep();
 }while(current_ang < angle);

 twist.angular.z = 0.0;
 twist_pub.publish(twist);
}

 double turtleSim::g_rotate(double desired_angle){
 double relative_angle = 0.0;
 ros::Rate loop_rate(0.5);
 do{
  relative_angle = desired_angle - po_theta;
  bool clockwise = ((relative_angle < 0) ? true:false);
  l_rotate(fabs(relative_angle), fabs(relative_angle), clockwise);
  ros::spinOnce();
  loop_rate.sleep();
 }while(fabs(relative_angle) >= ALLOWANCE);
}

void turtleSim::movePosition(double target_x, double target_y){
 if(target_x <= XMIN) target_x = XMIN;
 if(target_y <= YMIN) target_y = YMIN;
 if(target_x >= XMAX) target_x = XMAX;
 if(target_y >= YMAX) target_y = YMAX;

 double target_theta = atan2( (target_y - po_y), (target_x - po_x) );
 g_rotate(target_theta);
 
 double target_dis = sqrt(pow(target_x - po_x, 2) + pow(target_y - po_y, 2));
 move(target_dis, target_dis, 1);
  point_num++;
 if(point_num > POINT_NUM_MAX - 1)
  point_num = POINT_NUM_MAX - 1;
 }

int main(int argc, char **argv){
    ros::init(argc, argv, "move_turtlesim");
    turtleSim turtlesim;
    ros::spin();
    return 0;
}
