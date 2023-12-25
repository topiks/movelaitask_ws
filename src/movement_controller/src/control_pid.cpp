#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_datatypes.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>

#include "movement_controller/pid/pid_controller.h"

// =====================================================

void cllbck_sub_odometry(const nav_msgs::Odometry::ConstPtr &msg);
void cllbck_tim_10hz(const ros::TimerEvent &event);
bool move_to_goal(double x, double y);
void publish_marker_waypoints();

// -----------------------------------------------------

ros::Subscriber sub_odometry;

ros::Publisher pub_cmd_vel;
ros::Publisher pub_text_pos_x;
ros::Publisher pub_text_pos_y;
ros::Publisher pub_text_pos_theta;
ros::Publisher pub_marker_waypoints;

ros::Timer tim_10_hz;

// -----------------------------------------------------

double max_linear_vel;
double max_angular_vel;
float kp_vel;
float ki_vel;
float kd_vel;
float kp_theta;
float ki_theta;
float kd_theta;

double pos_x;
double pos_y;
double pos_theta;

float waypoint[5][2];

PIDController pid_linear_;
PIDController pid_angular_;

int status_algorithm = 0;

// -----------------------------------------------------
// =====================================================

int main(int argc, char **argv)
{
    ros::init(argc, argv, "control_pid");

    ros::NodeHandle NH;
    ros::AsyncSpinner AS(0);

    // -----------------------------

    NH.getParam("/velocity/linear", max_linear_vel);
    NH.getParam("/velocity/angular", max_angular_vel);
    NH.getParam("/pid_vel/kp", kp_vel);
    NH.getParam("/pid_vel/ki", ki_vel);
    NH.getParam("/pid_vel/kd", kd_vel);
    NH.getParam("/pid_theta/kp", kp_theta);
    NH.getParam("/pid_theta/ki", ki_theta);
    NH.getParam("/pid_theta/kd", kd_theta);
    NH.getParam("/point_1/x", waypoint[0][0]);
    NH.getParam("/point_1/y", waypoint[0][1]);
    NH.getParam("/point_2/x", waypoint[1][0]);
    NH.getParam("/point_2/y", waypoint[1][1]);
    NH.getParam("/point_3/x", waypoint[2][0]);
    NH.getParam("/point_3/y", waypoint[2][1]);
    NH.getParam("/point_4/x", waypoint[3][0]);
    NH.getParam("/point_4/y", waypoint[3][1]);
    NH.getParam("/point_5/x", waypoint[4][0]);
    NH.getParam("/point_5/y", waypoint[4][1]);

    for(int i = 0; i < 5; i++)
    {
        waypoint[i][0] /= 100;
        waypoint[i][1] /= 100;
    }

    pid_linear_.initPID(kp_vel, ki_vel, kd_vel);
    pid_angular_.initPID(kp_theta, ki_theta, kd_theta);

    // -----------------------------

    sub_odometry = NH.subscribe("/odom", 1, cllbck_sub_odometry);

    pub_cmd_vel = NH.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    pub_text_pos_x = NH.advertise<visualization_msgs::Marker>("/text_pos_x", 1);
    pub_text_pos_y = NH.advertise<visualization_msgs::Marker>("/text_pos_y", 1);
    pub_text_pos_theta = NH.advertise<visualization_msgs::Marker>("/text_pos_theta", 1);
    pub_marker_waypoints = NH.advertise<visualization_msgs::Marker>("/marker_waypoints", 1);

    tim_10_hz = NH.createTimer(ros::Duration(0.1), cllbck_tim_10hz);

    // -----------------------------

    AS.start();
    ros::waitForShutdown();

    return 0;
}

// ------------------------------------------------
// =================================================

void cllbck_tim_10hz(const ros::TimerEvent &event)
{
    publish_marker_waypoints();

    std::cout << "status_algorithm: " << status_algorithm << std::endl;

    switch (status_algorithm)
    {
        case 0:
            if(move_to_goal(waypoint[0][0], waypoint[0][1]))
            {
                status_algorithm = 1;
            }
        break;

        case 1:
            if(move_to_goal(waypoint[1][0], waypoint[1][1]))
            {
                status_algorithm = 2;
            }
        break;

        case 2:
            if(move_to_goal(waypoint[2][0], waypoint[2][1]))
            {
                status_algorithm = 3;
            }
        break;

        case 3:
            if(move_to_goal(waypoint[3][0], waypoint[3][1]))
            {
                status_algorithm = 4;
            }
        break;

        case 4:
            if(move_to_goal(waypoint[4][0], waypoint[4][1]))
            {
                status_algorithm = 0;
            }
        break;
        
        case 99:
            geometry_msgs::Twist cmd_vel_msg;
            cmd_vel_msg.linear.x = 0;
            cmd_vel_msg.angular.z = 0;
            pub_cmd_vel.publish(cmd_vel_msg);
        break;
    }
}

// ------------------------------------------------

void cllbck_sub_odometry(const nav_msgs::Odometry::ConstPtr &msg)
{
    pos_x = msg->pose.pose.position.x;
    pos_y = msg->pose.pose.position.y;

    // -----------------------------

    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );
    tf::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, pos_theta);
}

// ------------------------------------------------

bool move_to_goal(double x, double y)
{
    double error_x = x - pos_x;
    double error_y = y - pos_y;
    double distance_to_target = sqrt(error_x * error_x + error_y * error_y);
    double target_orientation = atan2(error_y, error_x);
    double error_theta = target_orientation - pos_theta;

    // -----------------------------

    if (error_theta > M_PI) {
        error_theta -= 2 * M_PI;
    } else if (error_theta < -M_PI) {
        error_theta += 2 * M_PI;
    }

    // -----------------------------

    double linear_control = pid_linear_.calculate(distance_to_target);
    double angular_control = pid_angular_.calculate(error_theta);

    if(linear_control > max_linear_vel)
        linear_control = max_linear_vel;
    else if(linear_control < -max_linear_vel)
        linear_control = -max_linear_vel;
    
    if(angular_control > max_angular_vel)
        angular_control = max_angular_vel;
    else if(angular_control < -max_angular_vel)
        angular_control = -max_angular_vel;

    // -----------------------------

    geometry_msgs::Twist cmd_vel_msg;
    cmd_vel_msg.linear.x = linear_control;
    cmd_vel_msg.angular.z = angular_control;
    pub_cmd_vel.publish(cmd_vel_msg);

    std::cout << "kecepatan linear: " << linear_control << std::endl;
    std::cout << "kecepatan angular " << angular_control << std::endl;
    std::cout << "===========" << std::endl;

    // -----------------------------

    if(distance_to_target <= 0.1)
        return true;
    else
        return false;
}

// ------------------------------------------------

void publish_marker_waypoints()
{
    for (int i = 0; i < 5; i++)
    {
        visualization_msgs::Marker marker_waypoints;
        marker_waypoints.header.frame_id = "odom";
        marker_waypoints.header.stamp = ros::Time();
        marker_waypoints.ns = "marker_waypoints";
        marker_waypoints.id = i;
        marker_waypoints.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker_waypoints.action = visualization_msgs::Marker::ADD;
        marker_waypoints.pose.position.x = waypoint[i][0];
        marker_waypoints.pose.position.y = waypoint[i][1];
        marker_waypoints.pose.position.z = 0.5;
        marker_waypoints.pose.orientation.x = 0.0;
        marker_waypoints.pose.orientation.y = 0.0;
        marker_waypoints.pose.orientation.z = 0.0;
        marker_waypoints.pose.orientation.w = 1.0;
        marker_waypoints.scale.x = 0.3;
        marker_waypoints.scale.y = 0.3;
        marker_waypoints.scale.z = 0.3;
        marker_waypoints.color.a = 1.0;
        marker_waypoints.color.r = 0.0;
        marker_waypoints.color.g = 0.0;
        marker_waypoints.color.b = 0.0;
        std::string str = std::to_string(i + 1);
        marker_waypoints.text = str;
        pub_marker_waypoints.publish(marker_waypoints);
    }
}