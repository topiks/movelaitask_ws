#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_datatypes.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>

void cllbck_sub_odometry(const nav_msgs::Odometry::ConstPtr &msg);
void cllbck_tim_10hz(const ros::TimerEvent &event);
bool move_to_goal(double x, double y);
void publish_marker_waypoints();

ros::Subscriber sub_odometry;

ros::Publisher pub_cmd_vel;
ros::Publisher pub_circle_L;
ros::Publisher pub_look_ahead_point;
ros::Publisher pub_marker_waypoints;

ros::Timer tim_10_hz;

double max_linear_vel;
double max_angular_vel;
float look_ahead_distance;

double pos_x;
double pos_y;
double pos_theta;
double look_ahead_point_x;
double look_ahead_point_y;

float waypoint[5][2];

int status_algorithm = 0;

// -----------------------------------------------------
// =====================================================

int main(int argc, char **argv)
{
    ros::init(argc, argv, "control_pure_pursuit");

    ros::NodeHandle NH;
    ros::AsyncSpinner AS(0);

    // -----------------------------

    NH.getParam("/velocity/linear", max_linear_vel);
    NH.getParam("/velocity/angular", max_angular_vel);
    NH.getParam("/pure_pursuit/lookahead", look_ahead_distance);
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

    // -----------------------------

    sub_odometry = NH.subscribe("/odom", 1, cllbck_sub_odometry);

    pub_cmd_vel = NH.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    pub_circle_L = NH.advertise<visualization_msgs::Marker>("/circle_L", 1);
    pub_look_ahead_point = NH.advertise<visualization_msgs::Marker>("/look_ahead_point", 1);
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

void cllbck_sub_odometry(const nav_msgs::Odometry::ConstPtr &msg)
{
    pos_x = msg->pose.pose.position.x;
    pos_y = msg->pose.pose.position.y;
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );
    tf::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, pos_theta);

    // publish circle L with L radius with center at robot
    visualization_msgs::Marker circle_L;
    circle_L.header.frame_id = "odom";
    circle_L.header.stamp = ros::Time();
    circle_L.ns = "circle_L";
    circle_L.id = 0;
    circle_L.type = visualization_msgs::Marker::CYLINDER;
    circle_L.action = visualization_msgs::Marker::ADD;
    circle_L.pose.position.x = pos_x;
    circle_L.pose.position.y = pos_y;
    circle_L.pose.position.z = 0;
    circle_L.pose.orientation.x = 0.0;
    circle_L.pose.orientation.y = 0.0;
    circle_L.pose.orientation.z = 0.0;
    circle_L.pose.orientation.w = 1.0;
    circle_L.scale.x = look_ahead_distance * 2;
    circle_L.scale.y = look_ahead_distance * 2;
    circle_L.scale.z = 0.05;
    circle_L.color.a = 0.5;
    circle_L.color.r = 0.0;
    circle_L.color.g = 0.0;
    circle_L.color.b = 1.0;
    pub_circle_L.publish(circle_L);

    // publis look ahead point
    visualization_msgs::Marker look_ahead_point;
    look_ahead_point.header.frame_id = "odom";
    look_ahead_point.header.stamp = ros::Time();
    look_ahead_point.ns = "look_ahead_point";
    look_ahead_point.id = 0;
    look_ahead_point.type = visualization_msgs::Marker::SPHERE;
    look_ahead_point.action = visualization_msgs::Marker::ADD;
    look_ahead_point.pose.position.x = look_ahead_point_x;
    look_ahead_point.pose.position.y = look_ahead_point_y;
    look_ahead_point.pose.position.z = 0;
    look_ahead_point.pose.orientation.x = 0.0;
    look_ahead_point.pose.orientation.y = 0.0;
    look_ahead_point.pose.orientation.z = 0.0;
    look_ahead_point.pose.orientation.w = 1.0;
    look_ahead_point.scale.x = 0.1;
    look_ahead_point.scale.y = 0.1;
    look_ahead_point.scale.z = 0.1;
    look_ahead_point.color.a = 1.0;
    look_ahead_point.color.r = 1.0;
    look_ahead_point.color.g = 0.0;
    look_ahead_point.color.b = 0.0;
    pub_look_ahead_point.publish(look_ahead_point);

}

bool move_to_goal(double x, double y)
{
    // Calculate delta between current position and goal
    double delta_x = x - pos_x;
    double delta_y = y - pos_y;

    double distance_to_goal = sqrt(delta_x * delta_x + delta_y * delta_y);

    if (distance_to_goal > 0.1) 
    { 
        double alpha = atan2(delta_y, delta_x) - pos_theta;
        double curvature_radius = look_ahead_distance / (2 * sin(alpha));
        double desired_angular_vel = max_linear_vel / curvature_radius;
        desired_angular_vel = std::max(-max_angular_vel, std::min(max_angular_vel, desired_angular_vel));
        look_ahead_point_x = pos_x + look_ahead_distance * cos(pos_theta);
        look_ahead_point_y = pos_y + look_ahead_distance * sin(pos_theta);

        // -----------------------------

        geometry_msgs::Twist cmd_vel_msg;
        cmd_vel_msg.linear.x = max_linear_vel; // Forward movement at max linear velocity
        cmd_vel_msg.angular.z = desired_angular_vel;
        pub_cmd_vel.publish(cmd_vel_msg);

        std::cout << "kecepatan linear: " << max_linear_vel << std::endl;
        std::cout << "kecepatan angular " << desired_angular_vel << std::endl;
        std::cout << "===========" << std::endl;

        return false; // Not at the goal yet
    } else {
        // Stop the robot when it reaches the goal
        geometry_msgs::Twist stop_msg;
        stop_msg.linear.x = 0.0;
        stop_msg.angular.z = 0.0;
        pub_cmd_vel.publish(stop_msg);

        return true; // Reached the goal
    }
}

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