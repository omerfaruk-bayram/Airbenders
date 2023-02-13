#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>

#define ALTITUDE 10
#define THRESHOLD 0.5
#define LEN 20.0

struct Point
{
    double x, y;
};

geometry_msgs::PoseStamped curr_pose;
geometry_msgs::PoseStamped target_pose;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    curr_pose = *msg;
}
mavros_msgs::State curr_state;
void state_cb(const mavros_msgs::State::ConstPtr &msg) {
    curr_state = *msg;
}
double distance() {
    double dx = curr_pose.pose.position.x - target_pose.pose.position.x;
    double dy = curr_pose.pose.position.y - target_pose.pose.position.y;
    double dz = curr_pose.pose.position.z - target_pose.pose.position.z;
    return sqrt(dx*dx + dy*dy + dz*dz);
}
bool setpointReached() {
    ROS_INFO("Distance: %f", distance());
	return distance() < THRESHOLD;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "task4_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, pose_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);


    std::vector<Point> coords = {
        {-LEN, LEN}, {-LEN, -LEN}, {LEN, -LEN}, {LEN, LEN}, {-LEN, LEN}
    };

    ros::Rate rate(10.0);


    while(ros::ok() && !curr_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Connected");


    target_pose.pose.position.x = 0;
    target_pose.pose.position.y = 0;
    target_pose.pose.position.z = ALTITUDE;

    for (int i = 100; ros::ok() && i > 0; --i) {
        local_pos_pub.publish(target_pose);
        ros::spinOnce();
        rate.sleep();
    }

    while (ros::ok() && !setpointReached()) {
        if (curr_state.mode != "OFFBOARD") {
            ROS_INFO("Offboard mode is not enabled!");
        }
        else if(!curr_state.armed) {
            ROS_INFO("Vehicle is not armed");
        }
        local_pos_pub.publish(target_pose);
        ros::spinOnce();
        rate.sleep();
    }

    for(int i=0; i < coords.size(); i++) {
        target_pose.pose.position.x = coords[i].x;
        target_pose.pose.position.y = coords[i].y;
        while(ros::ok() && !setpointReached()){
            local_pos_pub.publish(target_pose);
            ros::spinOnce();
            rate.sleep();
        }
    }


}