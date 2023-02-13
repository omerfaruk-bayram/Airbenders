#include <ros/ros.h>
#include <fstream>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>


mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr &msg) {
    current_state = *msg;
}
geometry_msgs::PoseStamped current_pose;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    current_pose = *msg;
}
sensor_msgs::NavSatFix current_glob_pose;
void glob_pose_cb(const sensor_msgs::NavSatFix::ConstPtr &msg) {
    current_glob_pose = *msg;
}
geometry_msgs::TwistStamped current_twist;
void twist_cb(const geometry_msgs::TwistStamped::ConstPtr &msg) {
    current_twist = *msg;
}
geometry_msgs::Vector3 current_accel;
double roll, pitch, yaw;
void imu_cb(const sensor_msgs::Imu::ConstPtr &msg) {
    current_accel = msg->linear_acceleration;
    roll = msg->orientation.x;
    pitch = msg->orientation.y;
    yaw = msg->orientation.z;
}
double cur_thrust;
void att_tar_cb(const mavros_msgs::AttitudeTarget::ConstPtr &msg) {
    cur_thrust = msg->thrust;
}

struct Point {
    double lati_, longi_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, pose_cb);
    ros::Subscriber global_pos_sub = nh.subscribe<sensor_msgs::NavSatFix>
            ("mavros/global_position/raw/fix", 10, glob_pose_cb);
    ros::Subscriber twist_sub = nh.subscribe<geometry_msgs::TwistStamped>
            ("mavros/global_position/raw/gps_vel", 10, twist_cb);
    ros::Subscriber accel_sub = nh.subscribe<sensor_msgs::Imu>
            ("mavros/imu/data", 10, imu_cb);
    ros::Subscriber att_tar_sub = nh.subscribe<mavros_msgs::AttitudeTarget>
            ("mavros/setpoint_raw/target_attitude", 10, att_tar_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    std::ifstream read_coords("/home/faruk/tutorial/src/package2/coords");
    std::vector<Point> Coords;
    std::string line;
    if(read_coords.is_open()) {
        
        while ( getline(read_coords, line) ){
            size_t pos = line.find(',');
            double x = stod(line.substr(1, pos-1));
            double y = stod(line.substr(pos+1, line.size()-2-pos));
            Point p {x, y};
            Coords.push_back(p);
        }
        read_coords.close();
    }

    int altitude = 10;
    
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(10.0);

    // wait for FCU connection
    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Connected");

    geometry_msgs::PoseStamped tkf_pose;
    tkf_pose.pose.position.x = 0;
    tkf_pose.pose.position.y = 0;
    tkf_pose.pose.position.z = altitude;

    ROS_INFO("Sending setpoints before requesting offboard");
    //send a few setpoints before starting
    for (int i = 100; ros::ok() && i > 0; --i) {
        local_pos_pub.publish(tkf_pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();


    while (ros::ok() && current_pose.pose.position.z < altitude - 0.5) {
        if (current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))) {
            if (set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent) {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if (!current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))) {
                if (arming_client.call(arm_cmd) &&
                    arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(tkf_pose);

        ros::spinOnce();
        rate.sleep();
    }

    while(ros::ok()){
        ROS_INFO("Velocity >> x:%f  y:%f  z:%f", current_twist.twist.linear.x, current_twist.twist.linear.y, current_twist.twist.linear.z);
        ROS_INFO("Global >> enlem:%f  boylam:%f  yukseklik:%f", current_glob_pose.latitude, current_glob_pose.longitude, current_glob_pose.altitude);
        for(int i = 0; i < Coords.size(); i++)
            ROS_INFO("Local%d >> enlem:%f  boylam:%f  yukseklik:%f",i+1 ,current_glob_pose.latitude - Coords[i].lati_, current_glob_pose.longitude - Coords[i].longi_, current_pose.pose.position.z);
        ROS_INFO("Acceleratiion x:%f  y:%f", current_accel.x, current_accel.y);
        ROS_INFO("Thrust:%f", cur_thrust);
        ROS_INFO("Roll:%f  Pitch:%f  Yaw:%f ", roll, pitch, yaw);
        ROS_INFO("----------------------------------------------------------");

        ros::spinOnce();
        rate.sleep();
    }
    

    return 0;
}

