/*
    Date: 2019/06/19
    Author: Xu Yucheng
    Abstract: tog Navigation Part
*/
#include <ros/ros.h>
#include <iostream>
#include <std_msgs/String.h>
#include <string.h>
#include <fstream>
#include <string>
#include <sstream>
//navigation中需要使用的位姿信息头文件
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
//move_base头文件
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseAction.h>
//actionlib头文件
#include <actionlib/client/simple_action_client.h>
#include <stdlib.h>
#include <cstdlib>

// 自定义消息类型
#include <kamerider_control_msgs/Mission.h>
#include <kamerider_control_msgs/Result.h>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class tog_navigation
{
private:
    // 记录未知位置的指示flag
    bool _start_navigation;
    // ros params
    std::string sub_control_back_topic_name;
    std::string pub_nav_result_topic_name;

    // ros subscriber & publisher
    ros::Subscriber sub_control;
    ros::Publisher pub_nav;

    // struct for nav pose
    struct location_pose
    {
        string location_name="default";
        float x=0;
        float y=0;
        float z=0;
        float ori_x=0;
        float ori_y=0;
        float ori_z=0;
        float ori_w=0;
    };

    // Goal pose
    std::string target_name;
    std::string target_type;
    geometry_msgs::Pose goal_pose;
    // vector for nav pose
    vector<tog_navigation::location_pose> nav_poses;

    void set_pose()
    {
        if (nav_poses.size() != 0)
        {
            nav_poses.clear();
        }
        std::ifstream waypoints;
        waypoints.open("/home/kamerider/catkin_ws/src/inspection/waypoints.txt");
        string data, line;
        vector<string> str; 
        tog_navigation::location_pose temp;

        if (waypoints)
        {
            while (getline (waypoints, line))
            {
                stringstream line_data(line);
                str.clear();
                while (getline(line_data, data, ','))
                {
                    str.push_back(data);
                }
                cout << "--------pose information--------" << endl;
                for (int i=0; i<str.size(); i++)
                {
                    cout << str[i] << " ";
                }
                cout << endl;
                temp.location_name = str[0];
                temp.x = atof(str[1].c_str());
                temp.y = atof(str[2].c_str());
                temp.z = atof(str[3].c_str());
                temp.ori_x = atof(str[4].c_str());
                temp.ori_y = atof(str[5].c_str());
                temp.ori_z = atof(str[6].c_str());
                temp.ori_w = atof(str[7].c_str());
                tog_navigation::nav_poses.push_back(temp);
                cout << "location_name: " << str[0] << endl;
                cout << "x: " << atof(str[1].c_str()) << endl;
                cout << "y: " << atof(str[2].c_str()) << endl;
                cout << "z: " << atof(str[3].c_str()) << endl;
                cout << "ori_x: " << atof(str[4].c_str()) << endl;
                cout << "ori_y: " << atof(str[5].c_str()) << endl;
                cout << "ori_z: " << atof(str[6].c_str()) << endl;
                cout << "ori_w: " << atof(str[7].c_str()) << endl;
                cout << "-------------------------------" << endl;
                cout << endl;
             }
        }
        else
        {
            std::cout << "No such waypoints file" << std::endl;
        }
    }

    void controlCallback(const kamerider_control_msgs::Mission& msg)
    {
        if (msg.mission_type == "navigate")
        {
            tog_navigation::target_name = msg.mission_name;
            tog_navigation::target_type = msg.mission_type;
            for(int i=0;i<nav_poses.size();i++)
            {
                if(nav_poses[i].location_name == target_name)
                {
                    cout << "[NOTICE] Now i will go to " << nav_poses[i].location_name << endl;
                    goal_pose.position.x = nav_poses[i].x;
                    goal_pose.position.y = nav_poses[i].y;
                    goal_pose.position.z = nav_poses[i].z;
                    goal_pose.orientation.x = nav_poses[i].ori_x;
                    goal_pose.orientation.y = nav_poses[i].ori_y;
                    goal_pose.orientation.z = nav_poses[i].ori_z;
                    goal_pose.orientation.w = nav_poses[i].ori_w;
                    _start_navigation = true;
                }
            }
        }
    }
public:
    int run(int argc, char** argv)
    {
        ROS_INFO ("--------INIT--------");
        ros::init (argc, argv, "tog_navigation");
        ros::NodeHandle nh;
        // 加载预先设定好的导航点（大概率不需要）
        set_pose();
        tog_navigation::_start_navigation = false;

        // get ros params
        nh.param<std::string>("sub_control_back_topic_name", tog_navigation::sub_control_back_topic_name, "/control_to_nav");
        nh.param<std::string>("pub_nav_result_topic_name", tog_navigation::pub_nav_result_topic_name, "/nav_to_control");

        // initialise subscribers & publishers
        tog_navigation::sub_control = nh.subscribe(sub_control_back_topic_name, 1, &tog_navigation::controlCallback, this);
        tog_navigation::pub_nav = nh.advertise<kamerider_control_msgs::Result>(pub_nav_result_topic_name, 1);

        // set move_base server
        MoveBaseClient mc_("move_base", true);
        move_base_msgs::MoveBaseGoal nav_goal;

        while (ros::ok())
        {
            if(_start_navigation)
            {
                ROS_INFO ("START navigating to TARGET position %s", target_name.c_str());
                nav_goal.target_pose.header.frame_id = "map";
                nav_goal.target_pose.header.stamp = ros::Time::now();
                nav_goal.target_pose.pose = geometry_msgs::Pose (goal_pose);
                while (!mc_.waitForServer (ros::Duration(5.0)))
                {
                    ROS_INFO ("Waiting For the Server...");
                }
                mc_.sendGoal (nav_goal);
                mc_.waitForResult (ros::Duration (40.0));
                if (mc_.getState () == actionlib::SimpleClientGoalState::SUCCEEDED)
                {
                    ROS_INFO ("Successfully reached %s", target_name.c_str());
                    _start_navigation = false;
                    kamerider_control_msgs::Result msg;
                    msg.mission_type = "navigate";
                    msg.result = "success";
                    pub_nav.publish (msg);
                }
            }
            ros::spinOnce();
        }
    }
};

int main(int argc, char** argv)
{
    tog_navigation agent;
    return agent.run(argc, argv);
}
