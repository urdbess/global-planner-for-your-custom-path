#include <pluginlib/class_list_macros.h>
#include <tf/transform_datatypes.h>
#include <fstream>
#include <boost/regex.hpp>
#include "nav_msgs/Path.h"
#include "path_planner.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(path_planner::GlobalPlanner, nav_core::BaseGlobalPlanner)

using namespace std;

// Default Constructor
namespace path_planner
{
    double two_points_distance(const geometry_msgs::PoseStamped &point_one, const geometry_msgs::PoseStamped &point_two)
    {
        Eigen::Vector2d one;
        Eigen::Vector2d two;
        one.x() = point_one.pose.position.x;
        one.y() = point_one.pose.position.y;
        two.x() = point_two.pose.position.x;
        two.y() = point_two.pose.position.y;
        return (one - two).norm();
    }

    vector<string> my_split(string str, string s)
    {
        boost::regex reg(s.c_str());
        vector<std::string> vec;
        boost::sregex_token_iterator it(str.begin(), str.end(), reg, -1);
        boost::sregex_token_iterator end;
        while (it != end)
        {
            vec.push_back(*it++);
        }
        return vec;
    }

    void load_data(string path_file, vector<geometry_msgs::PoseStamped> &vc)
    {
        ROS_WARN("loading data-------");
        // 清空路径
        vc.clear();

        // 读取路径文件
        ifstream infile;
        infile.open(path_file, ios::in | ios::binary);
        if (!infile)
        {
            ROS_WARN("Failed to open file!");
            return;
        }

        // 将路径点保存至vector
        string s;
        while (getline(infile, s))
        {
            geometry_msgs::PoseStamped pos;
            pos.header.frame_id = "map";
            double x = atof(my_split(s, " ")[0].c_str());
            double y = atof(my_split(s, " ")[1].c_str());
            double ox = atof(my_split(s, " ")[2].c_str());
            double oy = atof(my_split(s, " ")[3].c_str());
            double oz = atof(my_split(s, " ")[4].c_str());
            double ow = atof(my_split(s, " ")[5].c_str());
            pos.pose.position.x = x;
            pos.pose.position.y = y;
            pos.pose.position.z = 0;
            pos.pose.orientation.x = ox;
            pos.pose.orientation.y = oy;
            pos.pose.orientation.z = oz;
            pos.pose.orientation.w = ow;
            vc.push_back(pos);
        }
        infile.close();
        ROS_WARN("Loaded successfully!");
    }

    geometry_msgs::PoseStamped get_pose_now()
    {
        // 监听当前pose
        tf2_ros::Buffer buffer;
        tf2_ros::TransformListener listener(buffer);
        geometry_msgs::TransformStamped transformStamped;
        ros::Time now = ros::Time::now();
        transformStamped = buffer.lookupTransform("map", "base_footprint", ros::Time(0), ros::Duration(1));
        geometry_msgs::PoseStamped pose_cur;
        pose_cur.pose.position.x = transformStamped.transform.translation.x;
        pose_cur.pose.position.y = transformStamped.transform.translation.y;
        pose_cur.pose.orientation.x = transformStamped.transform.rotation.x;
        pose_cur.pose.orientation.y = transformStamped.transform.rotation.y;
        pose_cur.pose.orientation.z = transformStamped.transform.rotation.z;
        pose_cur.pose.orientation.w = transformStamped.transform.rotation.w;
        ROS_INFO("the position now:(%f, %f)", pose_cur.pose.position.x, pose_cur.pose.position.y);
        return pose_cur;
    }

    GlobalPlanner::GlobalPlanner()
    {
    }

    GlobalPlanner::GlobalPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
    {
        initialize(name, costmap_ros);
    }

    void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
    {

        ROS_WARN("initialize-------");
        // 初始化滑动窗口指针
        plan_window_it = path_poses.cbegin();
    }
    // 滑动窗口路径思路：在同一次规划内（发出一次导航目标后，再发出一次导航目标前），按频率根据小车当前位置发布一段向前固定距离的部分路径。
    bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan)
    {
        ros::NodeHandle n;
        ROS_WARN("makePlan-------");
        // 判断是否为同一次规划
        if (goal.header.stamp == is_the_same_plan.header.stamp)
        {
            ROS_WARN("THE SAME PLAN!");
        }
        else
        {
            // 读取路径文件
            // ros::NodeHandle n;
            string pathAddress;
            n.getParam("move_base/path", pathAddress);
            double start_time = ros::Time::now().toSec();
            load_data(pathAddress, path_poses);
            double end_time = ros::Time::now().toSec();
            ROS_INFO("%f s for load", end_time - start_time);

            is_the_same_plan.header.stamp = goal.header.stamp;
            plan_window_it = path_poses.cbegin();
            ROS_WARN("NOT THE SAME PLAN! AND THE NEW PATH IS BEING REPLANNED!");
        }

        geometry_msgs::PoseStamped pose_cur = get_pose_now();
        // 确定距离当前pose第一个最近的路径点作为发布路径的起始点（起始点一路向前滑才能解决在交叉点无法判断前进路径的问题）
        double nearest_dist = two_points_distance(pose_cur, *plan_window_it);
        for (auto it = plan_window_it + 1; it != path_poses.cend(); it++)
        {
            double last_dist = two_points_distance(*it, pose_cur);
            if (last_dist < nearest_dist)
            {
                nearest_dist = last_dist;
                plan_window_it = it;
            }
            else
            {
                // 找到第一个最近的路径点就可以不往下找了，防止找到接下来可能存在的交叉点
                break;
            }
        }
        ROS_INFO("THE NEAREST PATH POSE:(%f, %f)", plan_window_it->pose.position.x, plan_window_it->pose.position.y);

        // 发布一段path_window_size长的全局路径(m)
        double path_window_size;
        n.getParam("move_base/path_window_size", path_window_size);
        double length_sum = 0;
        auto tmp_it = plan_window_it;
        while (length_sum < path_window_size && tmp_it != path_poses.cend())
        {
            length_sum += two_points_distance(*tmp_it, *(tmp_it + 1));
            plan.push_back(*tmp_it);
            tmp_it++;
        }

        //    //将路径点导入plan（一次性全部导入）
        //     for(std::size_t i = 0; i < path_poses.size(); i++) {
        //         plan.push_back(path_poses[i]);
        //     }

        return true;
    }
};