/** include the libraries you need in your planner here */
/** for global path planner interface */
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <Eigen/Dense>
#include <vector>

using std::string;

#ifndef GLOBAL_PLANNER_CPP
#define GLOBAL_PLANNER_CPP

namespace path_planner
{
    double two_points_distance(const geometry_msgs::PoseStamped &point_one, const geometry_msgs::PoseStamped &point_two);
    std::vector<string> my_split(string str, string s);
    void load_data(std::string path_file, std::vector<geometry_msgs::PoseStamped>& vc);
    geometry_msgs::PoseStamped get_pose_now();

    class GlobalPlanner : public nav_core::BaseGlobalPlanner
    {
    public:
        // 判断是否为同一次规划（在同一次规划内作滑动窗口处理）
        geometry_msgs::PoseStamped is_the_same_plan;
        // 滑动窗口的指针
        std::vector<geometry_msgs::PoseStamped>::const_iterator plan_window_it;
        // 存储路径
        std::vector<geometry_msgs::PoseStamped> path_poses;
        GlobalPlanner();
        GlobalPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros);

        

        /** overridden classes from interface nav_core::BaseGlobalPlanner **/
        void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros);
        bool makePlan(const geometry_msgs::PoseStamped &start,
                      const geometry_msgs::PoseStamped &goal,
                      std::vector<geometry_msgs::PoseStamped> &plan);
    };
};
#endif