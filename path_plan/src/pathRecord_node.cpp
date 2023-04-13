#include "ros/ros.h"
#include "std_msgs/String.h"
#include <fstream>
#include <Eigen/Dense>
#include "path_plan/RecordStart.h"
#include "nav_msgs/Path.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//计数器
int count = 0;
//记录路径标记位
bool startFlag;
//记录文件
std::ofstream outfile;
std::string pathAddress;
//路径消息
nav_msgs::Path path;
//服务回调函数
bool startSwitch(path_plan::RecordStart::Request& req, 
            path_plan::RecordStart::Response& res) {
    if (req.start == 1) {
        outfile.open(pathAddress, std::ios::binary | std::ios::out | std::ios::trunc);
        path.poses.clear();
        startFlag = true;
        ROS_INFO("开始记录路径");
    } else if (req.start == 0) {
        count = 0;
        startFlag = false;
        ROS_INFO("结束记录路径");
        outfile.close();
    }
    res.result = 1;
    return true;
}

int main(int argc, char *argv[]) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "pathRecord_node");
    ros::NodeHandle n;

    //实例化话题发布者
    ros::Publisher pub = n.advertise<nav_msgs::Path>("path", 100);

    //实例化服务者
    ros::ServiceServer server = n.advertiseService("recordStart", startSwitch);

    //路径话题发布
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "map";
    
    //监听tf
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);
    n.getParam("pathRecord_node/path", pathAddress);

    //频率：10次每秒
    ros::Rate r(10);

    startFlag = false;

    Eigen::Vector2d cur;
    Eigen::Vector2d pre;
    while (ros::ok()) {
        geometry_msgs::TransformStamped transformStamped;
        
        try {
            //获取当前位置
            ros::Time now = ros::Time::now();
            transformStamped = buffer.lookupTransform("map", "base_footprint", ros::Time(0), ros::Duration(1));
            double x = transformStamped.transform.translation.x;
            double y = transformStamped.transform.translation.y;
            double ox = transformStamped.transform.rotation.x;
            double oy = transformStamped.transform.rotation.y;
            double oz = transformStamped.transform.rotation.z;
            double ow = transformStamped.transform.rotation.w;

            //第一个位置不需要判断
            if (count == 0) {
                pre.x() = x;
                pre.y() = y;
                
                ROS_INFO("position: %f, %f", x, y);
                outfile << x << ' ' << y <<' ' << ox << ' ' << oy << ' ' << oz << ' ' << ow << "\n";
            }
            
            cur.x() = x;
            cur.y() = y;

            //当位移超过0.1m才记录路径点
            Eigen::Vector2d diff = cur - pre;
            double distance = diff.norm();
            if (startFlag && distance > 0.1) {
                ROS_INFO("position: %f, %f", x, y);
                outfile << x << ' ' << y <<' ' << ox << ' ' << oy << ' ' << oz << ' ' << ow << "\n";


                //构建路径消息
                geometry_msgs::PoseStamped pose;
                pose.header.stamp = ros::Time::now();
                pose.header.frame_id = "map";
                pose.pose.position.x = x;
                pose.pose.position.y = y;
                pose.pose.position.z = 0;
                pose.pose.orientation.x = ox;
                pose.pose.orientation.y = oy;
                pose.pose.orientation.z = oz;
                pose.pose.orientation.w = ow;
                //将路径点放进路径并发布
                path.poses.push_back(pose);
                pub.publish(path);

                pre.x() = x;
                pre.y() = y;
            }
            

        }
        catch(const std::exception& e) {
            ROS_INFO("error:%s", e.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        
        r.sleep();
        ros::spinOnce();
        count++;
        
    }

    return 0;
}