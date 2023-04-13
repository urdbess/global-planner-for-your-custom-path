功能：记录小车路径&小车将记录路径作为全局规划

path_plan为ros功能包；
依赖：Eigen3库，安装命令：sudo apt-get install libeigen3-dev
重要文件在src文件夹中：
pathRecord_node.cpp为记录路径的节点文件，其将小车路径记录为path.txt文件，供全局路径规划器读取；
path_planner.cpp为全局路径规划器（插件形式）；

path_plan/launch/include/move_base.launch中：
将参数"base_global_planner"改为"path_planner/GlobalPlanner"即可使用；
"path_window_size"是滑动窗口大小，即每次规划器发布一段path_window_size长度的路径；
"planner_frequency"为路径发布频率；

附演示视频show_video.mp4；

存在问题：当路径存在间隔较小或交叉的部分时，小车会重新导航至先前的路径中，从而陷入局部循环，详见 问题.mp4；（问题已修复：利用滑动窗口每隔一段时间发布一定步长的路径，从而避免小车走回走过的路;一定程度避免小车走最短路径而不遵循全局路径;避免在闭环路径中小车在起点直接完成导航目标）


具体使用流程：
启动gazebo仿真环境
roslaunch path_plan test_world.launch

启动move_base和pathRecord_node节点
roslaunch path_plan navigation.launch map_name:=nav_map

开始记录路径
rosservice call /recordStart 1

操控机器人走动
rosrun teleop_twist_keyboard teleop_twist_keyboard.py 

结束记录路径
rosservice call /recordStart 0

操控小车回到路径起点附近，指定任意目标点开始导航，小车会将记录路径作为全局路径进行导航；
