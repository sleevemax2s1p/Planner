# Planner
运行在 **ubuntu18.04,16.04**系统下
# How to start
快速启动
```
mkdir -p course/src
cd course/src
git clone https://github.com/sleevemax2s1p/Planner.git
cd ..
catkin_make
source devel/setup.bash
```
# How to launch
ROS启动文件
```
roslaunch planner planner.launch
``` 
# 协作实现
见文件头 **include/planner/planner.h** 
```
struct Point{
    int x,y,z;
    int F,G,H;
    Point* parent;
    Point(int _x,int _y,int _z):x(_x),y(_y),z(_z),F(0),G(0),H(0),parent(NULL){};
    
};
``` 
实现算法时继承头文件中的  **Abstract_planer**
并实现其中的虚方法 **getpath(Point *start_point,Point *end_point,double &time,int &distance)****
```
virtual std::list<Point*>* getPath(Point *start_point,Point *end_point,double &time,int &distance){};
``` 
**time**用于存储规划所用时间
**distance**存储规划所得路径的长度
# 地图表示
0代表无障碍物，1代表有障碍物
# 路径可视化
参考 **visualization.h**
```
void Visualization::Init()
void Visualization::draw_path(std::list<Point*>path)
``` 