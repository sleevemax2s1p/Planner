#ifndef _VISUALIZATION__
#define _VISUALIZATION_


#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "planner.h"
class Visualization{
    public:
        ros::Publisher A_star_path_pub;
        ros::Publisher Jps_path_pub;
        ros::Publisher RRT_path_pub;
        ros::Publisher Dijkstra_path_pub;
        ros::NodeHandle &nh;
        void Init();
        void draw_path(std::list<Point*>path,double r,double g,double b,int type);
        Visualization(ros::NodeHandle &_nh):nh(_nh){};
};
void Visualization::Init(){
    A_star_path_pub = nh.advertise<visualization_msgs::Marker>("map/A_star_path",10);
    Jps_path_pub = nh.advertise<visualization_msgs::Marker>("map/Jps_path",10);
    RRT_path_pub = nh.advertise<visualization_msgs::Marker>("map/RRT_path",10);
    Dijkstra_path_pub = nh.advertise<visualization_msgs::Marker>("map/Dijkstra_path",10);
}
void Visualization::draw_path(std::list<Point*>path,double r,double g,double b,int type){
    visualization_msgs::Marker mk;
    visualization_msgs::Marker line;
    mk.header.frame_id = line.header.frame_id = "world";
    mk.header.stamp = line.header.stamp = ros::Time().now();
    mk.id = 1;
    line.id =2;
    
    mk.action = line.type = visualization_msgs::Marker::ADD;
    mk.type = visualization_msgs::Marker::POINTS;
    line.type = visualization_msgs::Marker::LINE_STRIP;

    line.color.r = r;
    line.color.g = g;
    line.color.b = b;
    line.color.a = 1.0;
    line.scale.x = line.scale.y = line.scale.z = 0.05;

    mk.scale.x = 0.1;
    mk.scale.y = 0.1;
    mk.scale.z = 0.1;

    mk.pose.orientation.w = 1.0;
    line.pose.orientation.w = 1.0;
    mk.color.r = 1.0;
    mk.color.a = 1.0;
    geometry_msgs::Point pt;
    for(auto p:path){
        pt.x = p->x-10;
        pt.y = p->y-7;
        pt.z = p->z-7;

        line.points.push_back(pt);
    }
    if(ros::ok()){
        switch (type)
        {
            
        case 1:
            Jps_path_pub.publish(line);
            break;
        case 2:
            RRT_path_pub.publish(line);
            break;
        case 3:
            Dijkstra_path_pub.publish(line);
            break;
        default:
             A_star_path_pub.publish(line);
            break;
        }
        
        //ROS_INFO("path has been generated");
    }

}
#endif