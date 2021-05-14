#ifndef _VISUALIZATION__
#define _VISUALIZATION_


#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include "planner.h"
class Visualization{
    public:
        ros::Publisher path_pub;

        void Init(ros::NodeHandle &nh);
        void draw_path(std::vector<Point*>path);
};
void Visualization::Init(ros::NodeHandle &nh){
    path_pub = nh.advertise<nav_msgs::Path>("map/path",10);
}
void Visualization::draw_path(std::vector<Point*>path){
    geometry_msgs::PoseStamped pose;

    nav_msgs::Path p;
    p.header.frame_id = "world";
    p.header.stamp = ros::Time().now();

    Point* lastpoint = NULL;
    for(Point* point:path){
        pose.pose.position.x = point->x;
        pose.pose.position.y = point->y;
        pose.pose.position.y = point->y;
        if(lastpoint!=NULL){
            
        }else{
            pose.pose.orientation.w = 1;
            pose.pose.orientation.x = 0;
            pose.pose.orientation.y = 0;
            pose.pose.orientation.z = 0;
        }
        lastpoint = point;
        p.poses.push_back(pose);
    }

    path_pub.publish(p);
}
#endif