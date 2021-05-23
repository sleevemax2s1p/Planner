#ifndef _Map_Construct_
#define _Map_Construct_

#include<ros/ros.h>
#include<visualization_msgs/Marker.h>
#include<vector>
using namespace std;

int sx,sy,sz;
int ex,ey,ez;
class Map_Construct
{
private:
    
public:
    visualization_msgs::Marker mk;
    ros::Publisher _map_data_pub;
    vector<vector<vector<int>>> map_data;
    void build();
    Map_Construct(vector<vector<vector<int>>> _map_data,ros::NodeHandle &nh);
    ~Map_Construct();
};



Map_Construct::Map_Construct(vector<vector<vector<int>>> _map_data,ros::NodeHandle &nh):map_data(_map_data)
{
    _map_data_pub = nh.advertise<visualization_msgs::Marker>("map/data",100);

    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.id = 0;
    mk.type = visualization_msgs::Marker::CUBE_LIST;
    mk.action = visualization_msgs::Marker::ADD;

     mk.pose.orientation.x = 0.0;
    mk.pose.orientation.y = 0.0;
    mk.pose.orientation.z = 0.0;
    mk.pose.orientation.w = 1.0;

    mk.color.r = 1.0;
    mk.color.g = 0.5;
    mk.color.b = 0.0;
    mk.color.a = 1.0;

    mk.scale.x=1;
    mk.scale.y=1;
    mk.scale.z=1;
    geometry_msgs::Point pt;
    for(int i=0;i<map_data.size();i++)
        for(int j=0;j<map_data[0].size();j++)
            for(int k=0;k<map_data[0][0].size();k++){
                if(map_data[i][j][k]==1){
                    pt.x = i;
                    pt.y = j;
                    pt.z = k;
                    mk.points.push_back(pt);
                }
            }
    std::string c = std::to_string(mk.points.size());

}

void Map_Construct::build(){
    _map_data_pub.publish(mk);
}

Map_Construct::~Map_Construct()
{
}
#endif