#ifndef _A_STAR_
#define _A_STAR_
#include <ros/ros.h>

#include "planner.h"
#include <list>
using namespace std;
class A_star:public Abstract_planner{
    public:
        void InitAstar(vector<vector<vector<int>>> &_map);
        
        
        std::list<Point*>* getPath(Point *start_point,Point *end_point,double& time,double &distance);
        std::vector<Point*>getSurroundPoints(const Point *point)const;
        Point* FindPath(Point &start_point,Point &end_point);
        bool isCanreach(const Point* point,const Point* target)const;
        Point* isInList(const std::list<Point*>& list,const Point* point)const;
        Point* getLeastFpoint();

        int calcG(Point *temp_start,Point *end);
        int calcH(Point *start,Point *end);
        int calcF(Point *point);

    private:
        std::list<Point*> openlist;
        std::list<Point*> closelist;

};

Point* A_star::getLeastFpoint(){
    if(!openlist.empty()){
        auto resPoint = openlist.front();
        for(auto& point:openlist)
        if(point->F<resPoint->F)
            resPoint = point;
        return resPoint;
    }
    return NULL;
}
int A_star::calcG(Point *temp_start,Point *end){
    int deltaX = temp_start->x-end->x;
    int deltaY = temp_start->y-end->y;
    int deltaZ = temp_start->z-end->z;
    int extraG = std::sqrt((double)(deltaX*deltaX)+(double)(deltaY*deltaY)+double(deltaZ*deltaZ));
    int parentG = end->parent==NULL?0:end->parent->G;
    return parentG+extraG;
}

int A_star::calcH(Point *start,Point *end){
    int deltaX = start->x-end->x;
    int deltaY = start->y-end->y;
    int deltaZ = start->z-end->z;
    return std::abs(deltaX)+std::abs(deltaY)+std::abs(deltaZ);
}
int A_star::calcF(Point *point){
    return point->G+point->H;
}
void A_star::InitAstar(vector<vector<vector<int>>> &_map){
    vector<vector<vector<int>>> realmap(_map.size(),vector<vector<int>>(_map[0].size(),vector<int>(_map[0][0].size())));
    for(int i=0;i<_map.size();i++)
        for(int j=0;j<_map[0].size();j++)
            for(int k=0;k<_map[0][0].size();k++){
                if(_map[i][j][k]==1){
                    realmap[i][j][k] = 1;
                }
            };
    map = realmap;
}
Point* A_star::FindPath(Point &start_point,Point &end_point){
    
    std::cout<<"FindPath start"<<endl;
    openlist.push_back(new Point(start_point.x,start_point.y,start_point.z));
    int i=0;
    
     while(!openlist.empty()){
        
        
        // if(i>50)break;
        auto curPoint = getLeastFpoint();
        openlist.remove(curPoint);
        closelist.push_back(curPoint);

        auto surroundPoints = getSurroundPoints(curPoint);
        for(auto& target:surroundPoints)
        {
            if(!isInList(openlist,target)){
                target->parent = curPoint;

                target->G = calcG(curPoint,target);
                target->H = calcH(target,&end_point);
                target->F = calcF(target);

                openlist.push_back(target);
            }
            else
            {
                int tempG = calcG(curPoint,target);
                if(tempG<target->G){
                    target->parent = curPoint;

                    target->G = tempG;
                    target->F = calcF(target);
                }
            }
            Point *resPoint = isInList(openlist,&end_point);
            if(resPoint)
                return resPoint;
        }
    }
    return NULL;
}

Point* A_star::isInList(const std::list<Point*>& list,const Point* point)const{
    for(auto p:list)
        if(p->x==point->x&&p->y==point->y&&p->z ==point->z)
            return p;
    return NULL;
}
bool A_star::isCanreach(const Point* point,const Point* target)const{
    if(target->x<0||target->y<0||target->z<0||target->x>=map.size()||target->y>=map[0].size()||target->z>=map[0][0].size()
        ||map[target->x][target->y][target->z]==1||(point->x==target->x&&point->y==target->y&&point->z==target->z)||isInList(closelist,target))
        return false;
    else return true;
}
std::list<Point*>* A_star::getPath(Point *start_point,Point *end_point,double &time,double &distance){
     std::cout<<"ready to findPoint";
     ros::Time start_time_point = ros::Time::now();
     Point* result = FindPath(*start_point,*end_point);
     ros::Time end_time_point = ros::Time::now();
     time = (end_time_point-start_time_point).toSec();
     distance =0;
     std::list<Point*>*path = new std::list<Point*>();
     //ROS_INFO("log");
     //int i=0;
      while(result){
          if(result->parent!=nullptr){
              double del_x = result->x-result->parent->x;
              double del_y = result->y-result->parent->y;
              double del_z = result->z-result->parent->z;
              distance+=sqrt(del_x*del_x+del_y*del_y+del_z*del_z);
          }
        // std::cout<<"i:"<<(++i)<<endl;
        //   if(i==6){
        //     std::cout<<"test"<<result->parent->x<<endl;
        //   }
        //   ROS_INFO("running");
          path->push_back(result);
          
        //   std::cout<<"wa"<<endl;
        //   std::cout<<"test";
        //   std::cout<<((result==NULL)?"result null":"result nonull")<<endl;
          result = result->parent;
            
      }
    openlist.clear();
    closelist.clear();
    return path;
}


vector<Point*> A_star::getSurroundPoints(const Point *point)const{
    vector<Point*>surround_points;
    for(int i=point->x-1;i<=point->x+1;i++)
        for(int j=point->y-1;j<=point->y+1;j++)
            for(int k=point->z-1;k<=point->z+1;k++){
                Point* surround_point = new Point(i,j,k);
                if(isCanreach(point,surround_point)) surround_points.push_back(surround_point);
                else delete surround_point;
            }
    return surround_points;
}
#endif