#ifndef __PLANNER__
#define __PLANNER__

#include<iostream>
#include<math.h>
#include<vector>
using namespace std;
struct Point{
    int x,y,z;
    int F,G,H;
    Point* parent;
    Point(int _x,int _y,int _z):x(_x),y(_y),z(_z),F(0),G(0),H(0){};
    int operator-(Point *b){
        int delta_x = this->x-b->x;
        int delta_y = this->y-b->y;
        int delta_z = this->z-b->z;
        return (int)sqrt(delta_x*delta_x+delta_y*delta_y+delta_z*delta_z);
    }
};
 class Abstract_planner{
    public:
        std::vector<std::vector<std::vector<int>>>map;
        std::vector<Point*> path;
        virtual std::vector<Point*> getPath(Point &start_point,Point &end_point){};
};


#endif
