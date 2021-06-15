#ifndef __PLANNER__
#define __PLANNER__

#include<iostream>
#include<math.h>
#include<vector>
#include<list>
using namespace std;
struct Point{
    int x,y,z;
    int F,G,H;
    
    Point* parent;
    Point(int _x,int _y,int _z):x(_x),y(_y),z(_z),F(0),G(0),H(0),parent(NULL){};
    
};
 class Abstract_planner{
    public:
        std::vector<std::vector<std::vector<int>>>map;
        std::vector<Point*> path;
        virtual std::list<Point*>* getPath(Point *start_point,Point *end_point,double &time,double &distance){};
};


#endif
