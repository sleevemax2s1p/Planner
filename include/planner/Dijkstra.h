#ifndef _DIJKSTRA_
#define _DIJKSTRA_

#include <ros/ros.h>
#include "planner.h"
#include <list>
#include <vector>
#include <queue>
#include <cmath>
#include <iostream>
#include <ctime>

using namespace std;
/*raw use raw dikstra
  opt1: use map optimize to get less node
  opt2: use heap optimize
  opt3: use heap optimize and map optimize

*/
// struct djspoint
// {
//     double x,y,z;
//     djspoint(double _x =0, double _y = 0, double _z = 0):x(_x), y(_y), z(_z){}
//     djspoint(const Point &p)
//     {
//         x=p.x;
//         y=p.y;
//         z=p.z;
//     }
//     bool operator <(const Point &a) const
//     {
//         return x==a.x?(y==a.y?z<a.z:y<a.y):x<a.x; 
//     }
//     bool operator <=(const Point &a) const
//     {
//         return x==a.x?(y==a.y?z<=a.z:y<a.y):x<a.x;
//     }
//     bool operator >(const Point &a) const
//     {
//         return x==a.x?(y==a.y?z>a.z:y>a.y):x>a.x;
//     }
//     bool operator >=(const Point &a) const
//     {
//         return x==a.x?(y==a.y?z>=a.z:y>a.y):x>a.x;
//     }
//     bool operator ==(const Point &a) const
//     {
//         return x==a.x&&y==a.y&&z==a.z;
//     }
//     bool operator !=(const Point &a) const
//     {
//         return !(*this==a);
//     }
// };


struct qnode
{
    int i;
    double c;
    qnode(int _i=0, double _c=0):i(_i),c(_c){}
    bool operator<(const qnode &b) const
    {
        return c>b.c;
    }
};

struct Edge
{
    int v;
    double cost;
    Edge(int _v=0, double _cost=0):v(_v),cost(_cost){}
};


// #define DEBUG_INFO 1

static const double mysqrt[]={0,1,sqrt(2),sqrt(3)};
class Dijkstra : public Abstract_planner
{
public:
    bool InitDijkstra_raw(const vector<vector<vector<int>>> &_map);
    bool InitDijkstra_opt1(const vector<vector<vector<int>>> &_map);
    bool InitDijkstra_opt2(const vector<vector<vector<int>>> &_map);
    bool InitDijkstra_opt3(const vector<vector<vector<int>>> &_map);
    list<Point *> *getPath_raw(Point *start_point, Point *end_point, double &time, double &distance);
    list<Point *> *getPath_opt1(Point *start_point, Point *end_point, double &time, double &distance);
    list<Point *> *getPath_opt2(Point *start_point, Point *end_point, double &time, double &distance);
    list<Point *> *getPath_opt3(Point *start_point, Point *end_point, double &time, double &distance);
    vector<Point *> getSurroundPoints(const Point *point) const;
    bool FindPath_raw(const Point &start_point, const Point &end_point);
    bool FindPath_opt1(const Point &start_point, const Point &end_point);
    bool FindPath_opt2(const Point &start_point, const Point &end_point);
    bool FindPath_opt3(const Point &start_point, const Point &end_point);
    bool isCanreach(const Point *point, const Point *target) const;
    Point *isInList(const std::list<Point *> &list, const Point *point) const;
    Point *getLeastFpoint();
    int Point_to_node(const Point &p);
    inline void node_to_Point(Point &p, const int node);
    bool clear();
    bool getAllPath(vector<list<Point *>> &vlp, const Point &start_point, const Point &end_point);
    int calcG(Point *temp_start, Point *end);
    int calcH(Point *start, Point *end);
    int calcF(Point *point);

private:
    bool bulidgraph_raw(const vector<vector<vector<int>>> &_map);
    bool bulidgraph_opt1(const vector<vector<vector<int>>> &_map);
    bool bulidgraph_opt2(const vector<vector<vector<int>>> &_map);
    bool bulidgraph_opt3(const vector<vector<vector<int>>> &_map);
    inline int point_to_node(const int x, const int y, const int z);
    inline bool postion_check(const int x, const int y, const int z);
    inline void resize_vectors(const int siz);

    vector<double> lowcost;
    vector<list<Point>> path;
    vector<int> path_to_node;
    vector<bool> vis;
    vector<int> pre;
    vector<vector<double>> cost;
    vector<vector<Edge>>edgegraph;
    list<Point *> path_one;
    list<Point *> openlist;
    list<Point *> closelist;
    // map<djspoint, int> mp;
    vector<int> i2p;    //int to point
    vector<int> p2i;    //point to int
 
    bool isInit = false;
    bool isClear = false;
    int x_size = 0, y_size = 0, z_size = 0;
    int graph_size = 0;
    const double graph_max_size = 1e6;
};

#endif
